#include "vehicle_model.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/double_v.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Gravity.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/World.hh>
#include <gz/transport/Node.hh>
#include <iostream>
#include <memory>
#include <mutex>
#include <sdf/Element.hh>
#include <stdexcept>

namespace vtol {
namespace {
gz::math::Vector3d flipYZ(const std::array<double, 3> &value) {
  return {value[0], -value[1], -value[2]};
}
} // namespace

class Plant final : public gz::sim::System,
                    public gz::sim::ISystemConfigure,
                    public gz::sim::ISystemPreUpdate {
  using Clock = std::chrono::steady_clock;
  CargadorConfiguracion::Configuracion config_;
  gz::sim::Entity link_{gz::sim::kNullEntity};
  std::mutex mutex_;
  std::array<double, 9> command_{};
  Clock::time_point received_{};
  bool stamped_{};
  std::chrono::nanoseconds source_time_{};
  double timeout_s_{0.5};
  std::chrono::steady_clock::duration last_sim_time_{};
  gz::transport::Node node_; // Destroy subscriptions before callback state.

  void receiveRaw(const gz::msgs::Double_V &message) { receive(message); }

  void receive(const gz::msgs::Double_V &message,
               const gz::msgs::Time *stamp = nullptr) {
    std::lock_guard<std::mutex> lock(mutex_);
    received_ = {};
    if (message.data_size() != 9)
      return;
    stamped_ = stamp != nullptr;
    if (stamped_) {
      if (stamp->sec() < 0 || stamp->sec() > 9000000000LL ||
          stamp->nsec() < 0 || stamp->nsec() >= 1000000000)
        return;
      source_time_ = std::chrono::seconds(stamp->sec()) +
                     std::chrono::nanoseconds(stamp->nsec());
    }
    std::array<double, 9> candidate;
    std::copy(message.data().begin(), message.data().end(), candidate.begin());
    try {
      pso_runtime::validateActuators(config_, candidate);
    } catch (const std::invalid_argument &) {
      return;
    }
    command_ = candidate;
    received_ = Clock::now();
  }

  void receiveNormalized(const gz::msgs::Actuators &message) {
    gz::msgs::Double_V model;
    if (message.normalized_size() == 9 && message.has_header() &&
        message.header().has_stamp()) {
      for (int i = 0; i < 9; ++i) {
        const double value = message.normalized(i);
        if (!std::isfinite(value) || value > 1 || value < (i < 3 ? 0 : -1)) {
          receive(model);
          return;
        }
        const double fraction = i < 3 ? value : (value + 1) / 2;
        model.add_data(config_.actuator_min[i] +
                       fraction *
                           (config_.actuator_max[i] - config_.actuator_min[i]));
      }
    }
    receive(model, message.has_header() && message.header().has_stamp()
                       ? &message.header().stamp()
                       : nullptr);
  }

public:
  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm,
                 gz::sim::EventManager &) override {
    const gz::sim::Model model(entity);
    const auto path =
        sdf->Get<std::string>("vehicle_config",
                              vehicleConfigDirectory() + "/vehicle.json")
            .first;
    config_ = CargadorConfiguracion::cargarVehiculo(path);
    link_ = model.LinkByName(
        ecm, sdf->Get<std::string>("link_name", "base_link").first);
    if (link_ == gz::sim::kNullEntity)
      throw std::runtime_error("VTOL plant: body link missing");
    timeout_s_ = sdf->Get<double>("command_timeout_s", 0.5).first;
    if (!std::isfinite(timeout_s_) || timeout_s_ <= 0)
      throw std::runtime_error("Invalid command timeout");
    const auto world = ecm.EntityByComponents(gz::sim::components::World());
    const auto *gravity = ecm.Component<gz::sim::components::Gravity>(world);
    if (!gravity ||
        (gravity->Data() - gz::math::Vector3d(0, 0, -config_.g)).Length() >
            1e-9)
      throw std::runtime_error(
          "VTOL plant: SDF/config gravity mismatch; launch simulation/run.sh");
    const auto *component = ecm.Component<gz::sim::components::Inertial>(link_);
    if (!component)
      throw std::runtime_error("VTOL plant: missing inertia");
    const auto &inertia = component->Data();
    const auto diagonal = inertia.MassMatrix().DiagonalMoments();
    if (std::abs(inertia.MassMatrix().Mass() - config_.masa) > 1e-9 ||
        (inertia.Pose().Pos() - flipYZ(config_.cg_model_m)).Length() > 1e-9 ||
        (diagonal - gz::math::Vector3d(config_.inercia[0], config_.inercia[1],
                                       config_.inercia[2]))
                .Length() > 1e-9 ||
        inertia.MassMatrix().OffDiagonalMoments().Length() > 1e-9 ||
        !inertia.Pose().Rot().Equal(gz::math::Quaterniond::Identity, 1e-9))
      throw std::runtime_error(
          "VTOL plant: SDF/config inertia mismatch; launch simulation/run.sh");
    gz::sim::Link(link_).EnableVelocityChecks(ecm);
    const auto topic = "/model/" + model.Name(ecm) + "/actuators";
    if (!node_.Subscribe(topic, &Plant::receiveRaw, this))
      throw std::runtime_error("Cannot subscribe to " + topic);
    if (!node_.Subscribe(topic + "_normalized", &Plant::receiveNormalized,
                         this))
      throw std::runtime_error(
          "Cannot subscribe to normalized actuator commands");
    std::cout << "VTOL plant: " << topic << ", " << config_.vehicle_id
              << " revision " << config_.vehicle_revision << " ("
              << config_.calibration_status << ")\n";
  }

  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override {
    if (info.paused || link_ == gz::sim::kNullEntity)
      return;
    const gz::sim::Link link(link_);
    const auto pose = link.WorldPose(ecm);
    const auto velocity =
        link.WorldLinearVelocity(ecm, flipYZ(config_.cg_model_m));
    if (!pose || !velocity)
      return;
    std::array<double, 9> command;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (info.simTime < last_sim_time_)
        received_ = {};
      last_sim_time_ = info.simTime;
      const bool source_fresh =
          !stamped_ ||
          (source_time_ <= info.simTime &&
           std::chrono::duration<double>(info.simTime - source_time_).count() <
               timeout_s_);
      if (source_fresh && received_ != Clock::time_point{} &&
          std::chrono::duration<double>(Clock::now() - received_).count() <
              timeout_s_)
        command = command_;
      else
        std::copy(config_.actuator_disarmed.begin(),
                  config_.actuator_disarmed.end(), command.begin());
    }
    const auto velocity_flu = pose->Rot().RotateVectorReverse(*velocity);
    const auto wrench = pso_runtime::bodyWrench(
        config_, {velocity_flu.X(), -velocity_flu.Y(), -velocity_flu.Z()},
        command);
    // The force model's moments are about the CG, not the mesh/link origin.
    link.AddWorldWrench(ecm, pose->Rot().RotateVector(flipYZ(wrench.force)),
                        pose->Rot().RotateVector(flipYZ(wrench.moment)),
                        flipYZ(config_.cg_model_m));
  }
};
} // namespace vtol

GZ_ADD_PLUGIN(vtol::Plant, gz::sim::System, vtol::Plant::ISystemConfigure,
              vtol::Plant::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(vtol::Plant, "vtol::Plant")
