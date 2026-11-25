#include "vehicle_model.hpp"
#include <chrono>
#include <cmath>
#include <filesystem>
#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/double_v.pb.h>
#include <gz/sim/Link.hh>
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/LinearVelocityCmd.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/transport/Node.hh>
#include <iostream>
#include <json-c/json.h>
#include <limits>
#include <memory>
#include <stdexcept>
#include <thread>
#include <tinyxml2.h>
#include <unistd.h>

using namespace std::chrono_literals;
namespace fs = std::filesystem;
using Vector = gz::math::Vector3d;
using Command = std::array<double, 9>;
void check(bool condition, const std::string &message) {
  if (!condition)
    throw std::runtime_error(message);
}
Vector flip(const std::array<double, 3> &value) {
  return {value[0], -value[1], -value[2]};
}

struct Snapshot {
  gz::math::Pose3d pose;
  Vector velocity, body_rate_frd;
  double time{};
};

class Observer : public gz::sim::System,
                 public gz::sim::ISystemPreUpdate,
                 public gz::sim::ISystemPostUpdate {
  int initialization_;
  Vector initial_velocity_, cg_;

public:
  Snapshot state;
  Observer(const Vector &velocity, const Vector &cg)
      : initialization_(velocity.Length() == 0 ? 2 : 0),
        initial_velocity_(velocity), cg_(cg) {}
  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override {
    if (info.paused || initialization_ == 2)
      return;
    const auto entity = ecm.EntityByComponents(
        gz::sim::components::Name("base_link"), gz::sim::components::Link());
    if (entity == gz::sim::kNullEntity)
      return;
    if (initialization_++ == 0)
      gz::sim::Link(entity).SetLinearVelocity(ecm, initial_velocity_);
    else
      ecm.RemoveComponent<gz::sim::components::LinearVelocityCmd>(entity);
  }
  void PostUpdate(const gz::sim::UpdateInfo &info,
                  const gz::sim::EntityComponentManager &ecm) override {
    const auto entity = ecm.EntityByComponents(
        gz::sim::components::Name("base_link"), gz::sim::components::Link());
    if (entity == gz::sim::kNullEntity)
      return;
    const gz::sim::Link link(entity);
    const auto pose = link.WorldPose(ecm);
    const auto velocity = link.WorldLinearVelocity(ecm, cg_);
    const auto angular = link.WorldAngularVelocity(ecm);
    if (!pose || !velocity || !angular)
      return;
    const auto flu = pose->Rot().RotateVectorReverse(*angular);
    state = {*pose,
             *velocity,
             {flu.X(), -flu.Y(), -flu.Z()},
             std::chrono::duration<double>(info.simTime).count()};
  }
};

struct Fixture {
  CargadorConfiguracion::Configuracion config;
  std::shared_ptr<Observer> observer;
  std::unique_ptr<gz::sim::Server> server;
  gz::transport::Node node;
  gz::transport::Node::Publisher publisher;
  bool normalized{};

  Fixture(const fs::path &root, const std::string &name,
          bool shifted_cg = false, double timeout = 10,
          const Vector &initial_velocity = Vector::Zero,
          bool use_normalized = false)
      : normalized(use_normalized) {
    std::cout << "CASE " << name << std::endl;
    auto *vehicle = json_object_from_file((root / "vehicle.json").c_str());
    check(vehicle, "load fixture vehicle");
    if (shifted_cg)
      json_object_object_add(json_object_object_get(vehicle, "rigid_body"),
                             "cg_model_m",
                             json_tokener_parse("[0.05,0.02,-0.03]"));
    const auto vehicle_path = root / (name + ".json");
    check(json_object_to_file(vehicle_path.c_str(), vehicle) == 0,
          "write fixture vehicle");
    json_object_put(vehicle);
    config = CargadorConfiguracion::cargarVehiculo(vehicle_path.string());
    tinyxml2::XMLDocument world, model;
    check(world.LoadFile((root / "pvtol.sdf").c_str()) == 0 &&
              model.LoadFile(
                  (root / "models/stallion_vtol/model.sdf").c_str()) == 0,
          "load SDF fixture");
    auto *world_element = world.RootElement()->FirstChildElement("world");
    auto *include = world_element->FirstChildElement("include");
    auto *element = model.RootElement()
                        ->FirstChildElement("model")
                        ->DeepClone(&world)
                        ->ToElement();
    element->SetAttribute("name", "controlled_drone");
    element->InsertFirstChild(
        include->FirstChildElement("pose")->DeepClone(&world));
    world_element->DeleteChild(include);
    world_element->InsertEndChild(element);
    auto *plugin = element->FirstChildElement("plugin");
    plugin->FirstChildElement("vehicle_config")->SetText(vehicle_path.c_str());
    plugin->FirstChildElement("command_timeout_s")->SetText(timeout);
    auto *pose = element->FirstChildElement("link")
                     ->FirstChildElement("inertial")
                     ->FirstChildElement("pose");
    const auto cg = flip(config.cg_model_m);
    const std::string cg_text = std::to_string(cg.X()) + " " +
                                std::to_string(cg.Y()) + " " +
                                std::to_string(cg.Z()) + " 0 0 0";
    pose->SetText(cg_text.c_str());
    const auto world_path = root / (name + ".sdf");
    check(world.SaveFile(world_path.c_str()) == 0, "write SDF fixture");
    gz::sim::ServerConfig settings;
    settings.SetSdfFile(world_path.string());
    std::cout << "  loading world" << std::endl;
    server = std::make_unique<gz::sim::Server>(settings);
    observer = std::make_shared<Observer>(initial_velocity, cg);
    check(server->AddSystem(observer).value_or(false), "add observer");
    std::cout << "  initializing paused world" << std::endl;
    check(server->RunOnce(true), "initialize paused server");
    std::cout << "  waiting for command subscription" << std::endl;
    publisher = normalized ? node.Advertise<gz::msgs::Actuators>(
                                 "/model/controlled_drone/actuators_normalized")
                           : node.Advertise<gz::msgs::Double_V>(
                                 "/model/controlled_drone/actuators");
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while (!publisher.HasConnections() &&
           std::chrono::steady_clock::now() < deadline)
      std::this_thread::sleep_for(10ms);
    check(publisher.HasConnections(), "plant subscription missing");
  }

  void send(const Command &command, size_t count = 9, double source_time = -1) {
    gz::msgs::Double_V message;
    gz::msgs::Actuators normalized_message;
    if (normalized) {
      const double stamp = source_time < 0 ? observer->state.time : source_time;
      const auto ns = static_cast<uint64_t>(std::llround(stamp * 1e9));
      normalized_message.mutable_header()->mutable_stamp()->set_sec(ns /
                                                                    1000000000);
      normalized_message.mutable_header()->mutable_stamp()->set_nsec(
          ns % 1000000000);
    }
    for (size_t i = 0; i < count; ++i) {
      const double fraction = (command[i] - config.actuator_min[i]) /
                              (config.actuator_max[i] - config.actuator_min[i]);
      normalized_message.add_normalized(i < 3 ? fraction : 2 * fraction - 1);
      message.add_data(command[i]);
    }
    for (int i = 0; i < 3; ++i) {
      check(normalized ? publisher.Publish(normalized_message)
                       : publisher.Publish(message),
            "publish actuator command");
      std::this_thread::sleep_for(10ms);
    }
  }

  Snapshot step(unsigned count = 1) {
    check(server->Run(true, count, false), "step physics");
    return observer->state;
  }

  void checkStep(const Command &applied) {
    const auto before = observer->state;
    const auto after = step();
    const double dt = after.time - before.time;
    check(dt > 0 && dt < 0.01, "unexpected physics step");
    const auto flu = before.pose.Rot().RotateVectorReverse(before.velocity);
    const auto wrench =
        pso_runtime::bodyWrench(config, {flu.X(), -flu.Y(), -flu.Z()}, applied);
    const auto acceleration =
        before.pose.Rot().RotateVector(flip(wrench.force)) / config.masa +
        Vector(0, 0, -config.g);
    const auto residual = after.velocity - before.velocity - acceleration * dt;
    std::cout << "  linear residual=" << residual
              << " before=" << before.velocity << " after=" << after.velocity
              << " predicted acceleration=" << acceleration << std::endl;
    check(residual.Length() < 1e-7,
          "Gazebo linear acceleration disagrees with allocator model");
    const auto omega = before.body_rate_frd;
    const Vector inertia(config.inercia[0], config.inercia[1],
                         config.inercia[2]);
    const auto torque =
        Vector(wrench.moment[0], wrench.moment[1], wrench.moment[2]) -
        omega.Cross(inertia * omega);
    const auto expected_rate =
        omega + Vector(torque.X() / inertia.X(), torque.Y() / inertia.Y(),
                       torque.Z() / inertia.Z()) *
                    dt;
    const double angular_error = (after.body_rate_frd - expected_rate).Length();
    std::cout << "  angular rate error=" << angular_error << std::endl;
    check(angular_error < 1e-6,
          "Gazebo angular acceleration disagrees with allocator model");
  }
};

Command hover(const CargadorConfiguracion::Configuracion &c) {
  const double weight = c.masa * c.g, front_x = c.force_positions[3][0],
               rear_x = c.force_positions[5][0];
  const double rear = weight * front_x / (front_x - rear_x);
  return {(weight - rear) / 2 / c.coeficiente_empuje[0],
          (weight - rear) / 2 / c.coeficiente_empuje[1],
          rear / c.coeficiente_empuje[2],
          0,
          0,
          0,
          0,
          0,
          0};
}

int main(int argc, char **argv) {
  try {
    check(argc == 2, "Usage: plant_test PREPARED_DIR");
    const fs::path root = fs::absolute(argv[1]);
    setenv("GZ_PARTITION", ("vtol-plant-" + std::to_string(getpid())).c_str(),
           1);
    {
      Fixture f(root, "freefall");
      f.step();
      f.checkStep({});
      std::cout << "PASS gravity with no command\n";
    }
    for (bool shifted : {false, true}) {
      Fixture f(root, shifted ? "cg_hover" : "hover", shifted);
      const auto command = hover(f.config);
      f.send(command);
      const auto before = f.step();
      const auto after = f.step(100);
      check((after.pose.Pos() - before.pose.Pos()).Length() < 1e-5,
            "hover position drift");
      check(after.body_rate_frd.Length() < 1e-5, "CG hover induces rotation");
      f.checkStep(command);
      std::cout << "PASS balanced hover, shifted CG=" << shifted << '\n';
    }
    for (bool normalized : {false, true})
      for (size_t i = 0; i < 9; ++i) {
        // Aerodynamic terms are exercised after injecting forward body
        // velocity.
        Fixture f(root,
                  "actuator_" + std::to_string(i) +
                      (normalized ? "_normalized" : ""),
                  false, 10, Vector(10, 0, 0), normalized);
        Command command{};
        command[i] = i < 3 ? 500 : 0.1;
        if (i == 3)
          command[0] = 500;
        if (i == 4)
          command[1] = 500;
        f.send(command);
        f.step(3);
        f.checkStep(command);
        std::cout << "PASS actuator " << i
                  << " and body/world frame conversion\n";
      }
    for (bool normalized : {false, true}) {
      Fixture f(root, normalized ? "invalid_normalized" : "invalid", false, 10,
                Vector::Zero, normalized);
      const auto command = hover(f.config);
      f.send(command);
      f.step();
      f.send(command, 8);
      f.checkStep({});
      auto invalid = command;
      invalid[0] = f.config.actuator_max[0] + 1;
      f.send(invalid);
      f.checkStep({});
      invalid[0] = std::numeric_limits<double>::quiet_NaN();
      f.send(invalid);
      f.checkStep({});
      std::cout
          << "PASS malformed, out-of-range and nonfinite commands disarm\n";
    }
    for (bool normalized : {false, true}) {
      Fixture f(root, normalized ? "timeout_normalized" : "timeout", false,
                0.05, Vector::Zero, normalized);
      const auto command = hover(f.config);
      f.send(command);
      f.step();
      std::this_thread::sleep_for(80ms);
      f.checkStep({});
      std::cout << "PASS stale commands disarm\n";
    }
    {
      Fixture f(root, "delayed_normalized", false, 0.05, Vector::Zero, true);
      f.step(100);
      f.send(hover(f.config), 9, 0);
      f.checkStep({});
      f.send(hover(f.config), 9, f.observer->state.time + 1);
      f.checkStep({});
      gz::msgs::Actuators unstamped;
      for (unsigned i = 0; i < 9; ++i)
        unstamped.add_normalized(0.5);
      check(f.publisher.Publish(unstamped), "publish missing timestamp");
      std::this_thread::sleep_for(30ms);
      f.checkStep({});
      std::cout << "PASS delayed and future source timestamps disarm\n";
    }
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
