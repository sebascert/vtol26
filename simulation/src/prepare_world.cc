#include "config_loader.hpp"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <tinyxml2.h>

namespace fs = std::filesystem;
using tinyxml2::XMLDocument;
using tinyxml2::XMLElement;

XMLElement *child(XMLElement *parent, const char *name) {
  if (!parent)
    throw std::runtime_error("Invalid SDF template");
  auto *result = parent->FirstChildElement(name);
  if (!result) {
    result = parent->GetDocument()->NewElement(name);
    parent->InsertEndChild(result);
  }
  return result;
}

int main(int argc, char **argv) {
  try {
    if (argc != 4)
      throw std::runtime_error(
          "Usage: vtol_prepare VEHICLE_JSON SIMULATION_DIR OUTPUT_DIR");
    const auto config_path = fs::absolute(argv[1]);
    const auto config =
        CargadorConfiguracion::cargarVehiculo(config_path.string());
    const fs::path source = fs::absolute(argv[2]),
                   output = fs::absolute(argv[3]);
    XMLDocument model, world;
    if (model.LoadFile((source / "models/stallion_vtol/model.sdf").c_str()) !=
            tinyxml2::XML_SUCCESS ||
        world.LoadFile((source / "worlds/pvtol.sdf").c_str()) !=
            tinyxml2::XML_SUCCESS)
      throw std::runtime_error("Cannot load model/world templates");
    auto *model_element = child(child(model.RootElement(), "model"), "link");
    auto *inertial = child(model_element, "inertial");
    child(inertial, "mass")->SetText(config.masa);
    std::ostringstream cg;
    cg << std::setprecision(17) << config.cg_model_m[0] << ' '
       << -config.cg_model_m[1] << ' ' << -config.cg_model_m[2] << " 0 0 0";
    child(inertial, "pose")->SetText(cg.str().c_str());
    for (auto *sensor = model_element->FirstChildElement("sensor"); sensor;
         sensor = sensor->NextSiblingElement("sensor"))
      child(sensor, "pose")->SetText(cg.str().c_str());
    auto *inertia = child(inertial, "inertia");
    const char *axes[] = {"ixx", "iyy", "izz"};
    for (size_t i = 0; i < 3; ++i)
      child(inertia, axes[i])->SetText(config.inercia[i]);
    for (const char *name : {"ixy", "ixz", "iyz"})
      child(inertia, name)->SetText(0.0);
    auto *plugin = child(child(model.RootElement(), "model"), "plugin");
    child(plugin, "vehicle_config")->SetText(config_path.c_str());
    std::ostringstream gravity;
    gravity << std::setprecision(17) << "0 0 " << -config.g;
    child(child(world.RootElement(), "world"), "gravity")
        ->SetText(gravity.str().c_str());
    const auto model_dir = output / "models/stallion_vtol";
    fs::create_directories(model_dir);
    const auto snapshot = output / "vehicle.json";
    if (!fs::exists(snapshot) || !fs::equivalent(config_path, snapshot))
      fs::copy_file(config_path, snapshot,
                    fs::copy_options::overwrite_existing);
    child(child(child(model.RootElement(), "model"), "plugin"),
          "vehicle_config")
        ->SetText(snapshot.c_str());
    const auto mesh_source = source / "models/stallion_vtol/meshes";
    const auto mesh_link = model_dir / "meshes";
    if (!fs::exists(mesh_link))
      fs::create_directory_symlink(mesh_source, mesh_link);
    if (!fs::equivalent(mesh_source, mesh_link))
      throw std::runtime_error("Unexpected mesh output directory");
    fs::copy_file(source / "models/stallion_vtol/model.config",
                  model_dir / "model.config",
                  fs::copy_options::overwrite_existing);
    if (model.SaveFile((model_dir / "model.sdf").c_str()) !=
            tinyxml2::XML_SUCCESS ||
        world.SaveFile((output / "pvtol.sdf").c_str()) != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("Cannot write configured SDF");
    std::cout << "Prepared " << output / "pvtol.sdf" << " from " << config_path
              << '\n';
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
