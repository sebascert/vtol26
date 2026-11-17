#include "pvtol_controller.hh"
#include <gz/plugin/Register.hh>

GZ_ADD_PLUGIN(pvtol::PvtolController, gz::sim::System,
              pvtol::PvtolController::ISystemConfigure,
              pvtol::PvtolController::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(pvtol::PvtolController, "pvtol::PvtolController")
