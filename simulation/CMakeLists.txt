cmake_minimum_required(VERSION 3.22)

project(pvtol_gz LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(gz-sim8 REQUIRED)
find_package(tinyxml2 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONC REQUIRED IMPORTED_TARGET json-c)
set(PSO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../companion/allocator/src/pso")
set(VEHICLE_CONFIG "$ENV{HOME}/.local/share/vtol26/config/vehicle.json")

add_library(
  vtol_model
  STATIC ${PSO_DIR}/vehicle_model.cpp ${PSO_DIR}/config_loader.cpp)
set_target_properties(vtol_model PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(vtol_model PUBLIC ${PSO_DIR})
target_link_libraries(vtol_model PUBLIC PkgConfig::JSONC)

add_library(vtol_plant SHARED src/vtol_plant.cc)
target_link_libraries(vtol_plant PRIVATE vtol_model gz-sim8::gz-sim8)

add_executable(vtol_prepare src/prepare_world.cc)
target_link_libraries(vtol_prepare PRIVATE vtol_model tinyxml2::tinyxml2)

include(CTest)
if(BUILD_TESTING)
  add_test(
    NAME hil_protocol
    COMMAND
      "${CMAKE_CURRENT_BINARY_DIR}/venv/bin/python3"
      "${CMAKE_CURRENT_SOURCE_DIR}/tests/hil_test.py")
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(
    NAME visual_assembly
    COMMAND
      ${Python3_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/tests/visual_test.py"
      "${CMAKE_CURRENT_SOURCE_DIR}")
  add_test(
    NAME prepare_config
    COMMAND
      ${Python3_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/tests/prepare_test.py"
      $<TARGET_FILE:vtol_prepare> "${CMAKE_CURRENT_SOURCE_DIR}"
      "${VEHICLE_CONFIG}" "${CMAKE_CURRENT_BINARY_DIR}")
  add_executable(plant_test tests/plant_test.cc)
  target_link_libraries(
    plant_test
    PRIVATE vtol_model gz-sim8::gz-sim8 tinyxml2::tinyxml2)
  add_test(
    NAME prepare_world
    COMMAND
      vtol_prepare "${VEHICLE_CONFIG}" "${CMAKE_CURRENT_SOURCE_DIR}"
      "${CMAKE_CURRENT_BINARY_DIR}/test-world")
  set_tests_properties(prepare_world PROPERTIES FIXTURES_SETUP prepared_world)
  add_test(
    NAME plant
    COMMAND plant_test "${CMAKE_CURRENT_BINARY_DIR}/test-world")
  set_tests_properties(
    plant
    PROPERTIES
      FIXTURES_REQUIRED prepared_world TIMEOUT 120 ENVIRONMENT
      "GZ_SIM_SYSTEM_PLUGIN_PATH=${CMAKE_CURRENT_BINARY_DIR};GZ_SIM_RESOURCE_PATH=${CMAKE_CURRENT_BINARY_DIR}/test-world/models:${CMAKE_CURRENT_SOURCE_DIR}/models")
endif()
