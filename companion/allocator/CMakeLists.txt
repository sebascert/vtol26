cmake_minimum_required(VERSION 3.14...3.24)
project(vtol_allocator LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONC REQUIRED IMPORTED_TARGET json-c)

include(FetchContent)
fetchcontent_declare(
  mavlink
  URL
    https://codeload.github.com/mavlink/c_library_v2/tar.gz/e5697c4deaf09068560cb33da2a0283ad6229b3e
  URL_HASH
    SHA256=6b699bd61331d4232931c8e20fcc5b8fdccefabe0fc43b194dfe0598cfd3ba1d)
fetchcontent_makeavailable(mavlink)

set(PSO_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/pso)
add_library(pso_runtime ${PSO_DIR}/pso_environment.cpp
            ${PSO_DIR}/config_loader.cpp ${PSO_DIR}/vehicle_model.cpp)
target_compile_definitions(pso_runtime PRIVATE PSO_RUNTIME_ONLY)
target_include_directories(pso_runtime PUBLIC ${PSO_DIR})
target_link_libraries(pso_runtime PUBLIC PkgConfig::JSONC)

add_library(companion_core src/application_config.cpp src/pso_interface.cpp
            src/mavlink_receiver.cpp src/mavlink_sender.cpp)
target_include_directories(companion_core PUBLIC src)
target_include_directories(companion_core SYSTEM PUBLIC ${mavlink_SOURCE_DIR})
target_link_libraries(companion_core PUBLIC pso_runtime)
add_executable(vtol_allocator src/main.cpp)
target_link_libraries(vtol_allocator PRIVATE companion_core)

include(CTest)
if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(
    NAME vehicle_config
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/vehicle_config_test.py
      $<TARGET_FILE:vtol_allocator>)
  add_executable(frames_test tests/frames_test.cpp)
  target_link_libraries(frames_test PRIVATE companion_core)
  add_test(
    NAME frames
    COMMAND frames_test ${CMAKE_CURRENT_BINARY_DIR}/frames_config.json)
  add_executable(allocation_test tests/allocation_test.cpp)
  target_link_libraries(allocation_test PRIVATE companion_core util)
  add_test(
    NAME installer
    COMMAND
      bash ${CMAKE_CURRENT_SOURCE_DIR}/tests/install_test.sh
      ${CMAKE_CURRENT_SOURCE_DIR}/deploy/install.sh ${CMAKE_CURRENT_BINARY_DIR})
  add_test(
    NAME launcher
    COMMAND
      bash ${CMAKE_CURRENT_SOURCE_DIR}/tests/run_companion_test.sh
      ${CMAKE_CURRENT_SOURCE_DIR}/deploy/run-companion.sh
      ${CMAKE_CURRENT_BINARY_DIR})
  add_test(
    NAME allocation
    COMMAND
      allocation_test ${CMAKE_CURRENT_SOURCE_DIR}/src/pso/config.json
      $<TARGET_FILE:vtol_allocator>)
endif()

install(TARGETS vtol_allocator RUNTIME DESTINATION bin)
install(
  FILES src/pso/config.json
  DESTINATION share/vtol26/companion/mavlink_uart/src/pso)
install(FILES deploy/vtol_allocator@.service DESTINATION lib/systemd/system)
