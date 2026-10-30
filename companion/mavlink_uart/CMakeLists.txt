cmake_minimum_required(VERSION 3.14...3.24)
project(mavlink_uart LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(mavlink
    URL https://codeload.github.com/mavlink/c_library_v2/tar.gz/e5697c4deaf09068560cb33da2a0283ad6229b3e
    URL_HASH SHA256=6b699bd61331d4232931c8e20fcc5b8fdccefabe0fc43b194dfe0598cfd3ba1d)
FetchContent_MakeAvailable(mavlink)

set(PSO_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../simulation/pso)
add_library(pso_runtime ${PSO_DIR}/pso_environment.cpp)
target_compile_definitions(pso_runtime PRIVATE PSO_RUNTIME_ONLY)
target_include_directories(pso_runtime PUBLIC ${PSO_DIR})

add_library(companion_core src/application_config.cpp src/pso_interface.cpp
    src/mavlink_receiver.cpp src/mavlink_sender.cpp)
target_include_directories(companion_core PUBLIC src)
target_include_directories(companion_core SYSTEM PUBLIC ${mavlink_SOURCE_DIR})
target_link_libraries(companion_core PUBLIC pso_runtime)
add_executable(mavlink_uart src/main.cpp)
target_link_libraries(mavlink_uart PRIVATE companion_core)

include(CTest)
if(BUILD_TESTING)
    add_executable(allocation_test tests/allocation_test.cpp)
    target_link_libraries(allocation_test PRIVATE companion_core util)
    add_test(NAME installer COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/tests/install_test.sh
        ${CMAKE_CURRENT_SOURCE_DIR}/deploy/install.sh ${CMAKE_CURRENT_BINARY_DIR})
    add_test(NAME allocation COMMAND allocation_test ${CMAKE_CURRENT_SOURCE_DIR}/src/pso/config.json $<TARGET_FILE:mavlink_uart>)
endif()

install(TARGETS mavlink_uart RUNTIME DESTINATION bin)
install(FILES src/pso/config.json DESTINATION share/vtol26/companion/mavlink_uart/src/pso)
install(FILES deploy/mavlink-uart@.service DESTINATION lib/systemd/system)
