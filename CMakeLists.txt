cmake_minimum_required(VERSION 3.20)
project(compass_stackchan LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# lib/ 配下は ESP32 非依存の純 C++17 として書き、ホストでテストする。
# ファームウェア側は platformio.ini の lib_extra_dirs で同じソースを拾う。
set(COMPASS_WARNINGS -Wall -Wextra -Wpedantic -Werror)

add_library(
  astro_core
  lib/astro_core/src/angles.cpp
  lib/astro_core/src/time.cpp
  lib/astro_core/src/coords.cpp
  lib/astro_core/src/sun.cpp
  lib/astro_core/src/moon.cpp
  lib/astro_core/src/planets.cpp
  lib/astro_core/src/ephemeris.cpp)
target_include_directories(astro_core PUBLIC lib/astro_core/include)
target_compile_options(astro_core PRIVATE ${COMPASS_WARNINGS})

add_library(
  compass_core
  lib/compass_core/src/calibration.cpp
  lib/compass_core/src/heading.cpp
  lib/compass_core/src/declination.cpp
  lib/compass_core/src/sphere_fit.cpp
  lib/compass_core/src/stability.cpp)
target_include_directories(compass_core PUBLIC lib/compass_core/include)
target_compile_options(compass_core PRIVATE ${COMPASS_WARNINGS})

add_library(
  pointing_core
  lib/pointing_core/src/servo_map.cpp
  lib/pointing_core/src/solver.cpp)
target_include_directories(pointing_core PUBLIC lib/pointing_core/include)
target_compile_options(pointing_core PRIVATE ${COMPASS_WARNINGS})

# 状態機械。実機とシミュレータで同じものを動かす。
add_library(app_core lib/app_core/src/state.cpp)
target_include_directories(app_core PUBLIC lib/app_core/include)
target_link_libraries(app_core PUBLIC astro_core compass_core pointing_core)
target_compile_options(app_core PRIVATE ${COMPASS_WARNINGS})

# 実機なしでロジックを目視確認するための CLI
add_executable(compass-simulator host/simulator.cpp)
target_link_libraries(compass-simulator PRIVATE app_core)
target_compile_options(compass-simulator PRIVATE ${COMPASS_WARNINGS})

include(CTest)
if(BUILD_TESTING)
  foreach(suite astro compass pointing)
    add_executable(${suite}-test test/${suite}_test.cpp)
    target_link_libraries(${suite}-test PRIVATE ${suite}_core)
    target_include_directories(${suite}-test PRIVATE test)
    target_compile_options(${suite}-test PRIVATE ${COMPASS_WARNINGS})
    add_test(NAME ${suite}-test COMMAND ${suite}-test)
  endforeach()

  # ファームの loop() が方位を確定させる経路をホストで再現する
  add_executable(heading-pipeline-test test/heading_pipeline_test.cpp)
  target_link_libraries(heading-pipeline-test PRIVATE app_core)
  target_include_directories(heading-pipeline-test PRIVATE test)
  target_compile_options(heading-pipeline-test PRIVATE ${COMPASS_WARNINGS})
  add_test(NAME heading-pipeline-test COMMAND heading-pipeline-test)

  add_executable(sphere-fit-test test/sphere_fit_test.cpp)
  target_link_libraries(sphere-fit-test PRIVATE compass_core)
  target_include_directories(sphere-fit-test PRIVATE test)
  target_compile_options(sphere-fit-test PRIVATE ${COMPASS_WARNINGS})
  add_test(NAME sphere-fit-test COMMAND sphere-fit-test)

  add_executable(app-test test/app_test.cpp)
  target_link_libraries(app-test PRIVATE app_core)
  target_include_directories(app-test PRIVATE test)
  target_compile_options(app-test PRIVATE ${COMPASS_WARNINGS})
  add_test(NAME app-test COMMAND app-test)

  # 天体計算からサーボ指令までを通しで固定するゴールデンテスト
  add_executable(integration-test test/integration_test.cpp)
  target_link_libraries(integration-test PRIVATE astro_core compass_core pointing_core)
  target_include_directories(integration-test PRIVATE test)
  target_compile_options(integration-test PRIVATE ${COMPASS_WARNINGS})
  add_test(NAME integration-test COMMAND integration-test)
endif()
