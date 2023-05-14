cmake_minimum_required(VERSION 3.8)
project(FireflyLED)

find_package(gz-cmake3 REQUIRED)

find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

find_package(gz-sim7 REQUIRED)
set(GZ_SIM_VER ${gz-sim7_VERSION_MAJOR})

add_library(FireflyLED SHARED FireflyLED.cc)
set_property(TARGET FireflyLED PROPERTY CXX_STANDARD 17)
target_link_libraries(FireflyLED
  PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  PRIVATE gz-sim7::gz-sim7)