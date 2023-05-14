cmake_minimum_required(VERSION 3.8)
project(FireflyLED)

find_package(gz-cmake3 REQUIRED)

find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-sim7 REQUIRED)
find_package(gz-transport12 REQUIRED)
find_package(gz-msgs9 REQUIRED)

add_library(FireflyLED SHARED FireflyLED.cc)
set_property(TARGET FireflyLED PROPERTY CXX_STANDARD 17)
target_link_libraries(FireflyLED
  PRIVATE gz-plugin2:gz-plugin2
  PRIVATE gz-sim7::gz-sim7
  PRIVATE gz-transport12::gz-transport12
  PRIVATE gz-msgs9::gz-msgs9
  )