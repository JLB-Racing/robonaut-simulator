cmake_minimum_required(VERSION 3.0.0...3.27.0)
project(robonaut-simulator VERSION 0.1.0 LANGUAGES C CXX)

include(CTest)
enable_testing()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_BUILD_TYPE Release)
add_compile_options(-Wall -Wextra -pedantic -Werror)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

#sfml
find_package(SFML 2.5 COMPONENTS graphics audio REQUIRED)

include_directories(include)

add_executable(robonaut-simulator src/main.cc)
target_link_libraries(robonaut-simulator sfml-graphics sfml-audio)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
