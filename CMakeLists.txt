cmake_minimum_required(VERSION 3.20)
project(eliott-math VERSION 1.0.0 LANGUAGES CXX)
set(CMAKE_TOOLCHAIN_FILE "C:/Dev/vcpkg/scripts/buildsystems/vcpkg.cmake")
set(CMAKE_PREFIX_PATH "C:/Dev/vcpkg/installed/x64-windows")

# Standard C++20 obligatoire
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# --- Dépendances ---
find_package(doctest CONFIG REQUIRED)

# --- Lib statique ---
add_library(eliott-math INTERFACE
)

target_include_directories(eliott-math INTERFACE

    include/
)

# --- Tests ---
add_executable(tests
    tests/test_math.cpp
)

target_link_libraries(tests PRIVATE
    eliott-math
    doctest::doctest
)