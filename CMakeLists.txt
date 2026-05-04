cmake_minimum_required(VERSION 3.20)
project(eliott-math VERSION 1.0.0 LANGUAGES CXX)

if(TARGET eliott-math)
    return()
endif()
if (PROJECT_IS_TOP_LEVEL)
set(CMAKE_TOOLCHAIN_FILE "C:/Dev/vcpkg/scripts/buildsystems/vcpkg.cmake")
set(CMAKE_PREFIX_PATH "C:/Dev/vcpkg/installed/x64-windows")
endif()

# Standard C++20 obligatoire
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)



# --- Lib statique ---
add_library(eliott-math INTERFACE
)

target_include_directories(eliott-math INTERFACE

    include/
)
if(PROJECT_IS_TOP_LEVEL)

# --- Dépendances ---
find_package(doctest CONFIG REQUIRED)

# --- Tests ---
add_executable(tests
    tests/test_math.cpp
)

target_link_libraries(tests PRIVATE
    eliott-math
    doctest::doctest
)

endif()