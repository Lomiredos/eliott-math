cmake_minimum_required(VERSION 3.20)

if(TARGET eliott-math)
    return()
endif()

project(eliott-math VERSION 1.0.0 LANGUAGES CXX)

add_library(eliott-math INTERFACE)
add_library(eliott::math ALIAS eliott-math)

target_include_directories(eliott-math INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_compile_features(eliott-math INTERFACE cxx_std_20)

option(ELIOTT_MATH_BUILD_TESTS "Build eliott-math tests" ${PROJECT_IS_TOP_LEVEL})

if(ELIOTT_MATH_BUILD_TESTS)
    find_package(doctest CONFIG QUIET)
    if(NOT doctest_FOUND)
        include(FetchContent)
        FetchContent_Declare(doctest
            GIT_REPOSITORY https://github.com/doctest/doctest.git
            GIT_TAG v2.4.12
            GIT_SHALLOW TRUE
        )
        FetchContent_MakeAvailable(doctest)
    endif()

    file(GLOB ELIOTT_MATH_TEST_SOURCES CONFIGURE_DEPENDS tests/*.cpp)
    add_executable(eliott-math-tests ${ELIOTT_MATH_TEST_SOURCES})
    target_link_libraries(eliott-math-tests PRIVATE eliott::math doctest::doctest)

    enable_testing()
    add_test(NAME eliott-math-tests COMMAND eliott-math-tests)
endif()
