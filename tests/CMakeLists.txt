# =============================================================================
# ACS4 Unit Tests — Native x86 Build
# =============================================================================
cmake_minimum_required(VERSION 3.20)
project(acs4_tests CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ── Google Test (fetched at configure time) ───────────────────────────────
include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.14.0
    GIT_SHALLOW    TRUE
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# ── Eigen (header-only, from submodule) ───────────────────────────────────
set(EIGEN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../lib/eigen)

# ── Navigation sources (platform-independent, no ChibiOS deps) ───────────
set(NAV_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/navigation/quaternion.cpp
)

# ── Test sources ──────────────────────────────────────────────────────────
set(TEST_SOURCES
    unit/test_quaternion.cpp
)

# ── Test executable ───────────────────────────────────────────────────────
add_executable(acs4_tests ${TEST_SOURCES} ${NAV_SOURCES})

target_include_directories(acs4_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../src
    ${EIGEN_DIR}
)

target_compile_options(acs4_tests PRIVATE
    -Wall -Wextra -Wpedantic
    -O2
)

target_link_libraries(acs4_tests
    GTest::gtest_main
)

# ── CTest integration ────────────────────────────────────────────────────
enable_testing()
include(GoogleTest)
gtest_discover_tests(acs4_tests)
