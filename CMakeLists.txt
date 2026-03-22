cmake_minimum_required(VERSION 3.10)

# msvc hot reload support
if(POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
endif()

project("space_sim")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(LOCAL_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include")

# executable and source files
add_executable(space_sim 
    "space_sim.cpp"
    "space_sim.h"
    "src/graphics.cpp"
    "include/globals.h"
)

# include directories for eigen and sdl
target_include_directories(space_sim PRIVATE 
    ${LOCAL_INCLUDE_DIR}
    "${LOCAL_INCLUDE_DIR}/Eigen"
)

# link sdl3 arm64 library
set(SDL3_LIB_PATH "${LOCAL_INCLUDE_DIR}/SDL3/lib/arm64/SDL3.lib")
target_link_libraries(space_sim PRIVATE ${SDL3_LIB_PATH})

# copy sdl3.dll to build folder post-build
add_custom_command(TARGET space_sim POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "${LOCAL_INCLUDE_DIR}/SDL3/lib/arm64/SDL3.dll"
    $<TARGET_FILE_DIR:space_sim>
)