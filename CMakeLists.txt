cmake_minimum_required(VERSION 3.25)
set(ProjectName TestOgre)
project(${ProjectName})

set(CMAKE_CXX_STANDARD 20)

set(OGRE_DIR h:/CodeLib/ogre-build-msvc-debug/sdk/CMake)

find_package(OGRE REQUIRED COMPONENTS Bites CONFIG)

#add_definitions(-D_MT)


message("OGRE_STATIC : ${OGRE_STATIC}")
message("OGRE_Bites_FOUND : ${OGRE_Bites_FOUND}")
message("OGRE_PLUGIN_DIR : ${OGRE_PLUGIN_DIR}")
message("OGRE_MEDIA_DIR : ${OGRE_MEDIA_DIR}")
message("OGRE_CONFIG_DIR : ${OGRE_CONFIG_DIR}")


# https://github.com/ilmola/generator
include_directories(h:/CodeLib/generator-build-msvc-debug-install/include)
include_directories(h:/CodeLib/gml/include)
link_directories(h:/CodeLib/generator-build-msvc-debug-install/lib/)


add_executable(${ProjectName}
        main.cpp
        # https://github.com/fetisov/ttf2mesh
        ttf2mesh/ttf2mesh.c
        ttf2mesh/ttf2mesh.h
        )
target_link_libraries(${ProjectName} OgreBites)

# https://github.com/ilmola/generator
target_link_libraries(${ProjectName} generator.lib)

