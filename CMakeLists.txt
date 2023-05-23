cmake_minimum_required(VERSION 3.25)
set(ProjectMainName TestOgre)
project(${ProjectMainName})

set(CMAKE_CXX_STANDARD 20)


# =============================== find Boost start ====================================

set(BOOST_ROOT h:/CodeLib/boost_1_81_0_msvc)


# https://www.boost.org/doc/libs/1_81_0/libs/assert/doc/html/assert.html
add_definitions(-DBOOST_ENABLE_ASSERT_HANDLER)  #

option(USE_DYNAMIC_BOOST "Config USE_DYNAMIC_BOOST")


set(CMAKE_THREAD_PREFER_PTHREAD ON)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

#if (NOT_USE_BOOST_THEAD)
#    set(BOOST_THEAD_MODULE "")
#    message(STATUS "NOT_USE_BOOST_THEAD !!!")
#else ()
#    set(BOOST_THEAD_MODULE "thread")
#    add_definitions(-DUSE_BOOST_THEAD)
#endif ()
set(BOOST_THEAD_MODULE "thread")
add_definitions(-DUSE_BOOST_THEAD)


if (USE_DYNAMIC_BOOST)
    message(STATUS "USE DYNAMIC BOOST !!!")
    add_definitions(-DBOOST_ALL_DYN_LINK=1)
else (USE_DYNAMIC_BOOST)
    message(STATUS "USE STATIC BOOST !!!")
    set(Boost_USE_STATIC_LIBS ON)
    set(Boost_USE_STATIC_RUNTIME OFF)
endif (USE_DYNAMIC_BOOST)

set(Boost_USE_MULTITHREADED ON)
#find_package(Boost REQUIRED COMPONENTS  REQUIRED)
find_package(Boost 1.81.0 REQUIRED COMPONENTS system filesystem program_options json log log_setup url ${BOOST_THEAD_MODULE} REQUIRED)


message(STATUS "Boost_INCLUDE_DIRS: ${Boost_INCLUDE_DIRS}")
message(STATUS "Boost_LIBRARY_DIRS: ${Boost_LIBRARY_DIRS}")
message(STATUS "Boost_LIBRARIES: ${Boost_LIBRARIES}")
message(STATUS "Boost_VERSION: ${Boost_VERSION}")

message(STATUS "CMAKE_THREAD_LIBS_INIT: ${CMAKE_THREAD_LIBS_INIT}")


# =============================== find Boost end ====================================


# =============================== find OGRE start ====================================
set(OGRE_DIR h:/CodeLib/ogre-build-msvc-debug/sdk/CMake)

find_package(OGRE REQUIRED COMPONENTS Bites Overlay CONFIG)

message(STATUS "OGRE_STATIC : ${OGRE_STATIC}")
message(STATUS "OGRE_Bites_FOUND : ${OGRE_Bites_FOUND}")
message(STATUS "OGRE_PLUGIN_DIR : ${OGRE_PLUGIN_DIR}")
message(STATUS "OGRE_MEDIA_DIR : ${OGRE_MEDIA_DIR}")
message(STATUS "OGRE_CONFIG_DIR : ${OGRE_CONFIG_DIR}")
message(STATUS "OGRE_INCLUDE_DIRS : ${OGRE_INCLUDE_DIRS}")
message(STATUS "OGRE_LIBRARIES : ${OGRE_LIBRARIES}")
message(STATUS "OGRE_LIBRARY_DIRS : ${OGRE_LIBRARY_DIRS}")

#add_definitions(-D_MT)


# =============================== find OGRE end ====================================

# =============================== find Bullet start ====================================


set(BULLET_BulletConfig h:/Code/bullet3-build_MSVC-install/lib/cmake/bullet)

find_package(Bullet PATHS ${BULLET_BulletConfig})

list(TRANSFORM BULLET_INCLUDE_DIRS PREPEND "${BULLET_ROOT_DIR}/")
list(TRANSFORM BULLET_LIBRARY_DIRS PREPEND "${BULLET_ROOT_DIR}/")

message(STATUS "BULLET_FOUND ${BULLET_FOUND}")
message(STATUS "BULLET_USE_FILE ${BULLET_USE_FILE}")
message(STATUS "BULLET_INCLUDE_DIR ${BULLET_INCLUDE_DIR}")
message(STATUS "BULLET_INCLUDE_DIRS ${BULLET_INCLUDE_DIRS}")
message(STATUS "BULLET_LIBRARIES ${BULLET_LIBRARIES}")
message(STATUS "BULLET_LIBRARY_DIRS ${BULLET_LIBRARY_DIRS}")
message(STATUS "BULLET_ROOT_DIR ${BULLET_ROOT_DIR}")
message(STATUS "BULLET_VERSION_STRING ${BULLET_VERSION_STRING}")

# TODO
list(TRANSFORM BULLET_LIBRARIES APPEND "_Debug")
list(TRANSFORM BULLET_LIBRARIES APPEND ".lib")
#message("BULLET_LIBRARIES ${BULLET_LIBRARIES}")

list(TRANSFORM BULLET_LIBRARIES PREPEND "${BULLET_LIBRARY_DIRS}/")

# =============================== add Custom Bullet Component ====================================

set(OgreBullet
        ./Bullet/OgreBullet.h
        ./Bullet/OgreBullet.cpp
        ./Bullet/BulletMemoryContainer.h
        ./Bullet/BulletMemoryContainer.cpp
        ./Bullet/BulletMemoryPool.cpp
        ./Bullet/BulletMemoryPool.h
        ./Bullet/VertexIndexToShape.cpp
        ./Bullet/VertexIndexToShape.h
        ./Bullet/BodyHelper.cpp
        ./Bullet/BodyHelper.h)
include_directories(./Bullet)

include_directories(${BULLET_INCLUDE_DIRS})
link_directories(${BULLET_LIBRARY_DIRS})

# =============================== find Bullet end ====================================


## =============================== find PhysX start ====================================
#
#
## https://github.com/NVIDIA-Omniverse/PhysX
#set(PHYSX_PATH "h:/CodeLib/PhysX5/physx")
## https://github.com/NVIDIA-Omniverse/PhysX/issues/127
#set(CMAKEMODULES_VERSION "1.28")
#set(CMAKEMODULES_PATH "h:/packman-repo/chk/CMakeModules/1.28.trunk.32494385/")
#
#set(PHYSX_SLN_PHYSXDEVICE_PATH "H:/packman-repo/chk/PhysXDevice/18.12.7.4/bin/x86/")
#set(PHYSX_SLN_FREEGLUT_PATH "H:/packman-repo/chk/freeglut-windows/3.4_1.1/bin/")
#
#
##### Build PhysX library ####
## https://github.com/NVIDIAGameWorks/PhysX/issues/249
## PHYSX_PATH - path to the `{cloned repository}/physx` repo directory git://github.com/NVIDIAGameWorks/PhysX.git
#set( PHYSX_ROOT_DIR ${PHYSX_PATH} ) #This is needed for $ENV{PHYSX_PATH}/compiler/public/CMakeLists.txt
#set( PHYSX_INCLUDE_DIRS ${PHYSX_PATH}/include/ ${PHYSX_PATH}/../pxshared/include/ )
#set( PHYSX_LIBRARIES
#        PhysXExtensions
#        PhysX
#        PhysXPvdSDK
#        PhysXVehicle
#        PhysXCharacterKinematic
#        PhysXCooking
#        PhysXCommon
#        PhysXFoundation
#        # SnippetUtils
#        )
#
#set(TARGET_BUILD_PLATFORM "windows") # has to match the TARGET_BUILD_PLATFORM in $ENV{PHYSX_PATH}/physix/buildtools/cmake_generate_projects.py
#set(PX_BUILDSNIPPETS OFF CACHE BOOL "Generate the snippets")
#set(PX_BUILDPUBLICSAMPLES OFF CACHE BOOL "Generate the samples projects")
#set(PX_GENERATE_STATIC_LIBRARIES ON CACHE BOOL "Generate static libraries")
#set(PX_FLOAT_POINT_PRECISE_MATH OFF CACHE BOOL "Float point precise math")
#set(NV_USE_STATIC_WINCRT ON CACHE BOOL "Use the statically linked windows CRT")
#set(NV_USE_DEBUG_WINCRT ON CACHE BOOL "Use the debug version of the CRT")
#set(PXSHARED_PATH ${PHYSX_PATH}/../pxshared)
#set(PXSHARED_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX})
#set(PX_OUTPUT_LIB_DIR ${CMAKE_CURRENT_BINARY_DIR}/externals/physx)
#set(PX_OUTPUT_BIN_DIR ${CMAKE_CURRENT_BINARY_DIR}/externals/physx)
#
#message(STATUS "PX_OUTPUT_LIB_DIR ${PX_OUTPUT_LIB_DIR}")
#message(STATUS "PX_OUTPUT_BIN_DIR ${PX_OUTPUT_BIN_DIR}")
#
## Call into PhysX's CMake scripts
#add_subdirectory(${PHYSX_PATH}/compiler/public externals/physx)
#
#
## =============================== find PhysX end ====================================




# =============================== find PhysX start ====================================

# HOW TO :
# 1. call `physx/generate_projects.sh` to generate .sln build file
# 2. open `physx/compiler/vc17win64/PhysXSDK.sln`
# 3. change all sub-target type to `/MD` `/MDd` in right tab project list , rebuild all file

set(PhysX_INCLUDE_DIR "h:/CodeLib/PhysX/physx/install/vc17win64/PhysX/include/")
#set(PhysX_LIBRARY_DIR "h:/CodeLib/PhysX/physx/bin/win.x86_64.vc143.mt/checked/")
set(PhysX_LIBRARY_DIR "h:/CodeLib/PhysX/physx/bin/win.x86_64.vc143.mt/debug/")

set(PhysX_LIBRARYS
        # copy dll to exe dir
        PhysXCommon_64
        PhysX_64
        PhysXFoundation_64
        PhysXCooking_64
        PVDRuntime_64
        # the PhysXGPU_64 is only a dll, copy it to exe dir
#        PhysXGPU_64
        # follow -- need rebuild with /MD /MDd
        PhysXPvdSDK_static_64
        PhysXExtensions_static_64
        )
list(TRANSFORM PhysX_LIBRARYS PREPEND ${PhysX_LIBRARY_DIR})
list(TRANSFORM PhysX_LIBRARYS APPEND ".lib")

message(STATUS "PhysX_LIBRARYS ${PhysX_LIBRARYS}")

# =============================== find PhysX end ====================================




# =============================== find generator start ====================================

# https://github.com/ilmola/generator
include_directories(h:/CodeLib/generator-build-msvc-debug-install/include)
include_directories(h:/CodeLib/gml/include)
link_directories(h:/CodeLib/generator-build-msvc-debug-install/lib/)

# =============================== find generator end ====================================


add_executable(${ProjectMainName}
        main.cpp
        # https://github.com/fetisov/ttf2mesh
        ttf2mesh/ttf2mesh.c
        ttf2mesh/ttf2mesh.h
        ${OgreBullet}
        )
target_link_libraries(${ProjectMainName} OgreBites)

# https://github.com/ilmola/generator
target_link_libraries(${ProjectMainName} generator.lib)

target_link_libraries(${ProjectMainName} ${BULLET_LIBRARIES})


include_directories(${PhysX_INCLUDE_DIR})
#link_directories(${PhysX_LIBRARY_DIR})
target_link_libraries(${ProjectMainName} ${PhysX_LIBRARYS})


if (CMAKE_THREAD_LIBS_INIT)
    target_link_libraries(${ProjectMainName} ${CMAKE_THREAD_LIBS_INIT})
endif ()

include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})
target_link_libraries(${ProjectMainName} ${Boost_LIBRARIES})

if (WIN32)
    # https://github.com/boostorg/log/issues/28
    #    add_compile_definitions(BOOST_USE_WINAPI_VERSION=0x0501)
    add_compile_definitions(BOOST_USE_WINAPI_VERSION=0x0600)
    target_link_libraries(${ProjectMainName} wsock32 ws2_32 crypt32)
else (WIN32)
endif (WIN32)

## =============================== post PhysX start ====================================
#
## Add physx libraries to target
#target_link_libraries(${ProjectMainName} ${PHYSX_LIBRARIES})
##### Windows only: Copy the Physx dll files to the simulation executable####
#
#if (TARGET_BUILD_PLATFORM STREQUAL "windows")
#    # References NvidiaBuildOptions.cmake to figure out if system is 32/64 bit
#    IF(CMAKE_SIZEOF_VOID_P EQUAL 8)
#        SET(LIBPATH_SUFFIX "64")
#    ELSE()
#        SET(LIBPATH_SUFFIX "32")
#    ENDIF()
#    GetPlatformBinName(PLATFORM_BIN_NAME ${LIBPATH_SUFFIX})
#    set(PhysxOutputPath ${PX_OUTPUT_LIB_DIR}/bin/${PLATFORM_BIN_NAME}/)
#    message(STATUS "PhysxOutputPath ${PhysxOutputPath}")
#    message("Physx Output Path: " ${PhysxOutputPath})
#
#    # copy PhysX dll's to build dir. Happens on every build.
#    add_custom_command(TARGET ${ProjectMainName} POST_BUILD
#            COMMAND ${CMAKE_COMMAND} -E copy_directory "${PhysxOutputPath}" "$<TARGET_FILE_DIR:${ProjectMainName}>/..")
#endif()
## =============================== post PhysX end ====================================

