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

message("OGRE_STATIC : ${OGRE_STATIC}")
message("OGRE_Bites_FOUND : ${OGRE_Bites_FOUND}")
message("OGRE_PLUGIN_DIR : ${OGRE_PLUGIN_DIR}")
message("OGRE_MEDIA_DIR : ${OGRE_MEDIA_DIR}")
message("OGRE_CONFIG_DIR : ${OGRE_CONFIG_DIR}")
message("OGRE_INCLUDE_DIRS : ${OGRE_INCLUDE_DIRS}")
message("OGRE_LIBRARIES : ${OGRE_LIBRARIES}")
message("OGRE_LIBRARY_DIRS : ${OGRE_LIBRARY_DIRS}")

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

set(OgreBullet
        ./Bullet/include/OgreBullet.h
        ./Bullet/src/OgreBullet.cpp
        )
include_directories(./Bullet/include)

include_directories(${BULLET_INCLUDE_DIRS})
link_directories(${BULLET_LIBRARY_DIRS})

# =============================== find Bullet end ====================================


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

if (CMAKE_THREAD_LIBS_INIT)
    target_link_libraries(${ProjectMainName} ${CMAKE_THREAD_LIBS_INIT})
endif ()

include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})
target_link_libraries(${ProjectMainName} ${Boost_LIBRARIES})

if (WIN32)
    target_link_libraries(${ProjectMainName} wsock32 ws2_32 crypt32)
else (WIN32)
endif (WIN32)


