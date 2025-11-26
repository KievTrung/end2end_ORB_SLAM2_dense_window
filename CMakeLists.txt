cmake_minimum_required(VERSION 3.10)
project(ORB_SLAM2)


SET(CMAKE_BUILD_TYPE RelWithDebInfo)

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

if (MSVC)
    add_definitions(-DCOMPILEDWITHC11)

    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /std:c++latest")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Zm2000")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W1")

    add_compile_definitions(_ENFORCE_MATCHING_ALLOCATORS=0)

    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()


if (${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    set(WINDOWS TRUE)

    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /Zi")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /Mp")

    SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /O2")
endif()


LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 3.2 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

find_package(Eigen3 CONFIG REQUIRED)
find_package(Pangolin CONFIG REQUIRED)
find_package(G2O REQUIRED)
find_package(PCL REQUIRED )


include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS}
)

add_definitions( ${PCL_DEFINITIONS} )
link_directories( ${PCL_LIBRARY_DIRS} )

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
src/System.cc
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/FrameDrawer.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Map.cc
src/MapDrawer.cc
src/Optimizer.cc
src/PnPsolver.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Initializer.cc
src/Viewer.cc
src/OSSpecific.cc
src/pointcloudmapping.cc
)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/RelWithDebInfo/DBoW2.lib
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/bin/RelWithDebInfo/g2o.lib
${PCL_LIBRARIES}
)


if (MSVC)
    target_compile_definitions(${PROJECT_NAME} PRIVATE WIN_EXPORT)
endif()

# Build examples

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/RGBD)

add_executable(rgbd_tum RGBD/rgbd_tum.cc)
target_link_libraries(rgbd_tum ${PROJECT_NAME})

add_executable(rgb_tumd RGBD/rgb_tumd.cc)
target_link_libraries(rgb_tumd ${PROJECT_NAME})

add_executable(rgbd_kitti RGBD/rgbd_kitti.cc)
target_link_libraries(rgbd_kitti ${PROJECT_NAME})

#Build tools
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/tools)
add_executable(bin_vocabulary tools/bin_vocabulary.cc)
target_link_libraries(bin_vocabulary ${PROJECT_NAME})

