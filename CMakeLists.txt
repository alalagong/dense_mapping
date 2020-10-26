cmake_minimum_required(VERSION 2.8.3)

project(lt_slam)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --std=c++11 Og -fopenmp -march=native -DEIGEN_DONT_PARALLELIZE")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Og -fopenmp -march=native -DEIGEN_DONT_PARALLELIZE")

find_package(catkin REQUIRED COMPONENTS
    pcl_conversions
    pcl_ros
    roscpp
    sensor_msgs
    dynamic_reconfigure
    geometry_msgs
    rosbag
)
find_package(Boost COMPONENTS thread serialization REQUIRED)
find_package(OpenCV 3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Ceres REQUIRED)

if(CUDA_VERSION VERSION_LESS "9.0")
	set(NPP_LIBRARIES "${CUDA_nppi_LIBRARY}")
else()
	set(NPP_LIBRARIES "${CUDA_nppif_LIBRARY}")
endif()

#CUDA
find_package(CUDA 10 REQUIRED)
include_directories(${CUDA_INCLUDE_DIRS})
set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} -lineinfo -gencode=arch=compute_75,code=sm_75)

#SET(STEREOSRCS
#        src/sgm/stereo_sgm.cpp
#        src/sgm/census.cu
#        src/sgm/matching_cost.cu
#        src/sgm/scan_scost.cu
#        src/sgm/winner_takes_all.cu
#        src/sgm/median_filter.cu
#        src/sgm/check_consistency.cu
#        src/sgm/cuda_utils.cu
#)
#CUDA_ADD_LIBRARY(sgm ${STEREOSRCS})
#target_link_libraries(sgm ${CUDA_LIBRARIES} ${NPP_LIBRARIES} ${OpenCV_LIBS})

FILE(GLOB SGM2_SRC src/sgm2/*.*)
CUDA_ADD_LIBRARY(sgm2 ${SGM2_SRC})
target_link_libraries(sgm2 ${CUDA_LIBRARIES} ${OpenCV_LIBS})

catkin_package(
    DEPENDS
    CATKIN_DEPENDS
)

include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${catkin_INCLUDE_DIRS}
    )

set(LIBS
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES}
    ${Pangolin_LIBRARIES}
    dw
    ${CUDA_LIBRARIES}
    ${NPP_LIBRARIES}
    ${CERES_LIBRARIES}
)

FILE(GLOB HEADER_FILES src/*.h*)
FILE(GLOB SOURCE_FILES src/*.c*)
FILE(GLOB_RECURSE CHISEL_FILES src/open_chisel/*.h* src/open_chisel/*.c*)
FILE(GLOB_RECURSE CERES_FILES src/ceres_factor/*.h* src/ceres_factor/*.c*)
FILE(GLOB_RECURSE ELAS_FILES src/libelas_omp/*.h* src/libelas_omp/*.c*)
FILE(GLOB_RECURSE VOC_FILES Vocabulary/DBoW2/*.h* Vocabulary/DBoW2/*.c* Vocabulary/DUtils/*.h* Vocabulary/DUtils/*.c*)
add_library(lt_slam SHARED ${SOURCE_FILES} ${HEADER_FILES} ${CERES_FILES} ${CHISEL_FILES} ${ELAS_FILES} ${VOC_FILES})
target_link_libraries(lt_slam ${LIBS} sgm2)

add_executable(lt_stereo_euroc ros_stereo_euroc.cc )
target_link_libraries(lt_stereo_euroc lt_slam ${LIBS})

add_executable(lt_kitti ros_stereo_kitti.cc)
target_link_libraries(lt_kitti lt_slam ${LIBS})

#add_executable(lt_compare compare.cc)
#target_link_libraries(lt_compare lt_slam ${LIBS})

add_executable(lt_new_college ros_stereo_newCollege.cc)
target_link_libraries(lt_new_college lt_slam ${LIBS})

add_executable(lt_hkust ros_stereo_hkust.cc)
target_link_libraries(lt_hkust lt_slam ${LIBS})

#add_executable(lt_stereo_live ros_stereo_live.cc)
#target_link_libraries(lt_stereo_live lt_slam)

#add_executable(record_data record_data.cc)
#target_link_libraries(record_data lt_slam)
