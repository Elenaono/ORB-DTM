cmake_minimum_required(VERSION 3.13)
project(ORB-DTM)

set( CMAKE_BUILD_TYPE "Debug" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

# 添加g2o的依赖
# 因为g2o不是常用库，要添加它的findg2o.cmake文件
# 添加cmake模块以使用g2o
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
SET( G2O_ROOT /usr/local/include/g2o )
find_package( g2o REQUIRED )
find_package( OpenCV 3.1 REQUIRED )
#find_package( CSparse REQUIRED )
#添加
include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIRS}
        ${G2O_INCLUDE_DIRS}
        ${CSPARSE_INCLUDE_DIR}
        "/usr/include/eigen3/"
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
        src/ORBextractor.cpp
        src/computeSM.cpp
        src/DTMunit.cpp)

target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        ${CSPARSE_LIBRARY}
        g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension
        )


add_executable(orbextract example/orbextract.cpp)
target_link_libraries(orbextract ${PROJECT_NAME})

add_executable(creatDelaunay example/creatDelaunay.cpp)
target_link_libraries(creatDelaunay ${PROJECT_NAME})

#add_executable(test example/test.cpp)
#target_link_libraries(test ${PROJECT_NAME})
