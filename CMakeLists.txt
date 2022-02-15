PROJECT(sos_slam)
CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)

set(BUILD_TYPE Release)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(SuiteParse REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system thread) 
find_package(Pangolin 0.2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
	geometry_msgs
	roscpp
	rosbag
	sensor_msgs
	cv_bridge
	image_transport
	message_filters
)

catkin_package()

add_definitions("-DENABLE_SSE")
set(CMAKE_CXX_FLAGS "${SSE_FLAGS} -O3 -g -march=native")

include_directories(
	src
	thirdparty/Sophus
	thirdparty/sse2neon
	thirdparty/g2o
	${EIGEN3_INCLUDE_DIR}
	${catkin_INCLUDE_DIRS}
	${Pangolin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	${CSPARSE_INCLUDE_DIR} 
	${CHOLMOD_INCLUDE_DIR}
) 

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(sos_slam_lib
	src/SlamNode.cpp
	src/FullSystem/FullSystem.cpp
	src/FullSystem/FullSystemOptimize.cpp
	src/FullSystem/FullSystemOptPoint.cpp
	src/FullSystem/FullSystemDebugStuff.cpp
	src/FullSystem/FullSystemMarginalize.cpp
	src/FullSystem/Residuals.cpp
	src/FullSystem/ScaleOptimizer.cpp
	src/FullSystem/CoarseInitializer.cpp
	src/FullSystem/CoarseTracker.cpp
	src/FullSystem/ImmaturePoint.cpp
	src/FullSystem/HessianBlocks.cpp
	src/FullSystem/PixelSelector2.cpp
	src/OptimizationBackend/EnergyFunctional.cpp
	src/OptimizationBackend/AccumulatedTopHessian.cpp
	src/OptimizationBackend/AccumulatedSCHessian.cpp
	src/OptimizationBackend/EnergyFunctionalStructs.cpp
	src/util/settings.cpp
	src/util/Undistort.cpp
	src/util/globalCalib.cpp
	src/IOWrapper/Pangolin/KeyFrameDisplay.cpp 
	src/IOWrapper/Pangolin/PangolinSOSVIOViewer.cpp 
	src/IOWrapper/OpenCV/ImageDisplay_OpenCV.cpp
	src/IOWrapper/OpenCV/ImageRW_OpenCV.cpp
	src/LoopClosure/LoopHandler.cpp 
	src/LoopClosure/ScanContext.cpp 
	src/LoopClosure/PoseEstimator.cpp
)

add_executable(sos_slam_node src/main.cpp)

target_link_libraries(sos_slam_node
	sos_slam_lib 
	${catkin_LIBRARIES} 
	${BOOST_THREAD_LIBRARY} 
	${Pangolin_LIBRARIES} 
	${OpenCV_LIBS} 
    ${PCL_LIBRARIES}
	${PROJECT_SOURCE_DIR}/thirdparty/g2o/lib/libg2o_core.so
	${PROJECT_SOURCE_DIR}/thirdparty/g2o/lib/libg2o_types_slam3d.so
	${PROJECT_SOURCE_DIR}/thirdparty/g2o/lib/libg2o_solver_eigen.so
	boost_system boost_thread cxsparse)

