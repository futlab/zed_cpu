cmake_minimum_required(VERSION 2.8.3)
project(zed_cpu)

find_package(catkin REQUIRED
        COMPONENTS
            cmake_modules
            roscpp
            tf
            cv_bridge
            image_transport
            camera_info_manager
        )

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    )

add_definitions(${EIGEN_DEFINITIONS})

link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
    # INCLUDE_DIRS include
    LIBRARIES 
    CATKIN_DEPENDS
)

add_executable(zed_cpu
    src/zed.cpp         include/zed.h
    src/zed_ros.cpp     include/zed_ros.h
    src/zed_node.cpp
)

target_link_libraries(zed_cpu ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})

add_executable(check src/check_node.cpp src/zed.cpp)
target_link_libraries(check ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(repeater
    src/zed.cpp         include/zed.h
    src/zed_ros.cpp     include/zed_ros.h
    src/repeater_node.cpp
)
target_link_libraries(repeater ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})

# install(TARGETS 
#        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#        )

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   PATTERN ".svn" EXCLUDE
# )
