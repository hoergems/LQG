cmake_minimum_required (VERSION 2.6)

project (lqg)
set(CMAKE_BUILD_TYPE DEBUG)
#set(CMAKE_CXX_FLAGS "-g -std=c++11 -pg")
set(CMAKE_CXX_FLAGS "-g -std=c++11")
#SET(CMAKE_EXE_LINKER_FLAGS "-pg")
#SET(CMAKE_SHARED_LINKER_FLAGS "-pg")
set(CMAKE_CXX_FLAGS_DEBUG "-g -std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-g -std=c++11")

SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${lqg_SOURCE_DIR})
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${lqg_SOURCE_DIR}/..)

include_directories(/usr/include/eigen3 
                    /usr/inclue/boost 
                    /usr/include/python2.7
                    /usr/include/jsoncpp
                    /usr/local/include/openrave-0.9
                    /usr/local/include/ompl
                    /usr/local/include/fcl/)

# set the path to the library folder
link_directories(/usr/lib /usr/local/lib /opt/ros/indigo/lib)

# link the libraries to the executable
add_library(util SHARED 
            utils.cpp
            Terrain.cpp
            Obstacle.cpp
            BoxObstacle.cpp)

add_library(area SHARED
            Area.cpp)

#add_library(kinematics SHARED
#            Kinematics.cpp
#            robot.cpp)

add_library(robot SHARED 
            robot.cpp
            propagator.cpp
            viewer_interface.cpp
            urdf_loader.cpp
            viewer.cpp
            integrate.cpp
            Kinematics.cpp)

add_library(integrate SHARED integrate.cpp)

add_library(propagator SHARED 
            propagator.cpp
            torque_damper.cpp            
            integrate.cpp)

add_library(obstacle SHARED
            Obstacle.cpp
            Terrain.cpp)

add_library(path_planner SHARED
            path_planner.cpp
            MotionValidator.cpp
            ManipulatorGoalRegion.cpp
            robot.cpp
            viewer_interface.cpp
            urdf_loader.cpp
            viewer.cpp
            Kinematics.cpp
            propagator.cpp
            integrate.cpp
            utils.cpp
            Terrain.cpp
            Obstacle.cpp
            BoxObstacle.cpp)

add_library(dynamic_path_planner SHARED 
            dynamic_path_planner.cpp
            ManipulatorSpaceInformation.cpp
            rrt_control.cpp 
            MotionValidator.cpp
            state_propagator.cpp
            propagator.cpp
            integrate.cpp            
            ManipulatorGoalRegion.cpp            
            Kinematics.cpp
            utils.cpp
            robot.cpp
            viewer_interface.cpp
            urdf_loader.cpp
            viewer.cpp
            Obstacle.cpp
            BoxObstacle.cpp
            Terrain.cpp
            torque_damper.cpp
            control_space.cpp
            control_sampler.cpp)

target_link_libraries (util
                       boost_system
                       boost_python
                       python2.7
                       tinyxml
                       fcl)

target_link_libraries(area
                      boost_system
                      boost_python
                      python2.7)

#target_link_libraries(kinematics
#                      boost_system
#                      boost_python
#                      python2.7
#                      tinyxml)

target_link_libraries(path_planner
                      boost_system
                      boost_python
                      python2.7 
                      openrave0.9 
                      openrave0.9-core                     
                      ompl
                      fcl
                      tinyxml
                      urdf)

target_link_libraries(obstacle
                      openrave0.9 
                      openrave0.9-core
                      boost_system
                      boost_python
                      python2.7
                      fcl
                      tinyxml)

target_link_libraries (robot
                       urdf
                       boost_system
                       boost_python
                       python2.7
                       openrave0.9 
                       openrave0.9-core
                       fcl
                       tinyxml
                       urdf)

target_link_libraries (integrate
                       boost_system
                       boost_python
                       python2.7)
                       
target_link_libraries (propagator
                       boost_system
                       boost_python
                       python2.7
                       openrave0.9
                       openrave0.9-core)

target_link_libraries (dynamic_path_planner                        
                       openrave0.9 
                       openrave0.9-core 
                       ompl
                       boost_system
                       boost_python
                       python2.7 
                       boost_thread
                       fcl
                       tinyxml
                       urdf)

#set_property(TARGET robot PROPERTY CXX_STANDARD 11)
                       
#file(COPY "libintegrate.so" DESTINATION "..")
