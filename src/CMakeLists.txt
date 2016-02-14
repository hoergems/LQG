cmake_minimum_required (VERSION 2.6)

project (lqg)
set(CMAKE_BUILD_TYPE DEBUG)
set(CMAKE_CXX_FLAGS_DEBUG "-g -std=c++11 -O3 -w -fopenmp -mavx -mfma -DNDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "-g -std=c++11 -O3 -w -fopenmp -mavx -mfma -DNDEBUG")

SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${lqg_SOURCE_DIR})
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${lqg_SOURCE_DIR}/../)
SET(BOOST_MIN_VERSION "1.54.0")
SET(PYTHON_MIN_VERSION "2.7.0")
include(FindPkgConfig)

####### USER SPECIFIED VARIABLES #########
set(USE_URDF True)


set(CMAKE_PREFIX_PATH "/home/uqmhoerg/home/opt/include")
message ("CMAKE PREFIXPATH ${CMAKE_PREFIX_PATH}" )

##############################
#Try to load URDF
##############################
if (USE_URDF)       
    if(PKG_CONFIG_FOUND)
        pkg_check_modules(URDF urdf)
        if(NOT URDF_FOUND)
           message(FATAL_ERROR "URDF package could not be found")
        endif()
        include_directories(${URDF_INCLUDE_DIRS})
        link_directories(${URDF_LIBRARY_DIRS})    
    endif()
        
############################# LOAD OPENRAVE #############################
    if(PKG_CONFIG_FOUND)
        pkg_check_modules(OPENRAVE openrave0.9-core)
        if(NOT OPENRAVE_FOUND)
           message(FATAL_ERROR "OpenRAVE 0.9 could not be found")           
        endif()
        include_directories(${OPENRAVE_INCLUDE_DIRS})
        link_directories(${OPENRAVE_LIBRARY_DIRS})           
    endif() 
    
    if(PKG_CONFIG_FOUND)    
	    pkg_check_modules(VIEWER viewer_interface)
	    if(NOT VIEWER_FOUND)
	       message(FATAL_ERROR "VIEWER could not be found")	       
	    endif()	
	    include_directories(${VIEWER_INCLUDE_DIRS})
	    link_directories(${VIEWER_LIBRARY_DIRS})    
        add_definitions(-DUSE_URDF)    
	endif()      
endif()

############################# LOAD PYTHON #############################
if(PKG_CONFIG_FOUND)    
    pkg_check_modules(PYTHON python-2.7)
    if(NOT PYTHON_FOUND)
       message(FATAL_ERROR "Python could not be found")
    endif()
    include_directories(${PYTHON_INCLUDE_DIRS})    
endif()

############################# LOAD RBDL_INTERFACE #############################
if(PKG_CONFIG_FOUND)    
    pkg_check_modules(RBDL rbdl_interface)
    if(NOT RBDL_FOUND)
       message(FATAL_ERROR "RBDL could not be found")
    endif()
    include_directories(${RBDL_INCLUDE_DIRS}) 
    message("RBDL include: ${RBDL_INCLUDE_DIRS}")   
endif()



############################# LOAD EIGEN #############################
if(PKG_CONFIG_FOUND)
    pkg_check_modules(EIGEN eigen3)
    if(NOT EIGEN_FOUND)
       message(FATAL_ERROR "EIGEN could not be found")
    endif()
    include_directories(${EIGEN_INCLUDE_DIRS}) 
    message("Eigen include dirs: ${EIGEN_INCLUDE_DIRS}")
    message("Eigen library dirs: ${EIGEN_LIBRARY_DIRS}")   
endif()


############################# LOAD BOOST #############################
find_package(Boost ${BOOST_MIN_VERSION} REQUIRED COMPONENTS python system thread random timer)
if (NOT Boost_FOUND)
    set(Boost_INCLUDE_DIRS "${BOOST_INCLUDE_DIRS}")
    IF ( NOT Boost_INCLUDE_DIRS )
        message(FATAL_ERROR "Please point the environment variable BOOST_INCLUDE_DIRS to the include directory of Boost")
    ENDIF()
endif (NOT Boost_FOUND)
include_directories(${Boost_INCLUDE_DIRS})

############################# LOAD FCL #############################
if(PKG_CONFIG_FOUND)
    pkg_check_modules(FCL fcl)
    if(NOT FCL_FOUND)
       message(FATAL_ERROR "FCL could not be found")
    endif()
    include_directories(${FCL_INCLUDE_DIRS})    
endif()

############################# LOAD OMPL #############################
if(PKG_CONFIG_FOUND)    
    pkg_check_modules(OMPL ompl)
    if(NOT OMPL_FOUND)
       message(FATAL_ERROR "OMPL could not be found")
    endif()
    include_directories(${OMPL_INCLUDE_DIRS})    
endif()

############################# LOAD TINYXML #############################
if(PKG_CONFIG_FOUND)    
    pkg_check_modules(TINYXML tinyxml)
    if(NOT TINYXML_FOUND)
       message(FATAL_ERROR "TINYXML could not be found")
    endif()
    include_directories(${TINYXML_INCLUDE_DIRS})    
endif()

############################# LOAD VIEWER_INTERFACE #############################
if (USE_URDF)
	if(PKG_CONFIG_FOUND)    
	    pkg_check_modules(VIEWER viewer_interface)
	    if(NOT VIEWER_FOUND)
	       message(FATAL_ERROR "VIEWER could not be found")
	    endif()
	    include_directories(${VIEWER_INCLUDE_DIRS})
            add_definitions(-DUSE_URDF)    
	endif()
endif()

#get_cmake_property(_variableNames VARIABLES)
#foreach (_variableName ${_variableNames})
#    message(STATUS "${_variableName}=${${_variableName}}")
#endforeach()
link_directories(${TINYXML_LIBRARY_DIRS}
                 ${OMPL_LIBRARY_DIRS}                 
                 ${PYTHON_LIBRARY_DIRS}
                 ${FCL_LIBRARY_DIRS}
                 ${Boost_LIBRARY_DIRS}
                 ${EIGEN_LIBRARY_DIRS}
                 ${RBDL_LIBRARY_DIRS})

add_library(robot SHARED 
            robot.cpp
            propagator.cpp                      
            integrate.cpp
            Kinematics.cpp)
            
add_library(path_planner SHARED
            path_planner.cpp
            MotionValidator.cpp
            ManipulatorGoalRegion.cpp
            robot.cpp
            Kinematics.cpp
            propagator.cpp
            integrate.cpp
            utils.cpp
            Terrain.cpp
            Obstacle.cpp
            BoxObstacle.cpp
            SphereObstacle.cpp)
            
add_library(dynamic_path_planner SHARED 
            dynamic_path_planner.cpp
            ManipulatorSpaceInformation.cpp
            DirectedControlSampler.cpp
            rrt_control.cpp
            EST_control.cpp 
            MotionValidator.cpp
            state_propagator.cpp
            propagator.cpp
            integrate.cpp            
            ManipulatorGoalRegion.cpp            
            Kinematics.cpp
            utils.cpp
            robot.cpp            
            Obstacle.cpp
            BoxObstacle.cpp
            SphereObstacle.cpp
            Terrain.cpp            
            control_space.cpp
            control_sampler.cpp)
            
add_library(obstacle SHARED
            Obstacle.cpp
            Terrain.cpp)
            
add_library(util SHARED 
            utils.cpp
            Terrain.cpp
            Obstacle.cpp
            BoxObstacle.cpp
            SphereObstacle.cpp)
            
add_library(area SHARED
            Area.cpp)

target_link_libraries (robot
                       ${URDF_LIBRARIES}
                       ${Boost_LIBRARIES}
                       ${PYTHON_LIBRARIES}
                       ${OPENRAVE_LIBRARIES}
                       ${FCL_LIBRARIES}
                       ${VIEWER_LIBRARIES}
                       ${TINYXML_LIBRARIES}
                       ${RBDL_LIBRARIES})

target_link_libraries(path_planner
                      ${Boost_LIBRARIES}
                      ${PYTHON_LIBRARIES}
                      ${OPENRAVE_LIBRARIES}                   
                      ${OMPL_LIBRARIES}
                      ${FCL_LIBRARIES}                      
                      ${URDF_LIBRARIES}
                      ${VIEWER_LIBRARIES}
                      ${TINYXML_LIBRARIES}
                      ${RBDL_LIBRARIES})
                      
target_link_libraries (dynamic_path_planner                        
                       ${OPENRAVE_LIBRARIES}
                       ${OMPL_LIBRARIES}
                       ${Boost_LIBRARIES}
                       ${PYTHON_LIBRARIES}                   
                       ${FCL_LIBRARIES}
                       ${TINYXML_LIBRARIES}
                       ${URDF_LIBRARIES}
                       ${VIEWER_LIBRARIES}
                       ${RBDL_LIBRARIES})                       
                      
target_link_libraries(obstacle                      
                      ${Boost_LIBRARIES}
                      ${PYTHON_LIBRARIES}
                      ${FCL_LIBRARIES})
                      
target_link_libraries (util
                       ${Boost_LIBRARIES}
                       ${PYTHON_LIBRARIES}                       
                       ${FCL_LIBRARIES}
                       ${TINYXML_LIBRARIES})
                       
target_link_libraries(area
                      ${Boost_LIBRARIES}
                      ${PYTHON_LIBRARIES})
