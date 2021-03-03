cmake_minimum_required(VERSION 3.3 FATAL_ERROR)
### Project name
project(mav_exploration_eval)

## Use C++14
set(CMAKE_CXX_STANDARD 14)

add_definitions(-std=c++11)


### Find dependencies
find_package(catkin REQUIRED COMPONENTS # Must be same as CATKIN_DEPENDS
        roscpp
        voxblox_ros
        voxblox
)

## Find system libraries
find_package(Eigen3 REQUIRED)

find_package(Boost REQUIRED)

find_package(VTK COMPONENTS 
        vtkIOPLY
        vtkCommonColor
        vtkCommonCore
        vtkCommonDataModel
        vtkFiltersCore
        vtkFiltersModeling
        vtkIOXML
        vtkInteractionStyle
        # vtkRenderingContextOpenGL2
        # vtkRenderingCore
        # vtkRenderingFreeType
        # vtkRenderingGL2PSOpenGL2
        # vtkRenderingOpenGL2
        QUIET
)
if (NOT VTK_FOUND)
  message("Skipping eval_holes: ${VTK_NOT_FOUND_MESSAGE}")
  return ()
endif()
message (STATUS "VTK_VERSION: ${VTK_VERSION}")


############
## catkin ##
############

### Define package
catkin_package(
        INCLUDE_DIRS 
                include
                ${EIGEN3_INCLUDE_DIR} # This is only necessary because Eigen3 sets a non-standard EIGEN3_INCLUDE_DIR variable
        # LIBRARIES 
        #         ${PROJECT_NAME}
        CATKIN_DEPENDS # Must be same as CATKIN_DEPENDS
                roscpp
                voxblox_ros
                voxblox
)



###########
## Build ##
###########

# Specify additional locations of header files
include_directories(
        include
        ${THIS_PACKAGE_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR} # Set manually because Eigen sets a non standard INCLUDE DIR
        ${Boost_INCLUDE_DIRS} # Set because Boost is an internal dependency, not transitive.
        ${VTK_INCLUDE_DIRS}
)


if (VTK_VERSION VERSION_LESS "8.90.0")
        # old system
        include(${VTK_USE_FILE})
        add_executable(eval_holes 
                src/eval_holes.cpp
        )
        target_compile_features(eval_holes INTERFACE cxx_std_11)
        target_link_libraries(eval_holes
                ${catkin_LIBRARIES}
                ${VTK_LIBRARIES}
        )
else ()
        # include all components
        add_executable(eval_holes 
                src/eval_holes.cpp
        )
        target_compile_features(eval_holes INTERFACE cxx_std_11)
        target_link_libraries(eval_holes
                ${catkin_LIBRARIES}
                ${VTK_LIBRARIES}
        )
        # vtk_module_autoinit is needed
        vtk_module_autoinit(
                TARGETS eval_holes
                MODULES ${VTK_LIBRARIES}
        )
endif ()





#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
        TARGETS eval_holes
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
        DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.hpp"
)

# Mark other files for installation
install(
        FILES launch/eval_holes.launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
