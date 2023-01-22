cmake_minimum_required(VERSION 3.9)
project(rigid2d)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  tf2_ros 
  geometry_msgs
  std_msgs
  message_generation
  catch_ros)

add_service_files(
  FILES
  set_pose.srv
 )

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  #CATKIN_DEPENDS roscpp
 )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# create the rigid2d library and add its include directories
add_library(${PROJECT_NAME} 
            src/rigid2d.cpp
            src/diff_drive.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

catkin_package(INCLUDE_DIRS include
               LIBRARIES ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# odometry node
add_executable(odometry_node src/odometry_node.cpp)
target_link_libraries(odometry_node 
                      ${PROJECT_NAME} 
                      ${catkin_LIBRARIES})

# fake turtle node
add_executable(fake_turtle_node src/fake_turtle_node.cpp)
target_link_libraries(fake_turtle_node 
                      ${PROJECT_NAME} 
                      ${catkin_LIBRARIES})

# create the executable and link it with the rigid2d library
add_executable(rigid2d_main src/main.cpp)
target_link_libraries(rigid2d_main ${PROJECT_NAME})

catch_add_test(rigid2d_test tests/rigid2d_tests.cpp)
target_link_libraries(rigid2d_test ${PROJECT_NAME} ${catkin_LIBRARIES})

#enable_testing()
#add_executable(rigid2d_test test/test.cpp)
#target_include_directories(rigid2d_test PUBLIC include/)
#target_link_libraries(rigid2d_test rigid2d)
#add_test(NAME "Testing_rigid2d_lib" COMMAND rigid2d_test)

find_package(Doxygen)
# Use the readme in your doxygen docs
set(DOXYGEN_USE_MDFILE_AS_MAINPAGE README.md) 
doxygen_add_docs(doxygen_rigid2d include/ src/ README.md ALL)
