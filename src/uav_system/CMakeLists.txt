cmake_minimum_required(VERSION 3.0.2)
project(uav_system)
find_package(catkin REQUIRED COMPONENTS std_msgs message_generation)
catkin_python_setup()
add_message_files(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/support/msg
                  FILES SourceStatus.msg SystemStatus.msg EvStatus.msg)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS std_msgs message_runtime)

# Five independent code modules, exported by one ROS package.
catkin_install_python(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/src/state_source_manager/state_source_manager.py
  ${CMAKE_CURRENT_SOURCE_DIR}/src/state_adapter/state_adapter.py
  ${CMAKE_CURRENT_SOURCE_DIR}/src/flight_supervisor/flight_supervisor.py
  ${CMAKE_CURRENT_SOURCE_DIR}/src/flight_supervisor/px4_ev_observer.py
  ${CMAKE_CURRENT_SOURCE_DIR}/src/px4_backend/px4_backend.py
  ${CMAKE_CURRENT_SOURCE_DIR}/src/commander/takeoff_hover_land.py
  ${CMAKE_CURRENT_SOURCE_DIR}/test/mock_state_source.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
