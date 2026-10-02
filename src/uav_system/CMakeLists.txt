cmake_minimum_required(VERSION 3.0.2)
project(uav_system)
find_package(catkin REQUIRED COMPONENTS std_msgs message_generation)
# setuptools installs the entire cached build/lib tree, including removed modules.
# Refresh only this package's Python staging directories before its install script.
install(CODE "file(REMOVE_RECURSE
  \"${CMAKE_CURRENT_BINARY_DIR}/lib/uav_core\"
  \"${CMAKE_CURRENT_BINARY_DIR}/lib/mission\"
  \"${CMAKE_CURRENT_BINARY_DIR}/lib/mission_executor\"
  \"${CMAKE_CURRENT_BINARY_DIR}/lib/flight_supervisor\"
  \"${CMAKE_CURRENT_BINARY_DIR}/lib/control_backend\")")
catkin_python_setup()
add_message_files(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/support/msg
                  FILES SourceStatus.msg SystemStatus.msg EvStatus.msg)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS std_msgs message_runtime)

# Removed relays can shadow domain packages after an incremental migration.
# Only remove catkin-generated wrappers, never arbitrary executable files.
foreach(old_node flight_supervisor.py control_backend_node.py commander_node.py takeoff_hover_land.py)
  set(old_relay "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/${old_node}")
  if(EXISTS "${old_relay}")
    file(READ "${old_relay}" relay_header LIMIT 256)
    if(relay_header MATCHES "generated from catkin/cmake/template/script.py.in")
      file(REMOVE "${old_relay}")
    endif()
  endif()
endforeach()

# Independent domain modules, exported by one ROS package.
catkin_install_python(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/src/state_source_manager/state_source_manager.py
  ${CMAKE_CURRENT_SOURCE_DIR}/src/state_adapter/state_adapter.py
  ${CMAKE_CURRENT_SOURCE_DIR}/src/flight_supervisor/flight_supervisor_node.py
  ${CMAKE_CURRENT_SOURCE_DIR}/src/flight_supervisor/px4_ev_observer.py
  ${CMAKE_CURRENT_SOURCE_DIR}/src/control_backend/px4_backend.py
  ${CMAKE_CURRENT_SOURCE_DIR}/src/mission_executor/mission_executor_node.py
  ${CMAKE_CURRENT_SOURCE_DIR}/test/mock_state_source.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/mission/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src/mission
        FILES_MATCHING PATTERN "*.py" PATTERN "__pycache__" EXCLUDE)
