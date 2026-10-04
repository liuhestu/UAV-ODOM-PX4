cmake_minimum_required(VERSION 3.0.2)
project(uav_system)
find_package(catkin REQUIRED)
catkin_package()
# Install actual root contents, never the source-space symlinks.
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../../config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../../launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
