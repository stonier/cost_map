###############################################################################
# Macro
###############################################################################

macro(cost_map_demos_add_demo name)
  add_executable(${name} ${name}.cpp)
  target_link_libraries(${name} ${catkin_LIBRARIES} ${PROJECT_NAME})
  install(TARGETS ${name} RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endmacro()

###############################################################################
# Setup
###############################################################################

cost_map_demos_add_demo(inflations)
cost_map_demos_add_demo(from_ros_costmaps)
