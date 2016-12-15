###############################################################################
# Macro
###############################################################################

macro(cost_map_add_utility name)
  add_executable(${name} ${name}.cpp)
  target_link_libraries(${name} ${catkin_LIBRARIES} ${PROJECT_NAME})
  install(TARGETS ${name} RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endmacro()

###############################################################################
# Setup
###############################################################################

cost_map_add_utility(alignment_test)
cost_map_add_utility(publish_image_bundle)
