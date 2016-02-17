###############################################################################
# Macro
###############################################################################

macro(cost_map_visualisations_add_global_utility name)
  add_executable(${name} ${name}.cpp)
  target_link_libraries(gopher_${name} ${catkin_LIBRARIES} ${PROJECT_NAME})
  set_target_properties(gopher_${name} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_BIN_DESTINATION})
  install(TARGETS gopher_${name} RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
endmacro()

macro(cost_map_visualisations_add_utility name)
  add_executable(${name} ${name}.cpp)
  target_link_libraries(${name} ${catkin_LIBRARIES} ${PROJECT_NAME})
  install(TARGETS ${name} RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endmacro()

###############################################################################
# Setup
###############################################################################

# cost_map_visualisations_add_global_utility(gopher_planner)
cost_map_visualisations_add_utility(node)
cost_map_visualisations_add_utility(service_relay)
cost_map_visualisations_add_utility(demo_inflations)
