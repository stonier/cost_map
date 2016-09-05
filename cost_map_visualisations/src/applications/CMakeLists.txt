###############################################################################
# Macro
###############################################################################

macro(cost_map_visualisations_add_utility name)
  add_executable(${name} ${name}.cpp)
  target_link_libraries(${name} ${catkin_LIBRARIES} ${PROJECT_NAME})
  install(TARGETS ${name} RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endmacro()

###############################################################################
# Setup
###############################################################################

cost_map_visualisations_add_utility(node)
cost_map_visualisations_add_utility(service_relay)
cost_map_visualisations_add_utility(demo_inflations)
