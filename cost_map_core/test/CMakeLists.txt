###############################################################################
# GTest setup
###############################################################################

macro(cost_map_core_add_gtest TESTNAME)
  catkin_add_gtest(cost_map_core_test_${TESTNAME} ${TESTNAME}.cpp)
  target_link_libraries(cost_map_core_test_${TESTNAME} ${PROJECT_NAME} ${catkin_LIBRARIES})
endmacro(cost_map_core_add_gtest)

##############################################################################
# Tests
##############################################################################

cost_map_core_add_gtest(cost_map_core)
cost_map_core_add_gtest(cost_map)
cost_map_core_add_gtest(ellipse_iterator)
cost_map_core_add_gtest(line_iterator)
cost_map_core_add_gtest(polygon_iterator)
cost_map_core_add_gtest(submap_iterator)
