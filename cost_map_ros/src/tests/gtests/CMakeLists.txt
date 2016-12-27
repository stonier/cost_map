###############################################################################
# GTest setup
###############################################################################

macro(cost_map_ros_add_gtest TESTNAME)
  catkin_add_gtest(cost_map_ros_test_${TESTNAME} ${TESTNAME}.cpp)
  target_link_libraries(cost_map_ros_test_${TESTNAME} ${PROJECT_NAME} ${catkin_LIBRARIES})
endmacro(cost_map_ros_add_gtest)

##############################################################################
# Tests
##############################################################################

cost_map_ros_add_gtest(to_grid_map)
