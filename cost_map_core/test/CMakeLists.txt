##############################################################################
# Tests
##############################################################################

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
# Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test
  test_cost_map_core.cpp
  CostMapTest.cpp
  EllipseIteratorTest.cpp
  SubmapIteratorTest.cpp
  PolygonIteratorTest.cpp
)
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
endif()

