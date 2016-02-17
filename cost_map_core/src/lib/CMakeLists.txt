############################################################################
# Sources
##############################################################################

file(GLOB SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)
file(GLOB ITERATOR_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} iterators/*.cpp)
file(GLOB OPERATOR_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} operators/*.cpp)

##############################################################################
# Library
##############################################################################

add_library(${PROJECT_NAME} ${SOURCES} ${ITERATOR_SOURCES} ${OPERATOR_SOURCES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

##############################################################################
# Installs
##############################################################################

install(TARGETS ${PROJECT_NAME}
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)