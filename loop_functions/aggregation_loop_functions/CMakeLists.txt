link_directories(${CMAKE_BINARY_DIR}/controllers/footbot_aggregation_one)

set(aggregation_user_functions_SOURCES
  aggregation_user_functions.h
  aggregation_user_functions.cpp)

add_library(aggregation_user_functions MODULE
  ${aggregation_user_functions_SOURCES})
  
target_link_libraries(aggregation_user_functions
  footbot_aggregation_one
  argos3core_simulator
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot)

if(ARGOS_QTOPENGL_FOUND)
  target_link_libraries(aggregation_user_functions argos3plugin_simulator_qtopengl)
endif(ARGOS_QTOPENGL_FOUND)