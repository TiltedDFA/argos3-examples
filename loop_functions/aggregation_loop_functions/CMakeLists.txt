set(aggregation_user_functions_SOURCES
  aggregation_user_functions.h
  aggregation_user_functions.cpp)

add_library(aggregation_user_functions MODULE
  ${aggregation_user_functions_SOURCES})
  
target_link_libraries(aggregation_user_functions
  argos3core_simulator
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot)
