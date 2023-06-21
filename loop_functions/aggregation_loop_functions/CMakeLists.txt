set(aggregation_loop_function_SOURCES
  aggregation_loop_function.h
  aggregation_loop_function.cpp)

add_library(aggregation_loop_function MODULE
  ${aggregation_loop_function_SOURCES})
target_link_libraries(aggregation_loop_function
  argos3core_simulator
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot)
