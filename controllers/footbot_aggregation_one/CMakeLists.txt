add_library(footbot_aggregation_one SHARED footbot_aggregation_one.h footbot_aggregation_one.cpp)
target_link_libraries(footbot_aggregation_one
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot)
