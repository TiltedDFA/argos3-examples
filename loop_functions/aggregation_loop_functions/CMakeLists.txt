link_directories(${CMAKE_BINARY_DIR}/controllers/footbot_aggregation_one)

set(aggregation_loop_functions_SOURCES
  aggregation_loop_functions.h
  aggregation_loop_functions.cpp)


if(ARGOS_QTOPENGL_FOUND)
  include_directories(${ARGOS_QTOPENGL_INCLUDE_DIRS})
  set(aggregation_loop_functions_SOURCES
      ${aggregation_loop_functions_SOURCES}
      aggregation_user_functions.h 
      aggregation_user_functions.cpp)
endif(ARGOS_QTOPENGL_FOUND)

add_library(aggregation_loop_functions MODULE
  ${aggregation_loop_functions_SOURCES})
  
target_link_libraries(aggregation_loop_functions
  footbot_aggregation_one
  argos3core_simulator
  argos3plugin_simulator_dynamics2d
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_media)

if(ARGOS_QTOPENGL_FOUND)
  target_link_libraries(aggregation_loop_functions argos3plugin_simulator_qtopengl)
endif(ARGOS_QTOPENGL_FOUND)
