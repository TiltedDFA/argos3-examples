#include "aggregation_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <controllers/footbot_foraging/footbot_foraging.h>

CAggregationLoopFunctions::CAggregationLoopFunctions(){}

argos::CColor CAggregationLoopFunctions::GetFloorColor(const argos::CVector2& c_position_on_plane) 
{
   if(c_position_on_plane.GetX() < -1.0f) {
      return argos::CColor::GRAY50;
   }
//    for(argos::UInt32 i = 0; i < m_cFoodPos.size(); ++i) {
//       if((c_position_on_plane - m_cFoodPos[i]).SquareLength() < m_fFoodSquareRadius) {
//          return argos::CColor::BLACK;
//       }
//    }
   return argos::CColor::WHITE;
}

void CAggregationLoopFunctions::Init(argos::TConfigurationNode& t_node)
{
    argos::TConfigurationNode& aggregation_node = argos::GetNode(t_node, "aggregation");
    argos::GetNodeAttribute(aggregation_node,"file_name", some_string_);
    file_stream_.open(some_string_,std::ios_base::trunc | std::ios_base::out);
    file_stream_ << "This is test" << std::endl;
}
REGISTER_LOOP_FUNCTIONS(CAggregationLoopFunctions, "aggregation_loop_functions")