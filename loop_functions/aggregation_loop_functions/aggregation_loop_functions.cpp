#include "aggregation_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <controllers/footbot_foraging/footbot_foraging.h>
#include <random>
#include <cmath>

TargetArea::TargetArea(const argos::CVector2& area_center, argos::Real area_radius): 
         area_center_(area_center),
         area_radius_(area_radius){}

bool TargetArea::PointWithinTargetArea(const argos::CVector2& point)const
{
   return 
      sqrt(pow(point.GetX()-area_center_.GetX(),2) + pow(point.GetY()-area_center_.GetY(),2)) 
      <= area_radius_;
}

CAggregationLoopFunctions::CAggregationLoopFunctions(){}

void CAggregationLoopFunctions::Init(argos::TConfigurationNode& t_node)
{
   uint64_t num_target_areas{0};
   argos::Real target_area_radius{0};

   argos::TConfigurationNode& aggregation_node = argos::GetNode(t_node, "aggregation");
   argos::GetNodeAttribute(aggregation_node,"file_name", some_string_);
   argos::GetNodeAttribute(aggregation_node,"num_target_areas", num_target_areas);
   argos::GetNodeAttribute(aggregation_node,"target_area_size", target_area_radius);

   file_stream_.open(some_string_,std::ios_base::trunc | std::ios_base::out);
   file_stream_ << "This is test" << std::endl;

   const argos::CRange<argos::CVector3> arena_limits = GetSpace().GetArenaLimits();
   std::uniform_real_distribution<> random_x_coord{arena_limits.GetMin().GetX(),arena_limits.GetMax().GetX()};
   std::uniform_real_distribution<> random_y_coord{arena_limits.GetMin().GetY(),arena_limits.GetMax().GetY()}; 
   for(uint64_t i{0}; i < num_target_areas;++i)
   {
      target_areas_.emplace_back(
         argos::CVector2(
            static_cast<argos::Real>(random_x_coord(rng_)),
            static_cast<argos::Real>(random_y_coord(rng_))),
         target_area_radius);
   }
}
argos::CColor CAggregationLoopFunctions::GetFloorColor(const argos::CVector2& c_position_on_plane) 
{
   for(const auto& i : target_areas_)
   {
      if(i.PointWithinTargetArea(c_position_on_plane)) return argos::CColor::BLACK;
   }
   return argos::CColor::WHITE;
}
void CAggregationLoopFunctions::Reset()
{
   // std::map<std::string, argos::CAny, std::less<std::string>> bots = GetSpace().GetEntitiesByType("foot-bot");
   // for(auto& [key,value] : bots)
   // {
   //    CFootBotEntity* val = any_cast<CFootBotEntity*>(value);
   //    dynamic_cast<CFootBotAggregationOne*>(val)->ResetHopCount();
   // }
}
void CAggregationLoopFunctions::Destroy(){}
void CAggregationLoopFunctions::PreStep(){}
REGISTER_LOOP_FUNCTIONS(CAggregationLoopFunctions, "aggregation_loop_functions")