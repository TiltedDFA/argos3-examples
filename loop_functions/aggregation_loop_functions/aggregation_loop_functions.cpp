#include "aggregation_loop_functions.h"


static const char* first_line_file = "Clock\tId\tHopCount\tForgettingActive\tNumConnections\tInAZone";

TargetArea::TargetArea(const argos::CVector2& area_center, argos::Real area_radius, argos::Real secondary_offset): 
         area_center_(area_center),
         area_radius_(area_radius),
         secondary_offset_(secondary_offset){}

bool TargetArea::PointWithinTargetArea(const argos::CVector2& point)const
{
   return 
      sqrt(pow(point.GetX()-area_center_.GetX(),2) + pow(point.GetY()-area_center_.GetY(),2)) 
      <= area_radius_;
}
bool TargetArea::PointWithinSecondaryArea(const argos::CVector2& point)const
{
   return 
      sqrt(pow(point.GetX()-area_center_.GetX(),2) + pow(point.GetY()-area_center_.GetY(),2)) 
      <= (area_radius_ + secondary_offset_);
}
CAggregationLoopFunctions::CAggregationLoopFunctions():
   file_name_("aggregation.txt"),
   file_stream_(),
   target_areas_(),
   bot_entities_(),
   file_out_in_csv_(false){}

void CAggregationLoopFunctions::Init(argos::TConfigurationNode& t_node)
{
   uint64_t num_target_areas{0};
   argos::Real target_area_radius{0};
   argos::Real secondary_area_offset{0};
   bool default_area_config{false};

   argos::TConfigurationNode& aggregation_node = argos::GetNode(t_node, "aggregation");
   argos::GetNodeAttribute(aggregation_node,"file_name", file_name_);
   argos::GetNodeAttribute(aggregation_node,"log_as_csv",file_out_in_csv_);
   argos::GetNodeAttribute(aggregation_node,"default_area_config",default_area_config);
   argos::GetNodeAttribute(aggregation_node,"num_target_areas", num_target_areas);
   argos::GetNodeAttribute(aggregation_node,"target_area_size", target_area_radius);
   argos::GetNodeAttribute(aggregation_node,"secondary_area_offset", secondary_area_offset);

   file_stream_.open(file_name_,std::ios_base::trunc | std::ios_base::out);
   if(!file_out_in_csv_) file_stream_ << first_line_file << std::endl;

   const argos::CRange<argos::CVector3> arena_limits = GetSpace().GetArenaLimits();
   const argos::CVector3 arena_center = GetSpace().GetArenaCenter();
   const auto arena_size = GetSpace().GetArenaSize();
   if(default_area_config)
   {
      target_areas_.emplace_back(
         argos::CVector2(arena_center.GetX(), arena_center.GetY() - arena_size.GetY()/2.5),
         arena_size.GetX()/25,
         (arena_size.GetX()/25)
         );
      target_areas_.emplace_back(
         argos::CVector2(arena_center.GetX(), arena_center.GetY() + arena_size.GetY()/2.5),
         arena_size.GetX()/25,
         (arena_size.GetX()/25)
         );
   }
   else
   {
      std::uniform_real_distribution<> random_x_coord{arena_limits.GetMin().GetX(),arena_limits.GetMax().GetX()};
      std::uniform_real_distribution<> random_y_coord{arena_limits.GetMin().GetY(),arena_limits.GetMax().GetY()}; 
      
      for(uint64_t i{0}; i < num_target_areas;++i)
      {
         target_areas_.emplace_back(
            argos::CVector2(
               static_cast<argos::Real>(random_x_coord(rng_)),
               static_cast<argos::Real>(random_y_coord(rng_))),
            target_area_radius,
            secondary_area_offset);
      }
   }

   const auto bots = GetSpace().GetEntitiesByType("foot-bot");
   for(auto& [key,value] : bots)
      bot_entities_.push_back(argos::any_cast<argos::CFootBotEntity*>(value));
   
   
}
argos::CColor CAggregationLoopFunctions::GetFloorColor(const argos::CVector2& c_position_on_plane) 
{
   for(const auto& i : target_areas_)
   {
      if(i.PointWithinTargetArea(c_position_on_plane)) return argos::CColor::BLACK;
      else if (i.PointWithinSecondaryArea(c_position_on_plane)) return argos::CColor::GRAY50;
   }
   return argos::CColor::WHITE;
}
void CAggregationLoopFunctions::Reset()
{
   file_stream_.close();
   file_stream_.open(file_name_,std::ios_base::trunc | std::ios_base::out);
   if(!file_out_in_csv_) file_stream_ << first_line_file << std::endl;
   for(const auto i : bot_entities_)
   {
      dynamic_cast<CFootBotAggregationOne&>(i->GetControllableEntity().GetController()).ResetHopCount();
   }
}
void CAggregationLoopFunctions::Destroy()
{
   file_stream_.close();
}
void CAggregationLoopFunctions::PreStep()
{
   if(file_out_in_csv_)
   {
      for(const argos::CFootBotEntity* i : bot_entities_)
      {
         const CFootBotAggregationOne& ctrlr = dynamic_cast<const CFootBotAggregationOne&>(i->GetControllableEntity().GetController());
         file_stream_   << GetSpace().GetSimulationClock()  << ','
                        << i->GetId()                       << ','
                        << ctrlr.GetHopCount()              << ','
                        << ctrlr.GetForgettingState()       << ','
                        << ctrlr.GetNumConnections()        << ',' 
                        << ctrlr.GetWithinAreaState()       << std::endl;
      }
   }
   else
   {
      for(const argos::CFootBotEntity* i : bot_entities_)
      {
         const CFootBotAggregationOne& ctrlr = dynamic_cast<const CFootBotAggregationOne&>(i->GetControllableEntity().GetController());
         file_stream_   << GetSpace().GetSimulationClock()  << "\t\t"
                        << i->GetId()                       << "\t"
                        << ctrlr.GetHopCount()              << "\t\t\t"
                        << ctrlr.GetForgettingState()       << "\t\t\t\t"
                        << ctrlr.GetNumConnections()        << "\t\t\t\t" 
                        << ctrlr.GetWithinAreaState()       << std::endl;
      }            
   }
}
REGISTER_LOOP_FUNCTIONS(CAggregationLoopFunctions, "aggregation_loop_functions")