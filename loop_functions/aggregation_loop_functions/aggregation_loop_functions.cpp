#include "aggregation_loop_functions.h"


static const char* first_line_file = "Clock\tId\tHopCount\tForgettingActive\tNumConnections\tInTrgtAgentZone";

CAggregationLoopFunctions::CAggregationLoopFunctions():
   file_name_("aggregation.txt"),
   file_stream_(),
   radius_within_target_(0.1),
   bot_entities_(),
   target_bot_(nullptr),
   file_out_in_csv_(false){}

void CAggregationLoopFunctions::Init(argos::TConfigurationNode& t_node)
{
   argos::TConfigurationNode& aggregation_node = argos::GetNode(t_node, "aggregation");
   argos::GetNodeAttribute(aggregation_node,"file_name", file_name_);
   argos::GetNodeAttribute(aggregation_node,"log_as_csv",file_out_in_csv_);
   argos::GetNodeAttribute(aggregation_node,"radius_within_trgt_fb",radius_within_target_);

   file_stream_.open(file_name_,std::ios_base::trunc | std::ios_base::out);
   if(!file_out_in_csv_) file_stream_ << first_line_file << std::endl;

   const argos::CRange<argos::CVector3> arena_limits = GetSpace().GetArenaLimits();
   const argos::CVector3 arena_center = GetSpace().GetArenaCenter();
   const auto arena_size = GetSpace().GetArenaSize();

   const auto bots = GetSpace().GetEntitiesByType("foot-bot");
   for(auto& [key,value] : bots)
      bot_entities_.push_back(argos::any_cast<argos::CFootBotEntity*>(value));
      
   target_bot_ = bot_entities_.at(0);
   if(target_bot_ == nullptr) throw std::runtime_error("Target bot ptr was null in loop function init");

   CFootBotAggregationOne& ctrlr = dynamic_cast<CFootBotAggregationOne&>(target_bot_->GetControllableEntity().GetController());
   ctrlr.SetPersitantHopCount(0);

}

argos::CColor CAggregationLoopFunctions::GetFloorColor(const argos::CVector2& c_position_on_plane) {return argos::CColor::GRAY50;}

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

void CAggregationLoopFunctions::Destroy(){file_stream_.close();}

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
                        << IsWithinTargetBotRadius(ctrlr.GetPosition())     << std::endl;
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
                        << IsWithinTargetBotRadius(ctrlr.GetPosition())     << std::endl;
      }            
   }

}

std::string CAggregationLoopFunctions::IsWithinTargetBotRadius(argos::CVector3 position)
{
   if(target_bot_ == nullptr) return "false";
   
   const argos::CVector3 pos = dynamic_cast<const CFootBotAggregationOne&>(target_bot_->GetControllableEntity().GetController()).GetPosition();
   
   return (sqrt(pow(position.GetX()-pos.GetX(),2) + pow(position.GetY()-pos.GetY(),2)) <= radius_within_target_) ?
      "true" :
      "false";

}
REGISTER_LOOP_FUNCTIONS(CAggregationLoopFunctions, "aggregation_loop_functions")