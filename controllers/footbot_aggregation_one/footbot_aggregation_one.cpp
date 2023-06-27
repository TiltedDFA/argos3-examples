/* Include the controller definition */
#include "footbot_aggregation_one.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>

#define ___LOG(comment,data) std::cout << (comment) << (data) << std::endl
#define __LOG_XML_DATA 1
#define __LOG_PROX_DATA 0
/****************************************/
/****************************************/
CRotationHandler::CRotationHandler(argos::Real robo_wheel_vel)
   :
   rot_frames_remaining_(0),
   time_needed_180_(static_cast<uint8_t>(223/robo_wheel_vel))
{}
void CRotationHandler::FindTimeNeededFor180(argos::Real robo_wheel_vel)
{
   time_needed_180_ = static_cast<uint8_t>(223/robo_wheel_vel);
}
void CRotationHandler::FindFramesNeeded(argos::Real desired_turning_angle)
{
   if(desired_turning_angle == 0)return;
   if(desired_turning_angle < 0)
   {
      desired_turning_angle *= -1;
      rot_frames_remaining_ = static_cast<int8_t>(-((desired_turning_angle/180)*time_needed_180_));
      return;
   }
   rot_frames_remaining_ = static_cast<int8_t>((desired_turning_angle/180)*time_needed_180_);
}
void CRotationHandler::ApproachZero()
{
   if(rot_frames_remaining_ == 0)return;
   if(rot_frames_remaining_ < 0)
   {
      ++rot_frames_remaining_;
      return;
   }
   --rot_frames_remaining_;
}
/****************************************/
/****************************************/
CHopCountManager::CHopCountManager( bool forgetting_allowed_,
                                    uint16_t hopcount_max_,
                                    uint16_t forgetting_time_period_)
   :forgetting_allowed_(forgetting_allowed_),
   hopcount_max_(hopcount_max_),
   forgetting_time_period_(forgetting_time_period_),
   forget_tp_counter_(0),
   current_hopcount_(hopcount_max_)
{}
/****************************************/
/****************************************/
CFootBotAggregationOne::CFootBotAggregationOne():
                                                   wheels_(NULL),
                                                   proximity_sen_(NULL),
                                                   rnb_actuator_(NULL),
                                                   rnb_sensor_(NULL),               
                                                   wheel_velocity_(2.5f),
                                                   delta_(0.5f),
                                                   alpha_(10.0f),
                                                   hopcount_max_(100u),
                                                   forgetting_allowed_(false),
                                                   forgetting_time_period_(2000u),
                                                   rotation_handler_(wheel_velocity_),
                                                   current_hopcount_(hopcount_max_),
                                                   navigation_threshold_
                                                   (
                                                      -argos::ToRadians(alpha_),
                                                      argos::ToRadians(alpha_)
                                                   ) {}

void CFootBotAggregationOne::Init(argos::TConfigurationNode& t_node) 
{
   wheels_            =    GetActuator<argos::CCI_DifferentialSteeringActuator>("differential_steering");
   proximity_sen_     =    GetSensor  <argos::CCI_FootBotProximitySensor      >("footbot_proximity"    );
   rnb_actuator_      =    GetActuator<argos::CCI_RangeAndBearingActuator     >("range_and_bearing"    );
   rnb_sensor_        =    GetSensor  <argos::CCI_RangeAndBearingSensor       >("range_and_bearing"    );

   argos::GetNodeAttributeOrDefault<argos::Real>(t_node, "velocity", wheel_velocity_, wheel_velocity_);
   argos::GetNodeAttributeOrDefault<argos::Real>(t_node, "delta", delta_, delta_);
   argos::GetNodeAttributeOrDefault<argos::CDegrees>(t_node, "alpha", alpha_, alpha_);
   argos::GetNodeAttributeOrDefault<uint16_t>(t_node, "hopcountmax", hopcount_max_, hopcount_max_);
   argos::GetNodeAttributeOrDefault<bool>(t_node, "forgettingallowed", forgetting_allowed_, forgetting_allowed_);
   argos::GetNodeAttributeOrDefault<uint16_t>(t_node, "forgettingtimeperiod", forgetting_time_period_, forgetting_time_period_);
   
   rotation_handler_ = CRotationHandler(wheel_velocity_);
   rotation_handler_.FindFramesNeeded(rng_angle_(rng_));
   navigation_threshold_.Set(-argos::ToRadians(alpha_), argos::ToRadians(alpha_));

#if __LOG_XML_DATA == 1
   ___LOG("velocity: ",wheel_velocity_);
   ___LOG("delta: ",delta_);
   ___LOG("alpha: ",alpha_);
   ___LOG("HCmax: ",hopcount_max_);
   ___LOG("Forgetting?: ",forgetting_allowed_);
   ___LOG("Forgetting time period: ", forgetting_time_period_);
#endif
}
static inline argos::Real FindAvg(const CCI_FootBotProximitySensor::TReadings& readings)
{
   argos::CVector2 cAccumulator{0};
   for(size_t i = 0; i < tProxReads.size(); ++i) 
   {
      cAccumulator += argos::CVector2(tProxReads[i].Value, tProxReads[i].Angle);
   }
   return (cAccumulator /= tProxReads.size()).Length();
}
void CFootBotAggregationOne::ControlStep() 
{
   //Turning
   if(rotation_handler_.rot_frames_remaining_ != 0)
   {
      if(rotation_handler_.rot_frames_remaining_ < 0)
      {
         wheels_->SetLinearVelocity(-wheel_velocity_,wheel_velocity_);
         rotation_handler_.ApproachZero();
      }
      else
      {
         wheels_->SetLinearVelocity(wheel_velocity_,-wheel_velocity_);
         rotation_handler_.ApproachZero();
      }
      return;
   }
  //if target found
  if(0)
  {
      current_hopcount_ = 0;
      
      return;
  }
  argos::Real avg_prox_sen_angle = FindAvg(proximity_sen_->GetReadings());
  if(avg_prox_sen_angle < delta_);
  //if()
  wheels_->SetLinearVelocity(wheel_velocity_,wheel_velocity_);
}
REGISTER_CONTROLLER(CFootBotAggregationOne, "footbot_aggregation_one")
