#include "footbot_aggregation_one.h"
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/math/vector2.h>

#define ___LOG(comment, data) std::cout << (comment) << (data) << std::endl
#define __LOG_XML_DATA 1
#define __LOG_PROX_DATA 0
#define __LOG_FORGETTING 1
/****************************************/
/****************************************/
CRotationHandler::CRotationHandler(argos::Real robo_wheel_vel)
    : rot_frames_remaining_(0),
      time_needed_180_(static_cast<uint8_t>(223 / robo_wheel_vel))
{
}
void CRotationHandler::FindTimeNeededFor180(argos::Real robo_wheel_vel)
{
   time_needed_180_ = static_cast<uint8_t>(223 / robo_wheel_vel);
}
void CRotationHandler::RotateTo(argos::Real desired_turning_angle)
{
   if (desired_turning_angle > 180 || desired_turning_angle < -180)
   {
      std::cerr << "Incorrect angle input into CRotationHandler::RotateTo" << std::endl;
      return;
   }
   if (desired_turning_angle == 0)
      return;
   if (desired_turning_angle < 0)
   {
      desired_turning_angle *= -1;
      rot_frames_remaining_ = static_cast<int8_t>(-((desired_turning_angle / 180) * time_needed_180_));
      return;
   }
   rot_frames_remaining_ = static_cast<int8_t>((desired_turning_angle / 180) * time_needed_180_);
}
void CRotationHandler::ApproachZero()
{
   if (rot_frames_remaining_ == 0)
      return;
   if (rot_frames_remaining_ < 0)
   {
      ++rot_frames_remaining_;
      return;
   }
   --rot_frames_remaining_;
}
/****************************************/
/****************************************/
CHopCountManager::CHopCountManager(bool forgetting_allowed, uint8_t hopcount_max, uint16_t forgetting_tp)
    : forgetting_enabled_(forgetting_allowed),
      forgetting_tp_(forgetting_tp),
      hopcount_max_(hopcount_max),
      forget_tp_counter_(0),
      current_hopcount_(hopcount_max),
      currently_forgetting_(false)
{}
void CHopCountManager::update()
{
   if (!forgetting_enabled_)return;
   
   if (!currently_forgetting_ && forget_tp_counter_ == 0)
   {
      currently_forgetting_ = true;
      return;
   }
   if (currently_forgetting_)
   {
      std::cout << "Currently forgetting " << std::endl;
      if (current_hopcount_ == hopcount_max_)
      {
         currently_forgetting_ = false;
         forget_tp_counter_ = forgetting_tp_;
         return;
      }
      ++current_hopcount_;
      return;
   }
   --forget_tp_counter_;
}
/****************************************/
/****************************************/
CFootBotAggregationOne::CFootBotAggregationOne() : wheels_(NULL),
                                                   proximity_sen_(NULL),
                                                   rnb_actuator_(NULL),
                                                   rnb_sensor_(NULL),
                                                   wheel_velocity_(2.5f),
                                                   delta_(0.5f),
                                                   alpha_(10.0f),
                                                   hop_count_(false, 127,2000),
                                                   rotation_handler_(wheel_velocity_),
                                                   navigation_threshold_(
                                                       -argos::ToRadians(alpha_),
                                                       argos::ToRadians(alpha_)) {}

void CFootBotAggregationOne::Init(argos::TConfigurationNode &t_node)
{
   wheels_ = GetActuator<argos::CCI_DifferentialSteeringActuator>("differential_steering");
   proximity_sen_ = GetSensor<argos::CCI_FootBotProximitySensor>("footbot_proximity");
   rnb_actuator_ = GetActuator<argos::CCI_RangeAndBearingActuator>("range_and_bearing");
   rnb_sensor_ = GetSensor<argos::CCI_RangeAndBearingSensor>("range_and_bearing");

   argos::GetNodeAttributeOrDefault<argos::Real>(t_node, "Velocity", wheel_velocity_, wheel_velocity_);
   argos::GetNodeAttributeOrDefault<argos::Real>(t_node, "Delta", delta_, delta_);
   argos::GetNodeAttributeOrDefault<argos::CDegrees>(t_node, "Alpha", alpha_, alpha_);
   argos::GetNodeAttributeOrDefault<uint8_t>(t_node, "HopCountMax", hop_count_.hopcount_max_, hop_count_.hopcount_max_);
   argos::GetNodeAttributeOrDefault<bool>(t_node, "ForgettingAllowed", hop_count_.forgetting_enabled_, hop_count_.forgetting_enabled_);
   argos::GetNodeAttributeOrDefault<uint16_t>(t_node, "ForgettingTimePeriod", hop_count_.forgetting_tp_, hop_count_.forgetting_tp_);

   rotation_handler_ = CRotationHandler(wheel_velocity_);
   rotation_handler_.RotateTo(rng_angle_(rng_));
   navigation_threshold_.Set(-argos::ToRadians(alpha_), argos::ToRadians(alpha_));

#if __LOG_XML_DATA == 1
   ___LOG("velocity: ", wheel_velocity_);
   ___LOG("delta: ", delta_);
   ___LOG("alpha: ", alpha_);
   ___LOG("HCmax: ", hop_count_.hopcount_max_);
   ___LOG("Forgetting?: ", hop_count_.forgetting_enabled_);
   ___LOG("HCTP: ", hop_count_.forgetting_tp_);
#endif
}
static inline argos::CVector2 FindAvg(const argos::CCI_FootBotProximitySensor::TReadings &readings)
{
   argos::CVector2 cAccumulator;
   for (size_t i = 0; i < readings.size(); ++i)
   {
      cAccumulator += argos::CVector2(readings[i].Value, readings[i].Angle);
   }
   return (cAccumulator / readings.size());
}
static inline std::vector<argos::CCI_RangeAndBearingSensor::SPacket>::iterator FindMinSensorReading(argos::CCI_RangeAndBearingSensor::TReadings& rnb_readings, uint16_t hop_count_max)
{
   auto min_hc = hop_count_max;
   auto min_index = 0;
   for(int i = 0; i < rnb_readings.size();++i)
   {
      if(rnb_readings[i].Data[0] < hop_count_max && rnb_readings[i].Data[0] < rnb_readings[min_index].Data[0])
      {
         min_index = i;
         min_hc = rnb_readings[i].Data[0];
      }
   }
   if(rnb_readings[min_index].Data[0] == hop_count_max)
   {
      return rnb_readings.end();
   }
   return rnb_readings.begin() + min_index;
}
void CFootBotAggregationOne::ControlStep()
{
   rnb_actuator_->ClearData();
   rnb_actuator_->SetData(0,hop_count_.current_hopcount_);
   // Turning
   if (rotation_handler_.rot_frames_remaining_ != 0)
   {
      std::cout << GetId() << "Rotation manager reached" << std::endl;
      if (rotation_handler_.rot_frames_remaining_ < 0)
      {
         wheels_->SetLinearVelocity(-wheel_velocity_, wheel_velocity_);
         rotation_handler_.ApproachZero();
      }
      else
      {
         wheels_->SetLinearVelocity(wheel_velocity_, -wheel_velocity_);
         rotation_handler_.ApproachZero();
      }
      return;
   }
   // Avoid collisions
   const argos::CVector2 avg_prox_sen_angle = FindAvg(proximity_sen_->GetReadings());
   if (avg_prox_sen_angle.Length() > delta_)
   {
      std::cout << GetId() << "Collision avoidance reached" << std::endl;
      if (avg_prox_sen_angle.Angle().GetValue() > 0)
      {
         wheels_->SetLinearVelocity(wheel_velocity_, 0);
      }
      else
      {
         wheels_->SetLinearVelocity(0, wheel_velocity_);
      }
      return;
   }
   argos::CCI_RangeAndBearingSensor::TReadings rnb_readings = rnb_sensor_->GetReadings();
   const std::vector<argos::CCI_RangeAndBearingSensor::SPacket>::iterator rnb_reading_min = FindMinSensorReading(rnb_readings,hop_count_.hopcount_max_);
   if(rnb_readings.size() > 0 && rnb_reading_min != rnb_readings.end())
   {
      std::cout << GetId() << "Rotating to small hc" << std::endl;
      const argos::CRadians angle_to_smallest_hc = rnb_reading_min->HorizontalBearing;
      if(!navigation_threshold_.WithinMinBoundIncludedMaxBoundIncluded(angle_to_smallest_hc))
         rotation_handler_.RotateTo(-argos::ToDegrees(angle_to_smallest_hc).GetValue());
      return;
   }
   std::cout << GetId() << "Going straight" << std::endl;
   wheels_->SetLinearVelocity(wheel_velocity_, wheel_velocity_);
}
std::string CFootBotAggregationOne::GetHC()
{
   return std::to_string(hop_count_.current_hopcount_);
}
REGISTER_CONTROLLER(CFootBotAggregationOne, "footbot_aggregation_one")
