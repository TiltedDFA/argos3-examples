#include "footbot_aggregation_one.h"
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/math/vector2.h>
#include <unordered_map>

#define ___LOG(comment, data) std::cout << (comment) << (data) << std::endl
#define __LOG_XML_DATA 1
#define __LOG_PROX_DATA 0
#define __LOG_FORGETTING 1
#define __LOG_LOWEST_DATA_RNB 0
#define __LOG_GROUND_SEN 0
#define __LOG_OVER_TRGT_AREA 0
/****************************************/
/****************************************/
CRotationHandler::CRotationHandler(argos::Real robo_wheel_vel)
    : rot_frames_remaining_(0),
      time_needed_180_(static_cast<uint8_t>(223 / robo_wheel_vel))
{}
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

      rot_frames_remaining_ = static_cast<int8_t>((desired_turning_angle / 180 * time_needed_180_));
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
void CRotationHandler::NonZeroRotateTo(argos::Real desired_turning_angle)
{
   if(rot_frames_remaining_ != 0)
   {
      RotateTo(desired_turning_angle);
   }
}
/****************************************/
/****************************************/
CHopCountManager::CHopCountManager(bool forgetting_allowed, uint16_t hopcount_max, uint16_t forgetting_tp)
    : forgetting_enabled_(forgetting_allowed),
      forgetting_tp_(forgetting_tp),
      hop_count_max_(hopcount_max),
      forget_tp_counter_(0),
      current_hop_count_(hopcount_max),
      currently_forgetting_(false)
{}
bool CHopCountManager::update()
{
   if (!forgetting_enabled_)
   {
      std::cout << "Forgetting skipped" << std::endl;
      return false;
   }
   if (!currently_forgetting_ && forget_tp_counter_ == 0)
   {
      std::cout << "Activated forgetting " << std::endl;
      currently_forgetting_ = true;
      return true;
   }
   if (currently_forgetting_)
   {
      std::cout << "Currently forgetting " << std::endl;
      if (current_hop_count_ == hop_count_max_)
      {
         std::cout << "Deactivated forgetting " << std::endl;
         currently_forgetting_ = false;
         forget_tp_counter_ = forgetting_tp_;
         return false;
      }
      ++current_hop_count_;
      return true;
   }
   --forget_tp_counter_;
   return false;
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
                                                   hop_count_(CHopCountManager(0,50,200)),
                                                   rotation_handler_(wheel_velocity_)
{}
void CFootBotAggregationOne::Init(argos::TConfigurationNode &t_node)
{
   wheels_        = GetActuator< argos::CCI_DifferentialSteeringActuator>("differential_steering");
   proximity_sen_ = GetSensor<   argos::CCI_FootBotProximitySensor      >("footbot_proximity");
   rnb_actuator_  = GetActuator< argos::CCI_RangeAndBearingActuator     >("range_and_bearing");
   rnb_sensor_    = GetSensor<   argos::CCI_RangeAndBearingSensor       >("range_and_bearing");
   ground_sensor_ = GetSensor<   argos::CCI_FootBotMotorGroundSensor    >("footbot_motor_ground" );

   argos::GetNodeAttributeOrDefault(t_node, "Velocity", wheel_velocity_, wheel_velocity_);
   argos::GetNodeAttributeOrDefault(t_node, "Delta", delta_, delta_);
   argos::GetNodeAttributeOrDefault(t_node, "Alpha", alpha_, alpha_);
   argos::GetNodeAttributeOrDefault(t_node, "HopCountMax", hop_count_.hop_count_max_, hop_count_.hop_count_max_);
   argos::GetNodeAttributeOrDefault(t_node, "ForgettingAllowed", hop_count_.forgetting_enabled_, hop_count_.forgetting_enabled_);
   argos::GetNodeAttributeOrDefault(t_node, "ForgettingTimePeriod", hop_count_.forgetting_tp_, hop_count_.forgetting_tp_);
   
   hop_count_.current_hop_count_ = hop_count_.hop_count_max_;
   rotation_handler_ = std::move(CRotationHandler(wheel_velocity_));


#if __LOG_XML_DATA == 1
   ___LOG("velocity: ", wheel_velocity_);
   ___LOG("delta: ", delta_);
   ___LOG("alpha: ", alpha_);
   ___LOG("HCmax: ", hop_count_.hop_count_max_);
   ___LOG("CurrentHC: ", hop_count_.current_hop_count_);
   ___LOG("Forgetting?: ", hop_count_.forgetting_enabled_);
   ___LOG("HCTP: ", hop_count_.forgetting_tp_);
#endif
}
//for the proximity sensor
static inline argos::CVector2 FindAvg(const argos::CCI_FootBotProximitySensor::TReadings &readings)
{
   argos::CVector2 cAccumulator;
   for (size_t i = 0; i < readings.size(); ++i)
   {
      cAccumulator += argos::CVector2(readings[i].Value, readings[i].Angle);
   }
   return (cAccumulator / readings.size());
}
//For the motor sensor
static inline bool IsOverTargetArea(const argos::CCI_FootBotMotorGroundSensor::TReadings& readings)
{
   return   readings[0].Value < 0.1 || 
            readings[1].Value < 0.1 || 
            readings[2].Value < 0.1 || 
            readings[3].Value < 0.1;
}
void CFootBotAggregationOne::TransmitHCData()
{
   rnb_actuator_->ClearData();
   rnb_actuator_->SetData(0,hop_count_.current_hop_count_);
}
bool CFootBotAggregationOne::HandleTurning()
{
   if (rotation_handler_.rot_frames_remaining_ != 0)
   {
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
      return true;
   }
   return false;
}
bool CFootBotAggregationOne::AvoidCollisions()
{
   const argos::CVector2 avg_prox_sen_angle = FindAvg(proximity_sen_->GetReadings());
   if (avg_prox_sen_angle.Length() > delta_)
   {
      if (avg_prox_sen_angle.Angle().GetValue() > 0)
      {
         wheels_->SetLinearVelocity(wheel_velocity_, 0);
      }
      else
      {
         wheels_->SetLinearVelocity(0, wheel_velocity_);
      }
      return true;
   }
   return false;
}
bool CFootBotAggregationOne::HandleForgetting()
{
   if(hop_count_.update())
   {
      rotation_handler_.NonZeroRotateTo(rng_angle_(rng_));
      return true;
   }
   return false;
}
bool CFootBotAggregationOne::HandleTargetArea()
{
   const argos::CCI_FootBotMotorGroundSensor::TReadings& gnd_sen_readings = ground_sensor_->GetReadings();
#if __LOG_GROUND_SEN == 1
   std::cout << 
      "Value of sen0: " << gnd_sen_readings[0].Value <<
      "Value of sen1: " << gnd_sen_readings[1].Value <<
      "Value of sen2: " << gnd_sen_readings[2].Value <<
      "Value of sen3: " << gnd_sen_readings[3].Value <<
   std::endl;
#endif
   if(IsOverTargetArea(gnd_sen_readings))
   {
#if __LOG_OVER_TRGT_AREA == 1
      std::cout << GetId() << " is over target area" << std::endl;
#endif
      hop_count_.current_hop_count_ = 0;
      rotation_handler_.NonZeroRotateTo(rng_angle_(rng_));
      return true;
   }
   return false;
}
bool CFootBotAggregationOne::ReadTransmitions()
{
   const argos::CCI_RangeAndBearingSensor::TReadings& rnb_readings = rnb_sensor_->GetReadings();
   if(rnb_readings.empty())
   {
      return false;
   }

   uint16_t min_hop_count{hop_count_.hop_count_max_}; 
   for(auto it = rnb_readings.cbegin(); it != rnb_readings.cend();++it)
   {
      if(it->Data[0] < min_hop_count) min_hop_count = it->Data[0];
   }

   if(min_hop_count >= hop_count_.current_hop_count_) return false;
   
   argos::CRadians total_bearing{0.0f};
   int64_t num_occurances{0};
   for(auto it = rnb_readings.cbegin(); it != rnb_readings.cend();++it)
   {
      if(it->Data[0] == min_hop_count)
      {
         ++num_occurances;
         total_bearing += it->HorizontalBearing;
      }
   }

   const argos::Real angle_to_smallest_hc = -argos::ToDegrees(total_bearing/num_occurances).GetValue();
   if(std::abs(angle_to_smallest_hc) > alpha_.GetValue())
   {
      rotation_handler_.RotateTo(angle_to_smallest_hc);
      hop_count_.current_hop_count_ = (min_hop_count + 1);
      return true;
   }

   return false;
}
void CFootBotAggregationOne::MoveForward()
{
   wheels_->SetLinearVelocity(wheel_velocity_, wheel_velocity_);
}
void CFootBotAggregationOne::ControlStep()
{
   TransmitHCData();
   if(HandleTurning())return;
   
   if(HandleTargetArea())return;
   if(HandleForgetting())return;
   if(ReadTransmitions())return;

   if(AvoidCollisions())return;


   MoveForward();
}
std::string CFootBotAggregationOne::GetHC()
{
   return std::to_string(hop_count_.current_hop_count_);
}
REGISTER_CONTROLLER(CFootBotAggregationOne, "footbot_aggregation_one")
