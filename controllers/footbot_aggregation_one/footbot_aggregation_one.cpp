#include "footbot_aggregation_one.h"
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/math/vector2.h>


CRotationHandler::CRotationHandler(argos::Real robo_wheel_vel)
    : rot_frames_remaining_(0),
      time_needed_180_(static_cast<uint8_t>(223 / robo_wheel_vel)){}

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
   
   if (desired_turning_angle == 0)return;

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

int8_t CRotationHandler::GetRemainingRotationTime(){return rot_frames_remaining_;}

CHopCountManager::CHopCountManager(bool forgetting_allowed, uint16_t hopcount_max, uint16_t forgetting_tp)
    : forgetting_enabled_(forgetting_allowed),
      forgetting_tp_(forgetting_tp),
      max_hop_count_(hopcount_max),
      forget_tp_counter_(0),
      current_hop_count_(hopcount_max),
      currently_forgetting_(false){}

bool CHopCountManager::Update()
{
   if (!forgetting_enabled_)
   {
      return false;
   }

   if (!currently_forgetting_ && forget_tp_counter_ == 0)
   {
      currently_forgetting_ = true;
      return true;
   }

   if (!currently_forgetting_)
   {
      --forget_tp_counter_;
      return false;
   }

   if (current_hop_count_ == max_hop_count_)
   {
      currently_forgetting_ = false;
      forget_tp_counter_ = forgetting_tp_;
      return false;
   }

   ++current_hop_count_;
   return true;
}
void CHopCountManager::ResetHopCount()                                  {current_hop_count_ = max_hop_count_;}
void CHopCountManager::SetMaxHopCount(uint16_t max_hc)                  {max_hop_count_ = max_hc;}

void CHopCountManager::SetCurrentHopCount(uint16_t hc)                  {current_hop_count_ = hc;}

void CHopCountManager::SetForgettingEnabled(bool ForgettingAllowed)     {forgetting_enabled_ = ForgettingAllowed;}

void CHopCountManager::SetForgettingTimePeriod(uint16_t forgetting_tp)  {forgetting_tp_ = forgetting_tp;}

uint16_t CHopCountManager::GetMaxHopCount()const                        {return max_hop_count_;}

uint16_t CHopCountManager::GetCurrentHopCount()const                    {return current_hop_count_;}

uint16_t CHopCountManager::GetForgettingTimePeriod()const               {return forgetting_tp_;}

bool CHopCountManager::GetForgettingEnabled()const                      {return forgetting_enabled_;}

bool CHopCountManager::GetCurrentlyForgetting()const                    {return currently_forgetting_;}

CFootBotAggregationOne::CFootBotAggregationOne() 
    : wheels_(NULL),
      proximity_sen_(NULL),
      rnb_actuator_(NULL),
      rnb_sensor_(NULL),
      wheel_velocity_(2.5f),
      delta_(0.5f),
      alpha_(10.0f),
      hop_count_(true,99,1000),
      rotation_handler_(wheel_velocity_),
      num_connections_(0),
      within_secondary_area_(false){}

void CFootBotAggregationOne::Init(argos::TConfigurationNode &t_node)
{
   wheels_        = GetActuator< argos::CCI_DifferentialSteeringActuator>("differential_steering");
   rnb_actuator_  = GetActuator< argos::CCI_RangeAndBearingActuator     >("range_and_bearing");
   proximity_sen_ = GetSensor<   argos::CCI_FootBotProximitySensor      >("footbot_proximity");
   rnb_sensor_    = GetSensor<   argos::CCI_RangeAndBearingSensor       >("range_and_bearing");
   ground_sensor_ = GetSensor<   argos::CCI_FootBotMotorGroundSensor    >("footbot_motor_ground" );

   uint16_t xml_max_hop_count{0};
   bool xml_forgetting_allowed{false};
   uint16_t xml_forgetting_time_period{0};

   argos::GetNodeAttributeOrDefault(t_node, "Velocity", wheel_velocity_, wheel_velocity_);
   argos::GetNodeAttributeOrDefault(t_node, "Delta", delta_, delta_);
   argos::GetNodeAttributeOrDefault(t_node, "Alpha", alpha_, alpha_);
   argos::GetNodeAttributeOrDefault(t_node, "HopCountMax", xml_max_hop_count, hop_count_.GetMaxHopCount());
   argos::GetNodeAttributeOrDefault(t_node, "ForgettingAllowed", xml_forgetting_allowed, hop_count_.GetForgettingEnabled());
   argos::GetNodeAttributeOrDefault(t_node, "ForgettingTimePeriod", xml_forgetting_time_period, hop_count_.GetForgettingTimePeriod());
   
   hop_count_.SetMaxHopCount(xml_max_hop_count);
   hop_count_.SetForgettingEnabled(xml_forgetting_allowed);
   hop_count_.SetForgettingTimePeriod(xml_forgetting_time_period);
   hop_count_.SetCurrentHopCount(hop_count_.GetMaxHopCount());

   rotation_handler_ = std::move(CRotationHandler(wheel_velocity_));
}

void CFootBotAggregationOne::RealTimeRotate(const argos::CRadians& avg_bearing)
{
   if(avg_bearing.GetValue() == 0) return;

   if(avg_bearing.GetValue() > 0)
   {
      wheels_->SetLinearVelocity(wheel_velocity_ / 2, wheel_velocity_);
   }
   else
   {
      wheels_->SetLinearVelocity(wheel_velocity_, wheel_velocity_ / 2);
   }
}

void CFootBotAggregationOne::TransmitHCData()
{
   rnb_actuator_->ClearData();
   rnb_actuator_->SetData(0,hop_count_.GetCurrentHopCount());
}

bool CFootBotAggregationOne::HandleTurning()
{
   if (rotation_handler_.GetRemainingRotationTime() == 0) {return false;}

   if (rotation_handler_.GetRemainingRotationTime() < 0)
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

bool CFootBotAggregationOne::AvoidCollisions()
{
   auto readings = proximity_sen_->GetReadings();
   argos::CVector2 total_values;

   for (size_t i = 0; i < readings.size(); ++i)
   {
      total_values += argos::CVector2(readings[i].Value, readings[i].Angle);
   }

   const argos::CVector2 avg_prox_sen_angle = total_values / readings.size();

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
   if(hop_count_.Update())
   {
      rotation_handler_.NonZeroRotateTo(rng_angle_(rng_));
      return true;
   }

   return false;
}

bool CFootBotAggregationOne::HandleTargetArea()
{
   const argos::CCI_FootBotMotorGroundSensor::TReadings& gnd_sen_readings = ground_sensor_->GetReadings();

   within_secondary_area_ = (gnd_sen_readings[0].Value <= 0.5 || gnd_sen_readings[1].Value <= 0.5 || gnd_sen_readings[2].Value <= 0.5 || gnd_sen_readings[3].Value <= 0.5);
   if(gnd_sen_readings[0].Value < 0.1 || gnd_sen_readings[1].Value < 0.1 || gnd_sen_readings[2].Value < 0.1 || gnd_sen_readings[3].Value < 0.1)
   {
      hop_count_.SetCurrentHopCount(0);
      rotation_handler_.NonZeroRotateTo(rng_angle_(rng_));
      return true;
   }
   return false;
}

bool CFootBotAggregationOne::ReadTransmissions()
{
   argos::CCI_RangeAndBearingSensor::TReadings rnb_readings = rnb_sensor_->GetReadings();
   
   if(rnb_readings.empty()) {return false;}

   uint16_t min_hop_count{hop_count_.GetMaxHopCount()}; 
   
   num_connections_ = rnb_readings.size();

   for(std::size_t i = 0;i < num_connections_;++i)
   {
      if(rnb_readings[i].Data[0] < min_hop_count) min_hop_count = rnb_readings[i].Data[0];
   }

   if(min_hop_count >= hop_count_.GetMaxHopCount() || min_hop_count > hop_count_.GetCurrentHopCount())
      return false;
   
   argos::CRadians total_bearing{0.0f};
   argos::Real num_occurances{0};

   for(auto it = rnb_readings.cbegin(); it != rnb_readings.cend();++it)
   {
      if(it->Data[0] == min_hop_count)
      {
         ++num_occurances;
         total_bearing += it->HorizontalBearing;
      }
   }

   total_bearing /= num_occurances;
   hop_count_.SetCurrentHopCount(min_hop_count + 1);

   if(argos::ToDegrees(total_bearing).GetAbsoluteValue() > alpha_.GetValue())
   {
      RealTimeRotate(total_bearing);
      return true;
   }

   return false;
}

void CFootBotAggregationOne::MoveForward()
{
   wheels_->SetLinearVelocity(wheel_velocity_, wheel_velocity_);
}

/**
 * The control step and the private functions of CFootBotAggregationOne work with a similar system to interupts.
 * The private functions can return true or false based on whether they have done something that should be executed for 
 * the rest of the time step. For example, if HandleTurning() decides that it currently needs to turn then it will return
 * true in order to skip the rest of the code, else it will return false. This same technique is used with the other functions.
 * If ReadTransmissions() finds neighbours with a lower hop count it will then move towards the average vector of the robots with
 * the smallest hop count. In order for this movement to be carried out it will return true to avoid calling the MoveForward() method
 * which if it ran would then change the wheel speed to something else.  
 * 
 */
void CFootBotAggregationOne::ControlStep()
{
   TransmitHCData();
   if(HandleTurning())return;

   if(HandleTargetArea())return;
   if(HandleForgetting())return;
   if(ReadTransmissions())return;

   if(AvoidCollisions())return;
   MoveForward();
}

std::string CFootBotAggregationOne::GetHopCount()const
{
   return std::to_string(hop_count_.GetCurrentHopCount());
}
std::string CFootBotAggregationOne::GetNumConnections()const
{
   return std::to_string(num_connections_);
}
std::string CFootBotAggregationOne::GetWithinAreaState()const
{
   return within_secondary_area_ ? "true" : "false";
}
std::string CFootBotAggregationOne::GetForgettingState()const
{
   return hop_count_.GetCurrentlyForgetting() ? "true" : "false";
}
void CFootBotAggregationOne::ResetHopCount()
{
   hop_count_.ResetHopCount();
}
REGISTER_CONTROLLER(CFootBotAggregationOne, "footbot_aggregation_one")
