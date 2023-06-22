/* Include the controller definition */
#include "footbot_aggregation_one.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>

#define ___LOG(comment,data) std::cout << (comment) << (data) << std::endl
#define __LOG_XML_DATA 1
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
                                                   current_hopcount_(0u),
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

void CFootBotAggregationOne::ControlStep() 
{
   const argos::CCI_FootBotProximitySensor::TReadings& proximity_readings = proximity_sen_->GetReadings();

   argos::CVector2 cAccumulator;

   for(size_t i = 0; i < proximity_readings.size(); ++i) 
   {
      if(i < 3) 
      {
         std::cout <<
         "Sensor"  << (i+1) <<
         " = V:"   << proximity_readings[i].Value <<
         ", A:"    << proximity_readings[i].Angle.GetValue() <<
         std::endl;
      }
      cAccumulator += argos::CVector2(proximity_readings[i].Value, proximity_readings[i].Angle);
   }

   cAccumulator /= proximity_readings.size();

   std::cout << "Average Value is " << cAccumulator.Length() << std::endl;

   argos::CRadians cAngle = cAccumulator.Angle();

   wheels_->SetLinearVelocity(wheel_velocity_,wheel_velocity_);
}

REGISTER_CONTROLLER(CFootBotAggregationOne, "footbot_aggregation_one")
