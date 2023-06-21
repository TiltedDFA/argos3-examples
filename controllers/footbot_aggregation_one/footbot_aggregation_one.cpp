/* Include the controller definition */
#include "footbot_aggregation_one.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
#define __MALMACRO_LOG(comment,data) std::cout << (comment) << (data) << std::endl
/****************************************/
/****************************************/

CFootBotAggregationOne::CFootBotAggregationOne():
                                                   wheels_(NULL),
                                                   proximity_sen_(NULL),
                                                   wheel_velocity_(2.5f),
                                                   navigation_threshold_(
                                                      -argos::ToRadians(ALPHA),
                                                      argos::ToRadians(ALPHA)) {}

void CFootBotAggregationOne::Init(argos::TConfigurationNode& t_node) 
{
   wheels_    = GetActuator<argos::CCI_DifferentialSteeringActuator>("differential_steering");

   proximity_sen_ = GetSensor  <argos::CCI_FootBotProximitySensor      >("footbot_proximity"    );


   navigation_threshold_.Set(-ToRadians(ALPHA), ToRadians(ALPHA));

   argos::GetNodeAttributeOrDefault(t_node, "velocity", wheel_velocity_, wheel_velocity_);
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
