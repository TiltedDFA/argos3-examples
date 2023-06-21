/* Include the controller definition */
#include "footbot_RnB_test.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
#define __MALMACRO_LOG(comment,data) std::cout << (comment) << (data) << std::endl
/****************************************/
/****************************************/
#define __LOG_PROX_READINGS 0
#define __LOG_RNB_ACT 0
#define __LOG_RNB_SEN 0

CFootBotRnBTest::CFootBotRnBTest() :
   wheels_(NULL),
   proximity_sen_(NULL),
   alpha_(10.0f),
   delta_(0.5f),
   wheel_velocity_(2.5f),
   turning_thresholds_(-ToRadians(alpha_),
                           ToRadians(alpha_)) {}

/****************************************/
/****************************************/

void CFootBotRnBTest::Init(TConfigurationNode& t_node) {

   wheels_    =            GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   proximity_sen_ =        GetSensor  <CCI_FootBotProximitySensor      >("footbot_proximity"    );
   rnb_actuator_      =    GetActuator<CCI_RangeAndBearingActuator     >("range_and_bearing"    );
   rnb_sensor_     =       GetSensor  <CCI_RangeAndBearingSensor       >("range_and_bearing"    );

   GetNodeAttributeOrDefault(t_node, "alpha", alpha_, alpha_);
   turning_thresholds_.Set(-ToRadians(alpha_), ToRadians(alpha_));
   GetNodeAttributeOrDefault(t_node, "delta", delta_, delta_);
   GetNodeAttributeOrDefault(t_node, "velocity", wheel_velocity_, wheel_velocity_);
}

/****************************************/
/****************************************/

void CFootBotRnBTest::ControlStep() {
   const CCI_FootBotProximitySensor::TReadings& prox_readings = proximity_sen_->GetReadings();

   rnb_actuator_->ClearData();
   //sends data on channel 0. (GetId()[2])&1) is used to send unique data based on robo ID.
   //I will need to implament a more robust way of doing this later. 
   rnb_actuator_->SetData(0,(GetId()[2])&1);

#if __LOG_RNB_SEN == 1
   std::cout << "Bot ID:" << GetId() << " SEND: " << static_cast<unsigned>((GetId()[2])&1) << std::endl;
#endif

   const auto& rnb_sensor_readings = rnb_sensor_->GetReadings();

#if __LOG_RNB_ACT == 1
   //recieves data and displays it. Should work with. need to
   for(size_t i = 0; i < rnb_sensor_readings.size(); ++i)
   {
      std::cout << "Bot ID:" << GetId() << " READ: " << (i+1) <<" DATA" << rnb_sensor_readings[i].Data[0] << std::endl;
   }
#endif

   CVector2 readings_avg;
   for(size_t i = 0; i < prox_readings.size(); ++i) 
   {
#if __LOG_PROX_READINGS == 1
      if(i < 3) 
      {
         std::cout <<
         "S" << (i+1) << 
         " = V:" << prox_readings[i].Value << 
         ", A:" << prox_readings[i].Angle.GetValue() << 
         std::endl;
      }
#endif
      readings_avg += CVector2(prox_readings[i].Value, prox_readings[i].Angle);
   }

   readings_avg /= prox_readings.size();
#if __LOG_PROX_READINGS == 1
   std::cout << "Average Value is " << readings_avg.Length() << std::endl;
#endif
   CRadians cAngle = readings_avg.Angle();

   if(turning_thresholds_.WithinMinBoundIncludedMaxBoundIncluded(cAngle) &&
      readings_avg.Length() < delta_ ) 
   {
      /* Go straight */
      wheels_->SetLinearVelocity(wheel_velocity_, wheel_velocity_);
   }
   else 
   {
      /* Turn, depending on the sign of the angle */
      if(cAngle.GetValue() > 0.0f) 
      {
         wheels_->SetLinearVelocity(wheel_velocity_, 0.0f);
      }
      else 
      {
         wheels_->SetLinearVelocity(0.0f, wheel_velocity_);
      }
   }
}

REGISTER_CONTROLLER(CFootBotRnBTest, "footbot_RnB_test_controller")
