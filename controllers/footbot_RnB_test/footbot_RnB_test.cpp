/* Include the controller definition */
#include "footbot_RnB_test.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
#define __MALMACRO_LOG(comment,data) std::cout << (comment) << (data) << std::endl
/****************************************/
/****************************************/

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
   const CCI_FootBotProximitySensor::TReadings& tProxReads = proximity_sen_->GetReadings();

   CVector2 cAccumulator;
   for(size_t i = 0; i < tProxReads.size(); ++i) {
      if(i < 3) std::cout << "S" << (i+1) << " = V:" << tProxReads[i].Value << ", A:" << tProxReads[i].Angle.GetValue() << std::endl;
      cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
   }

   cAccumulator /= tProxReads.size();

   std::cout << "Average Value is " << cAccumulator.Length() << std::endl;
   CRadians cAngle = cAccumulator.Angle();
   //wheels_->SetLinearVelocity(wheel_velocity_,wheel_velocity_);

   if(turning_thresholds_.WithinMinBoundIncludedMaxBoundIncluded(cAngle) &&
      cAccumulator.Length() < delta_ ) {
      /* Go straight */
      wheels_->SetLinearVelocity(wheel_velocity_, wheel_velocity_);
   }
   else {
      /* Turn, depending on the sign of the angle */
      if(cAngle.GetValue() > 0.0f) {
         wheels_->SetLinearVelocity(wheel_velocity_, 0.0f);
      }
      else {
         wheels_->SetLinearVelocity(0.0f, wheel_velocity_);
      }
   }
}

REGISTER_CONTROLLER(CFootBotRnBTest, "footbot_RnB_test_controller")
