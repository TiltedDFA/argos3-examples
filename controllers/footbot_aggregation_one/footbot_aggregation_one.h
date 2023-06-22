/*
 * AUTHOR: Malik Tremain
 *
 * This will be an implamentation of the hop count aggregation alg as
 * described in Schmickl T, Möslinger C, Crailsheim K 2007 Collective perception in a robot swarm. Swarm
 * Robotics, (Springer, Berlin) pp.144–157
 */

#ifndef FOOTBOT_AGGREGATION_ONE_H
#define FOOTBOT_AGGREGATION_ONE_H

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>

enum class aggregation_state:uint8_t 
{
   MoveTowardsTarget,
   MoveRandomly,
   Halt,
   Forgetting
};

class CFootBotAggregationOne : public argos::CCI_Controller {
public:
   CFootBotAggregationOne();
   virtual void Init(argos::TConfigurationNode& t_node);
   virtual void ControlStep();
   virtual void Reset() {}
   virtual void Destroy() {}

private:
   //robot components
   argos::CCI_DifferentialSteeringActuator* wheels_;
   argos::CCI_FootBotProximitySensor* proximity_sen_;
   argos::CCI_RangeAndBearingActuator* rnb_actuator_;
   argos::CCI_RangeAndBearingSensor* rnb_sensor_;

   //The group below is to be read from the XML 
   argos::Real wheel_velocity_;
   argos::Real delta_;
   argos::Real alpha_;
   uint16_t hopcount_max;
   bool forgetting_allowed_;
   uint16_t forgetting_time_period_;

   //Other variables
   uint16_t current_hopcount_;
   argos::CRange<argos::CRadians> navigation_threshold_;
};

#endif
