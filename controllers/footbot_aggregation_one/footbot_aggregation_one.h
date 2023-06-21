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

class CFootBotAggregationOne : public argos::CCI_Controller {
public:
   CFootBotAggregationOne();
   virtual void Init(argos::TConfigurationNode& t_node);
   virtual void ControlStep();
   virtual void Reset() {}
   virtual void Destroy() {}

public:
   const inline static uint16_t HOPCOUNT_MAX{100u};
   const inline static argos::CDegrees ALPHA{7.5f};
   const inline static argos::Real DELTA{0.1f};

private:
   argos::CCI_DifferentialSteeringActuator* wheels_;
   argos::CCI_FootBotProximitySensor* proximity_sen_;
   argos::Real wheel_velocity_;
   argos::CRange<argos::CRadians> navigation_threshold_;
};

#endif
