/*
 * AUTHOR: Malik Tremain
 *
 * This will be an implamentation of the hop count aggregation alg as
 * described in Schmickl T, Möslinger C, Crailsheim K 2007 Collective perception in a robot swarm. Swarm
 * Robotics, (Springer, Berlin) pp.144–157
 * 
 * TODO: 
 *    -change robot LED colour based on forgetting state
 */

#ifndef FOOTBOT_AGGREGATION_ONE_H
#define FOOTBOT_AGGREGATION_ONE_H

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_motor_ground_sensor.h>
#include <random>
#include <string>

class CRotationHandler
{
public:
   CRotationHandler()=delete;
   CRotationHandler(argos::Real robo_wheel_vel);

   void FindTimeNeededFor180(argos::Real robo_wheel_vel);
   void RotateTo(argos::Real desired_turning_angle);
   void ApproachZero();
   void NonZeroRotateTo(argos::Real desired_turning_angle);

   int8_t GetRemainingRotationTime();

private:
   int8_t rot_frames_remaining_;
   uint8_t time_needed_180_;
};

class CHopCountManager 
{
public:
   CHopCountManager()=delete;
   explicit CHopCountManager(bool forgetting_enabled,uint16_t hop_count_max,uint16_t forgetting_tp);

   bool update();

   void SetMaxHopCount(uint16_t max_hc);
   void SetCurrentHopCount(uint16_t hc);
   void SetForgettingEnabled(bool ForgettingAllowed);
   void SetForgettingTimePeriod(uint16_t forgetting_tp);

   uint16_t GetMaxHopCount()const;
   uint16_t GetCurrentHopCount()const;
   uint16_t GetForgettingTimePeriod()const;
   bool GetForgettingEnabled()const;

private:
   bool forgetting_enabled_;
   uint16_t forgetting_tp_;
   uint16_t max_hop_count_;
   uint16_t forget_tp_counter_;
   uint16_t current_hop_count_;
   bool currently_forgetting_;
};
class CFootBotAggregationOne : public argos::CCI_Controller 
{
public:
   CFootBotAggregationOne();
   virtual void Init(argos::TConfigurationNode& t_node);
   virtual void ControlStep();
   virtual void Reset() {}
   virtual void Destroy() {}
   std::string GetHopCount();
   
private:
   void RealTimeRotate(const argos::CRadians& avg_bearing);
   void TransmitHCData();
   bool HandleTurning();
   bool AvoidCollisions();
   bool HandleForgetting();
   bool HandleTargetArea();
   bool ReadTransmitions();
   void MoveForward();

public:
   //threadsafe random number gen
   inline static std::mt19937 rng_{std::random_device{}()};
   //will be used to generate a random direction for the robots to switch to
   inline static std::uniform_int_distribution<int16_t> rng_angle_{-180,180};

private:
   //robot components
   argos::CCI_DifferentialSteeringActuator*  wheels_;
   argos::CCI_FootBotProximitySensor*        proximity_sen_;
   argos::CCI_RangeAndBearingActuator*       rnb_actuator_;
   argos::CCI_RangeAndBearingSensor*         rnb_sensor_;
   argos::CCI_FootBotMotorGroundSensor*      ground_sensor_;

   //The group below is to be read from the XML 
   argos::Real       wheel_velocity_;
   argos::Real       delta_;
   argos::CDegrees   alpha_;
   

   //Other internal variables
   CHopCountManager hop_count_;
   CRotationHandler rotation_handler_;
};

#endif
