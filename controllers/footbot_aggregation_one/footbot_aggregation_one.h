/*
 * AUTHOR: Malik Tremain
 *
 * This will be an implementation of the hop count aggregation alg as
 * described in Schmickl T, Möslinger C, Crailsheim K 2007 Collective perception in a robot swarm. Swarm
 * Robotics, (Springer, Berlin) pp.144–157
 * 
 * TODO: 
 *    -change robot LED colour based on forgetting state
 *    -Algorithmically draw two target zones onto the floor 
 *    -Output num robots in radius of target zone into CSV file
 *       along with other information (loop functions post step)
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
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <argos3/core/utility/math/rng.h>
#include <string>
#include <queue>
#include <limits>

/**
 * @class CRotationHandler
 * @brief This class is used to handle rotations.
 * This class handles static rotations for the footbot.
 * Static rotations:
 *    These rotations work by setting a desired turning angle (-180 <=> 180 degrees). The class will then calculate
 *    the number of "turning frames" needed to achieve the requested angle. The calculation done by finding the robot's
 *    current time to turn 180 degrees based on a known measurement(it takes 89 frames to turn 180 degrees when the wheel's
 *    velocity is {-5,5}). The calculated value is then stored in time_needed_180_. When a RotateTo() method is called
 *    the class will calcuate the number of frames needed to achieve that specific angle of rotation.
 *    The calculated result is stored in rot_frames_remaining_, holding a negative value to indicate anti-clockwise rotation and
 *    a positive value for clockwise rotation. Every time step the "ApproachZero()" method is called.
 *    This will change the value of rot_frames_remaining_ to get 1 frame closer to zero. If the value of rot_frames_remaining is 
 *    exactly zero then the HandleRotation() method in CFootBotAggregationOne will do nothing.
 */
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

/**
 * @class CHopCountManager
 * @brief This class is used to handle hop count and "forgetting".
 * This class is used to handle the robot's current hop count and "forgetting". The main method of
 * this class is Update(). If forgetting is disabled in the XML file then it will do nothing. If 
 * forgetting is enabled then it will decrease the value of forget_tp_counter_(a variable used to
 * track when to activate forgetting). The inital value of forget_tp_counter is forgetting_tp
 * which is the time period in frames of when to activate forget. When the value of forget_tp_counter
 * is zero then it will set the state of currently_forgetting_ to true. In the comming time steps the 
 * Update method will increase the value of current_hop_count_, if current_hop_count_ is less than 
 * max_hop_count_. When current_hop_count_'s value is the same as max_hop_count_ the Update method
 * will then set the state of currently_forgetting_ to false and it will reset the forget_tp_counter_
 * to the value of forgetting_tp_.
 */
class CHopCount
{
public:
   virtual bool Update()=0;
   // 'HCM' will represent CHopCountManager and 'PHC' will represent CPersitantHopCount
   virtual bool IsPersitant()const=0;

   virtual void ResetHopCount()=0;
   virtual void SetMaxHopCount(uint16_t max_hc)=0;
   virtual void SetCurrentHopCount(uint16_t hc)=0;
   virtual void SetForgettingEnabled(bool ForgettingAllowed)=0;
   virtual void SetForgettingTimePeriod(uint16_t forgetting_tp)=0;
   virtual uint16_t GetMaxHopCount()const=0;
   virtual uint16_t GetCurrentHopCount()const=0;
   virtual uint16_t GetForgettingTimePeriod()const=0;
   virtual bool GetForgettingEnabled()const=0;
   virtual bool GetCurrentlyForgetting()const=0;

};

class CHopCountManager : public CHopCount
{
public:
   CHopCountManager()=delete;
   explicit CHopCountManager(bool forgetting_enabled,uint16_t hop_count_max,uint16_t forgetting_tp);

   bool Update()override;
   void ResetHopCount()override;

   void SetMaxHopCount(uint16_t max_hc)override;
   void SetCurrentHopCount(uint16_t hc)override;
   void SetForgettingEnabled(bool ForgettingAllowed)override;
   void SetForgettingTimePeriod(uint16_t forgetting_tp)override;

   uint16_t GetMaxHopCount()const override;
   uint16_t GetCurrentHopCount()const override;
   uint16_t GetForgettingTimePeriod()const override;
   bool GetForgettingEnabled()const override;
   bool GetCurrentlyForgetting()const override;

   bool IsPersitant()const override{return false;}

private:
   uint16_t forgetting_tp_;
   uint16_t max_hop_count_;
   uint16_t forget_tp_counter_;
   uint16_t current_hop_count_;
   bool     forgetting_enabled_;
   bool     currently_forgetting_;

};

class CPersistantHopCount : public CHopCount
{
public:
   CPersistantHopCount()=delete;
   CPersistantHopCount(uint16_t hop_count, uint16_t max_hop_count):
      hop_count_(hop_count),max_hop_count_(max_hop_count){}

   bool Update()override{return false;}
   bool IsPersitant()const override{return true;}

   void ResetHopCount()override{}

   void SetMaxHopCount(uint16_t max_hc)override{max_hop_count_ = max_hc;}
   void SetCurrentHopCount(uint16_t hc)override{}
   void SetForgettingEnabled(bool ForgettingAllowed)override{}
   void SetForgettingTimePeriod(uint16_t forgetting_tp)override{}

   uint16_t GetMaxHopCount()const override{return max_hop_count_;}
   uint16_t GetCurrentHopCount()const override{return hop_count_;}
   uint16_t GetForgettingTimePeriod()const override{return 0;}
   bool GetForgettingEnabled()const override{return false;}
   bool GetCurrentlyForgetting()const override{return false;}
   
private:
   uint16_t hop_count_;
   uint16_t max_hop_count_;

};

class CDelayedTransmissionManager
{
public:
   CDelayedTransmissionManager()=delete;
   CDelayedTransmissionManager(argos::Real DelayedTransmittionProbability, uint64_t NumTStepsDelay, argos::CRandom::CRNG* rng_);
   
   uint16_t Update(argos::CRandom::CRNG* rng_,uint16_t transmission_data);
   bool GetDelayedState()const;

private:
   argos::CRandom::CRNG* rng_ptr_;

   bool enabled_;
   uint64_t time_step_delay_;
   std::queue<uint16_t> delayed_transmission_data_;

};

class CFootBotAggregationOne : public argos::CCI_Controller 
{
public:
   CFootBotAggregationOne();
   virtual void Init(argos::TConfigurationNode& t_node);
   virtual void ControlStep();
   virtual void Reset() {}
   virtual void Destroy()override {delete hop_count_;}
   std::string GetHopCount()const;
   std::string GetNumConnections()const;
   std::string GetWithinAreaState()const;
   std::string GetForgettingState()const;
   argos::CVector3 GetPosition()const;
   void ResetHopCount();

   void SetPersitantHopCount(uint16_t hop_count);
private:
   void RealTimeRotate(const argos::CRadians& avg_bearing);
   void TransmitHCData();
   bool HandleTurning();
   bool AvoidCollisions();
   bool HandleForgetting();
   bool ReadTransmissions();
   void MoveForward();

public:
   inline static argos::CRandom::CRNG*             rnd_gen{argos::CRandom::CreateRNG("argos")};
   inline static const argos::CRange<argos::Real>  rng_val_range{-180.0,180.0};

private:
   //robot components
   argos::CCI_DifferentialSteeringActuator*  wheels_;
   argos::CCI_FootBotProximitySensor*        proximity_sen_;
   argos::CCI_RangeAndBearingActuator*       rnb_actuator_;
   argos::CCI_RangeAndBearingSensor*         rnb_sensor_;
   argos::CCI_FootBotMotorGroundSensor*      ground_sensor_;
   argos::CCI_PositioningSensor*             position_sensor_;

   //The group below is to be read from the XML 
   argos::Real       wheel_velocity_;
   argos::Real       delta_;
   argos::CDegrees   alpha_;
   

   //Other internal variables
   CHopCount* hop_count_;
   CRotationHandler rotation_handler_;
   CDelayedTransmissionManager rnb_delay_handler_;
   bool stop_when_reach_target_area_;

   //Variable for tracking info to output to the LOG file
   uint64_t num_connections_;
   bool     within_secondary_area_;
   
};

#endif
