/*
 * AUTHOR: Malik Tremain <cpinciro@ulb.ac.be>
 *
 * This will be an implamentation of the hop count aggregation alg as
 * described in Schmickl T, Möslinger C, Crailsheim K 2007 Collective perception in a robot swarm. Swarm
 * Robotics, (Springer, Berlin) pp.144–157
 */

#ifndef FOOTBOT_AGGREGATION_ONE_H
#define FOOTBOT_AGGREGATION_ONE_H

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of the foot-bot proximity sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>


/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CFootBotAggregationOne : public argos::CCI_Controller {
public:
   CFootBotAggregationOne();
   /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML
    * file in the <controllers><footbot_basictest_controller> section.
    */
   virtual void Init(argos::TConfigurationNode& t_node);

   /*
    * This function is called once every time step.
    * The length of the time step is set in the XML file.
    */
   virtual void ControlStep();

   /*
    * This function resets the controller to its state right after the
    * Init().
    * It is called when you press the reset button in the GUI.
    * In this example controller there is no need for resetting anything,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
   virtual void Reset() {}

   /*
    * Called to cleanup what done by Init() when the experiment finishes.
    * In this example controller there is no need for clean anything up,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
   virtual void Destroy() {}
public:
   const inline static uint16_t HOPCOUNT_MAX{100u};
private:

   /* Pointer to the differential steering actuator */
   argos::CCI_DifferentialSteeringActuator* m_pcWheels;
   /* Pointer to the foot-bot proximity sensor */
   argos::CCI_FootBotProximitySensor* m_pcProximity;

   /*
    * The following variables are used as parameters for the
    * algorithm. You can set their value in the <parameters> section
    * of the XML configuration file, under the
    * <controllers><footbot_basictest_controller> section.
    */

   /* Maximum tolerance for the angle between
    * the robot heading direction and
    * the closest obstacle detected. */
   argos::CDegrees m_cAlpha;
   /* Maximum tolerance for the proximity reading between
    * the robot and the closest obstacle.
    * The proximity reading is 0 when nothing is detected
    * and grows exponentially to 1 when the obstacle is
    * touching the robot.
    */
   argos::Real m_fDelta;
   /* Wheel speed. */
   argos::Real m_fWheelVelocity;
   /* Angle tolerance range to go straight.
    * It is set to [-alpha,alpha]. */
   argos::CRange<argos::CRadians> m_cGoStraightAngleRange;

};

#endif
