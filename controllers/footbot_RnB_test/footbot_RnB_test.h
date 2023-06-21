#ifndef FOOTBOT_RNB_TEST
#define FOOTBOT_RNB_TEST


/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of the foot-bot proximity sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>

using namespace argos;

class CFootBotRnBTest : public CCI_Controller {

public:

   CFootBotRnBTest();
   virtual ~CFootBotRnBTest() {}

   virtual void Init(TConfigurationNode& t_node);
   virtual void ControlStep();
   virtual void Reset() {}
   virtual void Destroy() {}

private:

   CCI_DifferentialSteeringActuator* wheels_;
   CCI_FootBotProximitySensor* proximity_sen_;
   CCI_RangeAndBearingActuator* rnb_actuator_;
   CCI_RangeAndBearingSensor* rnb_sensor_;
   
   CDegrees alpha_;
   Real delta_;
   Real wheel_velocity_;
   CRange<CRadians> turning_thresholds_;

};

#endif
