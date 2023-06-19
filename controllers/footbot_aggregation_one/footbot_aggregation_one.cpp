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
   m_pcWheels(NULL),
   m_pcProximity(NULL),
   m_cAlpha(10.0f),
   m_fDelta(0.5f),
   m_fWheelVelocity(2.5f),
   m_cGoStraightAngleRange(-argos::ToRadians(m_cAlpha),
                           argos::ToRadians(m_cAlpha)) {}

/****************************************/
/****************************************/

void CFootBotAggregationOne::Init(argos::TConfigurationNode& t_node) {
   /*
    * Get sensor/actuator handles
    *
    * The passed string (ex. "differential_steering") corresponds to the
    * XML tag of the device whose handle we want to have. For a list of
    * allowed values, type at the command prompt:
    *
    * $ argos3 -q actuators
    *
    * to have a list of all the possible actuators, or
    *
    * $ argos3 -q sensors
    *
    * to have a list of all the possible sensors.
    *
    * NOTE: ARGoS creates and initializes actuators and sensors
    * internally, on the basis of the lists provided the configuration
    * file at the <controllers><footbot_basictest><actuators> and
    * <controllers><footbot_basictest><sensors> sections. If you forgot to
    * list a device in the XML and then you request it here, an error
    * occurs.
    */
   m_pcWheels    = GetActuator<argos::CCI_DifferentialSteeringActuator>("differential_steering");
   m_pcProximity = GetSensor  <argos::CCI_FootBotProximitySensor      >("footbot_proximity"    );
   /*
    * Parse the configuration file
    *
    * The user defines this part. Here, the algorithm accepts three
    * parameters and it's nice to put them in the config file so we don't
    * have to recompile if we want to try other settings.
    */
   argos::GetNodeAttributeOrDefault(t_node, "alpha", m_cAlpha, m_cAlpha);
   m_cGoStraightAngleRange.Set(-ToRadians(m_cAlpha), ToRadians(m_cAlpha));
   argos::GetNodeAttributeOrDefault(t_node, "delta", m_fDelta, m_fDelta);
   argos::GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);
}

/****************************************/
/****************************************/

void CFootBotAggregationOne::ControlStep() {
   const argos::CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
   /* Sum them together */
   argos::CVector2 cAccumulator;
   for(size_t i = 0; i < tProxReads.size(); ++i) {
      if(i < 3) std::cout << "S" << (i+1) << " = V:" << tProxReads[i].Value << ", A:" << tProxReads[i].Angle.GetValue() << std::endl;
      cAccumulator += argos::CVector2(tProxReads[i].Value, tProxReads[i].Angle);
   }
   //finds average reading - divides both the angles and the value stored
   cAccumulator /= tProxReads.size();
   /* If the angle of the vector is small enough and the closest obstacle
    * is far enough, continue going straight, otherwise curve a little
    */
   std::cout << "Average Value is " << cAccumulator.Length() << std::endl;
   argos::CRadians cAngle = cAccumulator.Angle();
   m_pcWheels->SetLinearVelocity(m_fWheelVelocity,m_fWheelVelocity);
}

/****************************************/
/****************************************/
/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the configuration file to refer to this
 * controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CFootBotAggregationOne, "footbot_aggregation_one")
