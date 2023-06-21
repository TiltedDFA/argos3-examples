/*
 * This example shows how to define custom distributions to place the robots.
 */

#include <argos3/core/simulator/loop_functions.h>

using argos::CARGoSException;

class CAggregationLoopFunctions : public argos::CLoopFunctions 
{

public:

   CAggregationLoopFunctions();
   virtual ~CAggregationLoopFunctions();

   virtual void Init(argos::TConfigurationNode& t_tree);

private:

   /*
    *
    */
   void PlaceLine(const argos::CVector2& c_center,
                  argos::UInt32 un_robots,
                  argos::Real f_distance,
                  argos::UInt32 un_id_start);

   void PlaceCluster(const argos::CVector2& c_center,
                     argos::UInt32 un_robots,
                     argos::Real f_density,
                     argos::UInt32 un_id_start);

   void PlaceScaleFree(const argos::CVector2& c_center,
                       argos::UInt32 un_robots,
                       argos::Real f_range,
                       argos::UInt32 un_id_start);
   
private:

   enum ETopology {
      TOPOLOGY_LINE,
      TOPOLOGY_CLUSTER,
      TOPOLOGY_SCALEFREE
   };

};

