#include "aggregation_loop_function.h"

#include <argos3/plugins/simulator/entities/box_entity.h>
#include <argos3/plugins/simulator/entities/cylinder_entity.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

#include <sstream>
#include <list>

/****************************************/
/****************************************/

static const argos::Real         FB_RADIUS        = 0.085036758f;
static const argos::Real         FB_AREA          = ARGOS_PI * argos::Square(0.085036758f);
static const std::string         FB_CONTROLLER    = "ffc";
static const argos::UInt32       MAX_PLACE_TRIALS = 20;
static const argos::UInt32       MAX_ROBOT_TRIALS = 20;

/****************************************/
/****************************************/

CAggregationLoopFunctions::CAggregationLoopFunctions() {}

/****************************************/
/****************************************/

CAggregationLoopFunctions::~CAggregationLoopFunctions() {}

/****************************************/
/****************************************/

void CAggregationLoopFunctions::Init(argos::TConfigurationNode& t_tree) 
{
   try 
   {
      /*
       * Parse the configuration file
       */
      argos::UInt32 unPlacedRobots = 0;
      /* Go through the nodes */
      argos::TConfigurationNodeIterator itDistr;
      for(itDistr = itDistr.begin(&t_tree);
          itDistr != itDistr.end();
          ++itDistr) 
      {
         /* Make sure a known distribution was passed */
         if(itDistr->Value() != "line" &&
            itDistr->Value() != "cluster" &&
            itDistr->Value() != "scalefree") 
         {
            argos::THROW_ARGOSEXCEPTION("Unknown topology \"" << itDistr->Value() << "\"");
            
         }
         /* Get current node */
         argos::TConfigurationNode& tDistr = *itDistr;
         /* Parse common attributes */
         /* Distribution center */
         argos::CVector2 cCenter;
         argos::GetNodeAttribute(tDistr, "center", cCenter);
         /* Number of robots to place */
         argos::UInt32 unRobots;
         argos::GetNodeAttribute(tDistr, "robot_num", unRobots);
         /* Parse distribution-specific attributes and place robots */
         if(itDistr->Value() == "line") 
         {
            /* Distance between the robots */
            argos::Real fDistance;
            argos::GetNodeAttribute(tDistr, "robot_distance", fDistance);
            /* Place robots */
            PlaceLine(cCenter, unRobots, fDistance, unPlacedRobots);
         }
         else if(itDistr->Value() == "cluster") 
         {
            /* Density of the robots */
            argos::Real fDensity;
            argos::GetNodeAttribute(tDistr, "robot_density", fDensity);
            /* Place robots */
            PlaceCluster(cCenter, unRobots, fDensity, unPlacedRobots);
         }
         else /* (itDistr->Value() == "scalefree") */ 
         {
            /* Range around each robot */
            argos::Real fRange;
            argos::GetNodeAttribute(tDistr, "robot_range", fRange);
            /* Place robots */
            PlaceScaleFree(cCenter, unRobots, fRange, unPlacedRobots);
         }
         /* Update robot count */
         unPlacedRobots += unRobots;
      }
   }
   catch(argos::CARGoSException& ex) 
   {
      argos::THROW_ARGOSEXCEPTION_NESTED("Error initializing the loop functions", ex);
   }
}

/****************************************/
/****************************************/

void CAggregationLoopFunctions::PlaceLine(const argos::CVector2& c_center,
                                                  argos::UInt32 un_robots,
                                                  argos::Real f_distance,
                                                  argos::UInt32 un_id_start) 
{
   try 
   {
      argos::CFootBotEntity* pcFB;
      std::ostringstream cFBId;
      /* For each robot */
      for(size_t i = 0; i < un_robots; ++i) 
      {
         /* Make the id */
         cFBId.str("");
         cFBId << "fb" << (i + un_id_start);
         /* Create the robot in the origin and add it to ARGoS space */
         pcFB = new argos::CFootBotEntity
         (
            cFBId.str(),
            FB_CONTROLLER,
            argos::CVector3(i + c_center.GetX(), i + c_center.GetY(), 0)
         );
         AddEntity(*pcFB);
      }
   }
   catch(argos::CARGoSException& ex) 
   {
      THROW_ARGOSEXCEPTION_NESTED("While placing robots in a line", ex);
   }
}

/****************************************/
/****************************************/

void CAggregationLoopFunctions::PlaceCluster(const argos::CVector2& c_center,
                                                     argos::UInt32 un_robots,
                                                     argos::Real f_density,
                                                     argos::UInt32 un_id_start) 
{
   try 
   {
      /* Calculate side of the region in which the robots are scattered */
      argos::Real fHalfSide = Sqrt((FB_AREA * un_robots) / f_density) / 2.0f;
      argos::CRange<argos::Real> cAreaRange(-fHalfSide, fHalfSide);
      /* Place robots */
      argos::UInt32 unTrials;
      argos::CFootBotEntity* pcFB;
      std::ostringstream cFBId;
      argos::CVector3 cFBPos;
      argos::CQuaternion cFBRot;
      /* Create a RNG (it is automatically disposed of by ARGoS) */
      argos::CRandom::CRNG* pcRNG = argos::CRandom::CreateRNG("argos");
      /* For each robot */
      for(size_t i = 0; i < un_robots; ++i) 
      {
         /* Make the id */
         cFBId.str("");
         cFBId << "fb" << (i + un_id_start);
         /* Create the robot in the origin and add it to ARGoS space */
         pcFB = new argos::CFootBotEntity
         (
            cFBId.str(),
            FB_CONTROLLER
         );
         AddEntity(*pcFB);
         /* Try to place it in the arena */
         unTrials = 0;
         bool bDone;
         do 
         {
            /* Choose a random position */
            ++unTrials;
            cFBPos.Set(pcRNG->Uniform(cAreaRange) + c_center.GetX(),
                       pcRNG->Uniform(cAreaRange) + c_center.GetY(),
                       0.0f);
            cFBRot.FromAngleAxis(pcRNG->Uniform(argos::CRadians::UNSIGNED_RANGE),
                                 argos::CVector3::Z);
            bDone = MoveEntity(pcFB->GetEmbodiedEntity(), cFBPos, cFBRot);
         } while(!bDone && unTrials <= MAX_PLACE_TRIALS);
         if(!bDone) {
            argos::THROW_ARGOSEXCEPTION("Can't place " << cFBId.str());
         }
      }
   }
   catch(argos::CARGoSException& ex) 
   {
      argos::THROW_ARGOSEXCEPTION_NESTED("While placing robots in a cluster", ex);
   }
}

/****************************************/
/****************************************/

struct SFData 
{
   
   struct SEntry 
   {
      argos::UInt32 Conns;
      argos::CVector3& Pos;
      SEntry(argos::UInt32 un_conns,
             argos::CVector3& c_pos) :
         Conns(un_conns),
         Pos(c_pos) {}
   };

   SFData() :
      TotConns(0),
      RNG(argos::CRandom::CreateRNG("argos")) {}

   ~SFData() 
   {
      while(!Data.empty()) {
         delete Data.front();
         Data.pop_front();
      }
   }

   void Insert(argos::CFootBotEntity& c_entity) 
   {
      /* Two connections to be added: entity <-> pivot */
      TotConns += 2;
      Data.push_back(
         new SEntry(1,
                    c_entity.GetEmbodiedEntity().
                    GetOriginAnchor().Position));
   }

   SEntry* Pick() 
   {
      if(Data.size() > 1) {
         /* More than 1 element stored, look for the pivot */
         argos::UInt32 x = RNG->Uniform(argos::CRange<argos::UInt32>(0, TotConns));
         argos::UInt32 unSum = 0;
         std::list<SEntry*>::iterator it = Data.begin();
         while(it != Data.end() && unSum <= x) {
            unSum += (*it)->Conns;
            ++it;
         }
         if(it != Data.end()) {
            --it;
            return *it;
         }
         else {
            return Data.back();
         }
      }
      else if(Data.size() == 1) {
         /* One element stored, just return that one */
         return Data.front();
      }
      else argos::THROW_ARGOSEXCEPTION("SFData::Pick(): empty structure");
   }

private:

   std::list<SEntry*> Data;
   argos::UInt32 TotConns;
   argos::CRandom::CRNG* RNG;
   
};

static argos::Real GenerateCoordinate(argos::CRandom::CRNG* pc_rng,
                               argos::Real f_half_range) 
{
   argos::Real v = pc_rng->Uniform(argos::CRange<argos::Real>(-f_half_range, f_half_range));
   if(v > 0.0) v += f_half_range;
   else v -= f_half_range;
   return v;
}

void CAggregationLoopFunctions::PlaceScaleFree(const argos::CVector2& c_center,
                                                       argos::UInt32 un_robots,
                                                       argos::Real f_range,
                                                       argos::UInt32 un_id_start) 
{
   try 
   {
      /* Data structures for the insertion of new robots */
      argos::UInt32 unRobotTrials, unPlaceTrials, unPivot;
      argos::CFootBotEntity* pcFB;
      std::ostringstream cFBId;
      argos::CVector3 cFBPos;
      argos::CQuaternion cFBRot;
      SFData sData;
      SFData::SEntry* psPivot;
      bool bDone;
      argos::Real fHalfRange = f_range * 0.5;
      /* Create a RNG (it is automatically disposed of by ARGoS) */
      argos::CRandom::CRNG* pcRNG = argos::CRandom::CreateRNG("argos");
      /* Add first robot in the origin */
      /* Create the robot in the origin and add it to ARGoS space */
      cFBId << "fb" << un_id_start;
      pcFB = new argos::CFootBotEntity(
         cFBId.str(),
         FB_CONTROLLER);
      AddEntity(*pcFB);
      MoveEntity(pcFB->GetEmbodiedEntity(),
                 argos::CVector3(c_center.GetX(),
                          c_center.GetY(),
                          0.0),
                 argos::CQuaternion());
      sData.Insert(*pcFB);
      /* Add other robots */
      for(argos::UInt32 i = 1; i < un_robots; ++i) {
         /* Make the id */
         cFBId.str("");
         cFBId << "fb" << (i + un_id_start);
         /* Create the robot in the origin and add it to ARGoS space */
         pcFB = new argos::CFootBotEntity(
            cFBId.str(),
            FB_CONTROLLER);
         AddEntity(*pcFB);
         /* Retry choosing a pivot until you get a position or have an error */
         unRobotTrials = 0;
         do 
         {
            /* Choose a pivot */
            ++unRobotTrials;
            psPivot = sData.Pick();
            cFBRot.FromAngleAxis(pcRNG->Uniform(argos::CRadians::UNSIGNED_RANGE),
                                 argos::CVector3::Z);
            /* Try placing a robot close to this pivot */
            unPlaceTrials = 0;
            do 
            {
               ++unPlaceTrials;
               /* Pick a position within the range of the pivot */
               cFBPos.Set(GenerateCoordinate(pcRNG, fHalfRange) + c_center.GetX(),
                          GenerateCoordinate(pcRNG, fHalfRange) + c_center.GetY(),
                          0.0f);
               cFBPos += psPivot->Pos;
               /* Try placing the robot */
               bDone = MoveEntity(pcFB->GetEmbodiedEntity(), cFBPos, cFBRot);
            }
            while(!bDone && unPlaceTrials <= MAX_PLACE_TRIALS);

         } while(!bDone && unRobotTrials <= MAX_ROBOT_TRIALS);
         /* Was the robot placed successfully? */
         if(!bDone) 
         {
            argos::THROW_ARGOSEXCEPTION("Can't place " << cFBId.str());
         }
         /* Yes, insert it in the data structure */
         ++psPivot->Conns;
         sData.Insert(*pcFB);
      }
   }
   catch(argos::CARGoSException& ex) 
   {
      argos::THROW_ARGOSEXCEPTION_NESTED("While placing robots in a scale-free distribution", ex);
   }
}
REGISTER_LOOP_FUNCTIONS(CAggregationLoopFunctions, "aggregation_loop_function");
