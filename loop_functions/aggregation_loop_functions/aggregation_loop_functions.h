#ifndef AGGREGATION_LOOP_FUNCTIONS_H
#define AGGREGATION_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>
#include "controllers/footbot_aggregation_one/footbot_aggregation_one.h"
#include <random>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <cmath>

//target areas will be circles
class TargetArea
{
public:
    TargetArea()=delete;
    explicit TargetArea(const argos::CVector2& area_center, argos::Real area_radius, argos::Real secondary_offset);
    bool PointWithinTargetArea(const argos::CVector2& point)const;
    bool PointWithinSecondaryArea(const argos::CVector2& point)const;
private:
    argos::CVector2 area_center_;
    argos::Real area_radius_;
    argos::Real secondary_offset_;
};
class CAggregationLoopFunctions : public argos::CLoopFunctions 
{
public:
    CAggregationLoopFunctions();
    virtual ~CAggregationLoopFunctions(){};

    virtual void Init(argos::TConfigurationNode& t_node);
    virtual argos::CColor GetFloorColor(const argos::CVector2& c_position_on_plane);
    virtual void Reset();
    virtual void Destroy();
    virtual void PreStep();
public:
    inline static argos::CRandom::CRNG* rnd_gen{argos::CRandom::CreateRNG("argos")};
private:
    uint64_t GetZone(argos::CVector3 position);
private:
    std::string file_name_;
    std::ofstream file_stream_;
    std::vector<TargetArea> target_areas_;
    std::vector<argos::CFootBotEntity*> bot_entities_;
    bool file_out_in_csv_;
};






#endif //#ifndef AGGREGATION_LOOP_FUNCTIONS_H 