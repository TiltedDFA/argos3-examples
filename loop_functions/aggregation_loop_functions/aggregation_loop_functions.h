#ifndef AGGREGATION_LOOP_FUNCTIONS 
#define AGGREGATION_LOOP_FUNCTIONS

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>


class CAggregationLoopFunctions : public argos::CLoopFunctions 
{
public:
    CAggregationLoopFunctions();
    virtual ~CAggregationLoopFunctions();

    virtual void Init(argos::TConfigurationNode& t_node);
    virtual argos::CColor GetFloorColor(const argos::CVector2& c_position_on_plane);
    virtual void Reset();
    virtual void Destroy();
    virtual void PreStep();
private:
    std::string some_string_;
    std::ofstream file_stream_;
};






#endif //#ifndef AGGREGATION_LOOP_FUNCTIONS 