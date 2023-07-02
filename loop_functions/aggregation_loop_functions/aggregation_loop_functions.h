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

    virtual void Init(TConfigurationNode& t_node);
    virtual void Reset();
    virtual void Destroy();
    virtual void PreStep();
private:
    std::string some_string_;
    std::ofstream file_stream_;
}






#endif //#ifndef AGGREGATION_LOOP_FUNCTIONS 