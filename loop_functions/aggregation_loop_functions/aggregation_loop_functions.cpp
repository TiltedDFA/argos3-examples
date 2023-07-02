#include "aggregation_loop_functions.h"

CAggregationLoopFunctions::CAggregationLoopFunctions(){}

void CAggregationLoopFunctions::Init(argos::TConfigurationNode& t_node)
{
    argos::TConfigurationNode& aggregation_node = argos::GetNode(t_node, "aggregation");
    argos::GetNodeAttribute(aggregation_node,"file_name", some_string_);
    file_stream_.open(some_string_,std::ios_base::trunc | std::ios_base::out);
    file_stream_ << "This is test" << std::endl;
}