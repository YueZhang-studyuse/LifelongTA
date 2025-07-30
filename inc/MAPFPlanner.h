#pragma once
#include <ctime>
#include "SharedEnv.h"
#include "ActionModel.h"
#include "planner.h"


class MAPFPlanner
{
public:
    SharedEnvironment* env;

	MAPFPlanner(SharedEnvironment* env): env(env){};
    MAPFPlanner(){env = new SharedEnvironment();};
	virtual ~MAPFPlanner(){delete env;};


    virtual void initialize(int preprocess_time_limit);

    // return next states for all agents
    virtual void plan(int time_limit, std::vector<Action> & plan);

    std::vector<DefaultPlanner::Double4> get_flow();
    void plan_pibt(int time_limit,vector<Action> & actions); 

};
