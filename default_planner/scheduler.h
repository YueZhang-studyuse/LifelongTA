#ifndef SCHEDULER
#define SCHEDULER

#include "Types.h"
#include "SharedEnv.h"
#include "heuristics.h"
#include <random>

namespace DefaultPlanner{

void schedule_initialize(int preprocess_time_limit, SharedEnvironment* env);

void schedule_plan(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env, std::vector<Int4> background_flow);
void schedule_plan_raw(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env);

}

#endif