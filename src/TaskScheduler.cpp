#include "TaskScheduler.h"
#include "const.h"

/**
 * Initializes the task scheduler with a given time limit for preprocessing.
 * 
 * This function prepares the task scheduler by allocating up to half of the given preprocessing time limit 
 * and adjust for a specified tolerance to account for potential timing errors. 
 * It ensures that initialization does not exceed the allocated time.
 * 
 * @param preprocess_time_limit The total time limit allocated for preprocessing (in milliseconds).
 *
 */
void TaskScheduler::initialize(int preprocess_time_limit)
{
    //give at most half of the entry time_limit to scheduler;
    //-SCHEDULER_TIMELIMIT_TOLERANCE for timing error tolerance
    int limit = preprocess_time_limit/2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    DefaultPlanner::schedule_initialize(limit, env);    
}

/**
 * Plans a task schedule within a specified time limit.
 * 
 * This function schedules tasks by calling shedule_plan function in default planner with half of the given time limit,
 * adjusted for timing error tolerance. The planned schedule is output to the provided schedule vector.
 * 
 * @param time_limit The total time limit allocated for scheduling (in milliseconds).
 * @param proposed_schedule A reference to a vector that will be populated with the proposed schedule (next task id for each agent).
 */

void TaskScheduler::plan(int time_limit, std::vector<int> & proposed_schedule)
{
    //give at most half of the entry time_limit to scheduler;
    //-SCHEDULER_TIMELIMIT_TOLERANCE for timing error tolerance
    int limit = time_limit/2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;


    if (!use_traffic)
    {
        DefaultPlanner::schedule_plan_flow(limit, proposed_schedule, env, background_flow, use_traffic, new_only);
    }
    else
    {
        if (solver == 1)
        {
            DefaultPlanner::schedule_plan_flow(limit, proposed_schedule, env, background_flow, use_traffic, new_only);
        }
        else if (solver == 2)
        {
            DefaultPlanner::schedule_plan_flow_hist(limit, proposed_schedule, env, env->past_waitings, new_only);
        }
        else if (solver == 3)
        {
            DefaultPlanner::schedule_plan_matching(limit, proposed_schedule, env, background_flow, use_traffic, new_only, max_matching_edges);
        }
        else
        {
            std::cerr << "Invalid solver type. Please choose either 1 (matching) or 2 (flow)." << std::endl;
            exit(1);
        }
    }
}

void TaskScheduler::set_flow(std::vector<DefaultPlanner::Int4> flow)
{
    background_flow = flow;
}

void TaskScheduler::set_use_traffic(bool use_traffic)
{
    this->use_traffic = use_traffic;
}
void TaskScheduler::set_new_only(bool new_only)
{
    this->new_only = new_only;
}
void TaskScheduler::set_solver(int solver)
{
    this->solver = solver;
}
void TaskScheduler::set_max_matching_edges(int max_matching_edges)
{
    this->max_matching_edges = max_matching_edges;
}
