#include "Entry.h"
#include "Tasks.h"
#include "utils.h"
#include "heuristics.h"

// The initialize function will be called by competition system at the preprocessing stage.
// Implement the initialize functions of the planner and scheduler to load or compute auxiliary data.
// Note that, this function runs untill preprocess_time_limit (in milliseconds) is reached.
// This is an offline step, after it completes then evaluation begins.
void Entry::initialize(int preprocess_time_limit)
{
    env->past_waitings.resize(env->map.size()*5,{0,0});
    env->accu_waitings.resize(env->num_of_agents,0);
    scheduler->initialize(preprocess_time_limit);
    planner->initialize(preprocess_time_limit);
}

//The compute function will be called by competition system on each timestep.
//It computes:
//  1. a schedule that specifies which agent complete which task.
//  2. a next action that specifies how each agent should move in the next timestep.
//NB: the parameter time_limit is specified in milliseconds.
void Entry::compute(int time_limit, std::vector<Action> & plan, std::vector<int> & proposed_schedule)
{

    if (time_in_commit_window == 0)
    {
        //  first call task schedule

        scheduler->set_flow(planner->get_flow());
        scheduler->plan(time_limit,proposed_schedule);

        //then update the first unfinished errand/location of tasks for planner reference
        update_goal_locations(proposed_schedule);
        
        //call the planner to compute the actions
        planner->plan(time_limit,plan);
    }
    else
    {
        //then update the first unfinished errand/location of tasks for planner reference
        update_goal_locations(proposed_schedule);
        //then call planner with only pibt
        planner->plan_pibt(time_limit,plan);
    }

    if (time_in_commit_window == commit_window-1)
    {
        time_in_commit_window = 0;
    }
    else
    {
        time_in_commit_window++;
    }

}

// Set the next goal locations for each agent based on the proposed schedule
void Entry::update_goal_locations(std::vector<int> & proposed_schedule)
{
    // record the proposed schedule so that we can tell the planner
    env->curr_task_schedule = proposed_schedule;

    // The first unfinished errand/location of each task is the next goal for the assigned agent.
    for (size_t i = 0; i < proposed_schedule.size(); i++)
    {
        env->goal_locations[i].clear();
        int t_id = proposed_schedule[i];
        if (t_id == -1)
            continue;

        int i_loc = env->task_pool[t_id].idx_next_loc;
        while (i_loc < env->task_pool[t_id].locations.size())
        {
            env->goal_locations[i].push_back({env->task_pool[t_id].locations.at(i_loc), env->task_pool[t_id].t_revealed});
            i_loc++;
        }
    }

    return;
}