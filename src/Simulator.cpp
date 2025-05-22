#include "Simulator.h"
#include "nlohmann/json.hpp"
using json = nlohmann::ordered_json;

vector<State> Simulator::move(vector<Action>& actions) 
{
    //bool move_valid = true;
    for (int k = 0; k < num_of_agents; k++)
    {
        //move_valid = false;
        all_valid = false;
        if (k >= actions.size()){
            planner_movements[k].push_back(Action::NA);
        }
        else
        {
            planner_movements[k].push_back(actions[k]);
        }
    }

    if (!model->is_valid(curr_states, actions, timestep))
    {
        //move_valid = false;
        all_valid = false;
        actions = std::vector<Action>(num_of_agents, Action::WA);
    }

    curr_states = model->result_states(curr_states, actions);
    timestep++;

    for (int k = 0; k < num_of_agents; k++){
        paths[k].push_back(curr_states[k]);
        actual_movements[k].push_back(actions[k]);
    }
    //return move_valid;
    return curr_states;
}

void Simulator::sync_shared_env(SharedEnvironment* env) 
{
    for (int i = 0; i < planner_movements.size(); i++)
    {
        if (planner_movements[i].back() == Action::NA)
            continue;
        if (planner_movements[i].back() == Action::WA)
        {
            env->accu_waitings[i]++;
        }
        else
        {
            int time = env->accu_waitings[i]+1;
            env->past_waitings[curr_states[i].location*5].first += time;
            env->past_waitings[curr_states[i].location*5].second++;
            //north, east, south, west in order
            int counter = 0;
            if (planner_movements[i].back() == Action::N)
                counter = 1;
            else if (planner_movements[i].back() == Action::E)
                counter = 2;
            else if (planner_movements[i].back() == Action::S)
                counter = 3;
            else if (planner_movements[i].back() == Action::WE)
                counter = 4;
            env->past_waitings[curr_states[i].location*5+counter].first += time;
            env->past_waitings[curr_states[i].location*5+counter].second++;
            env->accu_waitings[i]=0;
        }
    }
    env->curr_states = curr_states;
    env->curr_timestep = timestep;
}

json Simulator::actual_path_to_json() const
{
    // Save actual paths
    json apaths = json::array();
    for (int i = 0; i < num_of_agents; i++)
    {
        std::string path;
        bool first = true;
        for (const auto action : actual_movements[i])
        {
            if (!first)
            {
                path+= ",";
            }
            else
            {
                first = false;
            }
            // path+=action;

            // if (action == Action::FW)
            // {
            //     path+="F";
            // }
            // else if (action == Action::CR)
            // {
            //     path+="R";
            // } 
            // else if (action == Action::CCR)
            // {
            //     path+="C";
            // }
            // else if (action == Action::NA)
            // {
            //     path+="T";
            // }
            // else
            // {
            //     path+="W";
            // }
            if (action == Action::N)
            {
                path+="NO";
            }
            else if (action == Action::E)
            {
                path+="EA";
            } 
            else if (action == Action::S)
            {
                path+="SO";
            }
            else if (action == Action::WE)
            {
                path+="WE";
            }
            else if (action == Action::NA)
            {
                path+="T";
            }
            else
            {
                path+="W";
            }
        }
        apaths.push_back(path);
    }

    return apaths;
}

json Simulator::planned_path_to_json() const
{
    //planned paths
    json ppaths = json::array();
    for (int i = 0; i < num_of_agents; i++)
    {
        std::string path;
        bool first = true;
        for (const auto action : planner_movements[i])
        {
            if (!first)
            {
                path+= ",";
            } 
            else 
            {
                first = false;
            }
            // path+=action;

            // if (action == Action::FW)
            // {
            //     path+="F";
            // }
            // else if (action == Action::CR)
            // {
            //     path+="R";
            // } 
            // else if (action == Action::CCR)
            // {
            //     path+="C";
            // } 
            // else if (action == Action::NA)
            // {
            //     path+="T";
            // }
            // else
            // {
            //     path+="W";
            // }
            if (action == Action::N)
            {
                path+="NO";
            }
            else if (action == Action::E)
            {
                path+="EA";
            } 
            else if (action == Action::S)
            {
                path+="SO";
            }
            else if (action == Action::WE)
            {
                path+="WE";
            }
            else if (action == Action::NA)
            {
                path+="T";
            }
            else
            {
                path+="W";
            }
        }  
        ppaths.push_back(path);
    }

    return ppaths;
}

json Simulator::starts_to_json() const
{
    json start = json::array();
    for (int i = 0; i < num_of_agents; i++)
    {
        json s = json::array();
        s.push_back(starts[i].location/map.cols);
        s.push_back(starts[i].location%map.cols);
        switch (starts[i].orientation)
        {
        case 0:
            s.push_back("E");
            break;
        case 1:
            s.push_back("S");
        case 2:
            s.push_back("W");
            break;
        case 3:
            s.push_back("N");
            break;
        }
        start.push_back(s);
    }

    return start;
}

json Simulator::action_errors_to_json() const
{
    // Save errors
    json errors = json::array();
    for (auto error: model->errors)
    {
        std::string error_msg;
        int agent1;
        int agent2;
        int timestep;
        std::tie(error_msg,agent1,agent2,timestep) = error;
        json e = json::array();
        e.push_back(agent1);
        e.push_back(agent2);
        e.push_back(timestep);
        e.push_back(error_msg);
        errors.push_back(e);
    }

    return errors;
}