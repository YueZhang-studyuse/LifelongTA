#include "scheduler.h"
#include "gurobi_c++.h"

namespace DefaultPlanner{

std::mt19937 mt;
std::unordered_set<int> free_agents;
std::unordered_set<int> free_tasks;

void schedule_initialize(int preprocess_time_limit, SharedEnvironment* env)
{
    // cout<<"schedule initialise limit" << preprocess_time_limit<<endl;
    DefaultPlanner::init_heuristics(env);
    mt.seed(0);
    return;
}

void schedule_plan(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env, std::vector<Int4> background_flow)
{
    // // //use at most half of time_limit to compute schedule, -10 for timing error tolerance
    // //so that the remainning time are left for path planner
    // TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // // cout<<"schedule plan limit" << time_limit <<endl;

    // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());
    // proposed_schedule.resize(env->num_of_agents, -1); //default no schedule


    // int i_task, min_task_i, min_task_makespan, dist, c_loc, count;
    // clock_t start = clock();

    // for (int i = 0; i < env->num_of_agents && std::chrono::steady_clock::now() < endtime; i++)
    // {
        
    //     if (env->curr_task_schedule[i] == -1)
    //     {
            
    //         min_task_i = -1;
    //         min_task_makespan = INT_MAX;
    //         count = 0;
    //         for (i_task=0 ; i_task < env->task_pool.size() && std::chrono::steady_clock::now() < endtime ;i_task++)
    //         {                

    //             if (env->task_pool[i_task].agent_assigned != -1)
    //                 continue;
    //             dist = 0;
    //             c_loc = env->curr_states.at(i).location;
    //             for (int loc : env->task_pool[i_task].locations){
    //                 dist += DefaultPlanner::get_h(env, c_loc, loc);
    //                 c_loc = loc;
    //             }
    //             if (dist < min_task_makespan){
    //                 min_task_i = i_task;
    //                 min_task_makespan = dist;
    //             }
    //             count++;            
    //         }


    //         if (min_task_i != -1){
    //             proposed_schedule[i] = env->task_pool[min_task_i].task_id;
    //             env->task_pool[min_task_i].agent_assigned = i;
    //         }
    //         else{
    //             proposed_schedule[i] = -1;
    //         }
            

    //     }
    //     else
    //     {
    //         proposed_schedule[i] = env->curr_task_schedule[i];
    //     }
    // }
    // cout << ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;

    proposed_schedule.resize(env->num_of_agents, -1);

    unordered_set<int>opened_agents;
    unordered_set<int>opened_tasks;
    vector<int>agent_id_matches;
    vector<int>task_id_matches;

    //pre check the opened tasks.
    for (int i_task=0 ; i_task < env->task_pool.size() ;i_task++)
    {
        if (env->task_pool[i_task].idx_next_loc > 0) //task opened
        //if (env->task_pool[i_task].agent_assigned != -1) //agent assigned
        {
            opened_agents.insert(env->task_pool[i_task].agent_assigned);
            opened_tasks.insert(i_task);
            proposed_schedule[env->task_pool[i_task].agent_assigned] = env->curr_task_schedule[env->task_pool[i_task].agent_assigned];
        }
        else
        {
            task_id_matches.push_back(i_task);
        }
    }

    for (int i = 0; i < env->num_of_agents; i++)
    {
        if (opened_agents.find(i) == opened_agents.end())
        {
            agent_id_matches.push_back(i);
        }
    }

    int num_workers = agent_id_matches.size();
    int num_tasks = task_id_matches.size();




    //Create an environment
    GRBEnv grb_env = GRBEnv();
    grb_env.start();

    // Create an empty model
    GRBModel model = GRBModel(grb_env);

    std::vector<std::vector<int>> cost(num_workers,std::vector<int>(num_tasks));

    std::vector<std::vector<GRBVar>> x(num_workers, std::vector<GRBVar>(num_tasks));

    GRBLinExpr objective = 0;

    //naive code for dijkstra using traffic distance
    vector<unordered_map<int,int>> task_heuristics;
    task_heuristics.resize(num_tasks);
    std::deque<HNode> open;
    std::unordered_map<int,HNode*> all_nodes;
    //add locations
    unordered_set<int> dij_goals;
    unordered_set<int> closed;
    for (int j = 0; j < num_tasks; j++)
    {
        dij_goals.clear();
        dij_goals.insert(env->task_pool[task_id_matches[j]].locations[1]);
        for (int i = 0; i < num_workers; i++)
        {
            dij_goals.insert(env->curr_states.at(agent_id_matches[i]).location);
        }

        open.clear();
        closed.clear();
        int goal_location = env->task_pool[task_id_matches[j]].locations[0];
        HNode root(goal_location,0, 0);
        open.push_back(root);
        closed.insert(goal_location);

        std::vector<int> neighbors;
        int  diff, d, cost, op_flow, total_cross, all_vertex_flow,vertex_flow, depth,p_diff, p_d;
        int next_d1, next_d2, next_d1_loc, next_d2_loc;
        int temp_op, temp_vertex;
		while (!open.empty())
		{
			HNode curr = open.front();
			open.pop_front();
            closed.insert(curr.location);
            if (dij_goals.find(curr.location) != dij_goals.end())
            {
                task_heuristics[j][curr.location] = curr.value;
                dij_goals.erase(curr.location);
                //cout<<"find goal"<<curr.location<<" "<<dij_goals.size()<<endl;
            }
            if (dij_goals.empty())
                break;
			
			neighbors = global_neighbors.at(curr.location);
			
			for (int next : neighbors)
			{
				if (closed.find(next) != closed.end())
                    continue;
                
                cost = curr.value + 1;
                op_flow = 0;
                all_vertex_flow = 0;
                diff = curr.location-next;
                d = get_d(diff,env);
                temp_op = ( (background_flow[curr.location].d[d]+1) * background_flow[next].d[(d+2)%4]);
                temp_vertex = 1;
                for (int j=0; j<4; j++)
                {
                    temp_vertex += background_flow[next].d[j];                
                }
                op_flow += temp_op;
                all_vertex_flow+= (temp_vertex-1) /2;

                cost = cost + op_flow + all_vertex_flow;

                if (all_nodes.find(next) != all_nodes.end())
                {
                    HNode* old = all_nodes[next];
                    if (cost < old->value)
                    {
                        old->value = cost;
                    }
                }
                else
                {
                    HNode next_node(next,0, cost);
                    open.push_back(next_node);
                    all_nodes[next] = &next_node;
                }
				
			}
		}
        all_nodes.clear();
    }

    for (int i = 0; i < num_workers; i++)
    {
        int start = env->curr_states.at(agent_id_matches[i]).location;
        for (int j = 0; j < num_tasks; j++)
        {
            //we assume pick_up + delivery
            int goal = env->task_pool[task_id_matches[j]].locations[0];
            int goal2 = env->task_pool[task_id_matches[j]].locations[1];
            // cost[i][j] = DefaultPlanner::get_h(env, start, goal) + DefaultPlanner::get_h(env, goal, goal2);
            cost[i][j] = task_heuristics[j][start]+task_heuristics[j][goal2];
            x[i][j] = model.addVar(0.0, 1.0, cost[i][j], GRB_BINARY, "x_" + std::to_string(i) + "_" + std::to_string(j));
            objective += cost[i][j] * x[i][j];
        }
    }

    model.setObjective(objective, GRB_MINIMIZE);

    for (int i = 0; i < num_workers; ++i) 
    {
        GRBLinExpr worker_sum = 0;
        for (int j = 0; j < num_tasks; ++j) 
        {
            worker_sum += x[i][j];
        }
        model.addConstr(worker_sum == 1, "worker_" + std::to_string(i));
    }

    // Add constraints: each task can be assigned to at most one worker
    for (int j = 0; j < num_tasks; ++j) 
    {
        GRBLinExpr task_sum = 0;
        for (int i = 0; i < num_workers; ++i)
        {
            task_sum += x[i][j];
        }
        model.addConstr(task_sum <= 1, "task_" + std::to_string(j));
    }

    model.optimize();

    // Display the results
    if (model.get(GRB_IntAttr_Status) == GRB_OPTIMAL) 
    {
        std::cout << "Optimal assignment with minimum cost:" << std::endl;
        for (int i = 0; i < num_workers; ++i) 
        {
            for (int j = 0; j < num_tasks; ++j) 
            {
                if (x[i][j].get(GRB_DoubleAttr_X) > 0.5) 
                {  // Check if x[i][j] is 1
                    std::cout << "Worker " << i << " assigned to Task " << j
                                << " with cost " << cost[i][j] << std::endl;
                    proposed_schedule[agent_id_matches[i]] = env->task_pool[task_id_matches[j]].task_id;
                    env->task_pool[task_id_matches[j]].agent_assigned = agent_id_matches[i];
                }
            }
        }
        std::cout << "Total minimum cost: " << model.get(GRB_DoubleAttr_ObjVal) << std::endl;
    } 
    else 
    {
        std::cout << "No optimal solution found." << std::endl;
    }
    
}
}
