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

void schedule_plan_raw(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env)
{
    //use at most half of time_limit to compute schedule, -10 for timing error tolerance
    //so that the remainning time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;

    // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    int min_task_i, min_task_makespan, dist, c_loc, count;
    clock_t start = clock();

    // iterate over the free agents to decide which task to assign to each of them
    std::unordered_set<int>::iterator it = free_agents.begin();
    while (it != free_agents.end())
    {
        //keep assigning until timeout
        if (std::chrono::steady_clock::now() > endtime)
        {
            break;
        }
        int i = *it;

        assert(env->curr_task_schedule[i] == -1);
            
        min_task_i = -1;
        min_task_makespan = INT_MAX;
        count = 0;

        // iterate over all the unassigned tasks to find the one with the minimum makespan for agent i
        for (int t_id : free_tasks)
        {
            //check for timeout every 10 task evaluations
            if (count % 10 == 0 && std::chrono::steady_clock::now() > endtime)
            {
                break;
            }
            dist = 0;
            c_loc = env->curr_states.at(i).location;

            // iterate over the locations (errands) of the task to compute the makespan to finish the task
            // makespan: the time for the agent to complete all the errands of the task t_id in order
            for (int loc : env->task_pool[t_id].locations){
                dist += DefaultPlanner::get_h(env, c_loc, loc);
                c_loc = loc;
            }

            // update the new minimum makespan
            if (dist < min_task_makespan){
                min_task_i = t_id;
                min_task_makespan = dist;
            }
            count++;            
        }

        // assign the best free task to the agent i (assuming one exists)
        if (min_task_i != -1){
            proposed_schedule[i] = min_task_i;
            it = free_agents.erase(it);
            free_tasks.erase(min_task_i);
        }
        // nothing to assign
        else{
            proposed_schedule[i] = -1;
            it++;
        }
    }
    #ifndef NDEBUG
    cout << "Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
    cout << "new free agents: " << env->new_freeagents.size() << " new tasks: "<< env->new_tasks.size() <<  endl;
    cout << "free agents: " << free_agents.size() << " free tasks: " << free_tasks.size() << endl;
    #endif
    return;

}

void schedule_plan(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env, std::vector<Int4> background_flow)
{

    int maximum_edges = 100;

    proposed_schedule.resize(env->num_of_agents, -1);

    vector<int>flexible_agent_ids(env->new_freeagents); //storing the agents not doing a opened task
    vector<int>flexible_task_ids(env->new_tasks); //storing the tasks we consider to swap/assign

    for (int i = 0; i < env->num_of_agents; i++)
    {
        int t_id = env->curr_task_schedule[i];
        if (t_id >= 0 && env->task_pool[t_id].idx_next_loc == 0) //task assigned but not opened
        {
            flexible_agent_ids.push_back(i);
            flexible_task_ids.push_back(t_id);
        }
        else if (t_id >= 0) //task opened
        {
            proposed_schedule[i] = t_id;
        }
    }

    //prepare for matching

    cout<<"num of flexible agents: "<<flexible_agent_ids.size()<<endl;
    cout<<"num of flexible tasks: "<<flexible_task_ids.size()<<endl;

    int num_workers = flexible_agent_ids.size();
    int num_tasks = flexible_task_ids.size();

    // maximum_edges = num_workers;

    if (maximum_edges > num_tasks)
        maximum_edges = num_tasks;

    //Create an environment
    GRBEnv grb_env = GRBEnv();
    grb_env.start();

    // Create an empty model
    GRBModel model = GRBModel(grb_env);

    std::vector<std::vector<int>> cost(num_workers,std::vector<int>(num_tasks));

    std::vector<std::vector<GRBVar>> x(num_workers, std::vector<GRBVar>(num_tasks));

    GRBLinExpr objective = 0;

    //computing heuristics
    vector<unordered_map<int,int>> agent_task_heuristic;
    agent_task_heuristic.resize(env->num_of_agents);
    std::deque<HNode> open;
    std::unordered_map<int,HNode*> all_nodes;
    unordered_set<int> closed;
    unordered_map<int,list<int>> task_loc_ids;
    int goal_reach_cnt;
    for (int id: flexible_task_ids)
    {
        task_loc_ids[env->task_pool[id].locations[0]].push_back(id);
    }

    for (int id: flexible_agent_ids)
    {
        open.clear();
        closed.clear();
        goal_reach_cnt = 0;
        int goal_location = env->curr_states[id].location;
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
            if (task_loc_ids.find(curr.location)!= task_loc_ids.end())
            {
                for (int t_id: task_loc_ids[curr.location])
                {
                    agent_task_heuristic[id][t_id] = curr.value;
                    goal_reach_cnt++;
                    if (env->task_pool[t_id].agent_assigned < 0 && env->curr_task_schedule[id] < 0) //no assignment yet
                    {
                        //set an assignment greedily
                        env->curr_task_schedule[id] = t_id;
                        env->task_pool[t_id].agent_assigned = id;
                    }
                }
            }

            if (env->curr_task_schedule[id] >= 0 && goal_reach_cnt >= maximum_edges && agent_task_heuristic[id].find(env->curr_task_schedule[id]) != agent_task_heuristic[id].end())
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
        for (int j = 0; j < num_tasks; j++)
        {
            if (agent_task_heuristic[flexible_agent_ids[i]].find(flexible_task_ids[j]) == agent_task_heuristic[flexible_agent_ids[i]].end())
                continue;
            cost[i][j] = agent_task_heuristic[flexible_agent_ids[i]][flexible_task_ids[j]];
            x[i][j] = model.addVar(0.0, 1.0, cost[i][j], GRB_BINARY, "x_" + std::to_string(i) + "_" + std::to_string(j));
            objective += cost[i][j] * x[i][j];

            // //warm start with initial solution
            if (env->curr_task_schedule[flexible_agent_ids[i]] == flexible_task_ids[j])
                x[i][j].set(GRB_DoubleAttr_Start, 1.0);
            else    
                x[i][j].set(GRB_DoubleAttr_Start, 0.0);
        }
    }

    model.setObjective(objective, GRB_MINIMIZE);

    for (int i = 0; i < num_workers; ++i) 
    {
        GRBLinExpr worker_sum = 0;
        for (int j = 0; j < num_tasks; ++j) 
        {
            if (agent_task_heuristic[flexible_agent_ids[i]].find(flexible_task_ids[j]) == agent_task_heuristic[flexible_agent_ids[i]].end())
                continue;
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
            if (agent_task_heuristic[flexible_agent_ids[i]].find(flexible_task_ids[j]) == agent_task_heuristic[flexible_agent_ids[i]].end())
                continue;
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
                if (agent_task_heuristic[flexible_agent_ids[i]].find(flexible_task_ids[j]) == agent_task_heuristic[flexible_agent_ids[i]].end())
                    continue; // Skip invalid assignments
                    
                if (x[i][j].get(GRB_DoubleAttr_X) > 0.5) 
                {  // Check if x[i][j] is 1
                    std::cout << "Worker " << i << " assigned to Task " << j
                                << " with cost " << cost[i][j] << std::endl;
                    proposed_schedule[flexible_agent_ids[i]] = env->task_pool[flexible_task_ids[j]].task_id;
                    env->task_pool[flexible_task_ids[j]].agent_assigned = flexible_agent_ids[i];
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
