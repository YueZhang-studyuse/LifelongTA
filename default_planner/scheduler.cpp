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

void schedule_plan_full(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env, std::vector<Int4> background_flow)
{
    proposed_schedule.resize(env->num_of_agents, -1);

    unordered_set<int>opened_agents;
    // unordered_set<int>opened_tasks;
    vector<int>agent_ids;
    vector<int>task_ids;

    //pre check the opened tasks.
    // for (int i_task=0 ; i_task < env->task_pool.size() ;i_task++)
    for (auto task: env->task_pool)
    {
        if (task.second.idx_next_loc > 0) //task opened
        {
            opened_agents.insert(task.second.agent_assigned);
            // opened_tasks.insert(i_task);
            proposed_schedule[task.second.agent_assigned] = task.first;
        }
        else
        {
            task_ids.push_back(task.first);
        }
    }

    for (int i = 0; i < env->num_of_agents; i++)
    {
        if (opened_agents.find(i) == opened_agents.end())
        {
            agent_ids.push_back(i);
        }
    }

    cout<<"num of background agents: "<<opened_agents.size()<<endl;

    int num_workers = agent_ids.size();
    int num_tasks = task_ids.size();




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
    // if (env->curr_timestep > 0)
    // {
    for (int j = 0; j < num_tasks; j++)
    {
        dij_goals.clear();
        //dij_goals.insert(env->task_pool[task_ids[j]].locations[1]);
        for (int i = 0; i < num_workers; i++)
        {
            dij_goals.insert(env->curr_states.at(agent_ids[i]).location);
        }

        open.clear();
        closed.clear();
        int goal_location = env->task_pool[task_ids[j]].locations[0];
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
    //}

    for (int i = 0; i < num_workers; i++)
    {
        int start = env->curr_states.at(agent_ids[i]).location;
        for (int j = 0; j < num_tasks; j++)
        {
            //we assume pick_up + delivery
            int goal = env->task_pool[task_ids[j]].locations[0];
            int goal2 = env->task_pool[task_ids[j]].locations[1];
            // if (env->curr_timestep == 0)
            //     cost[i][j] = DefaultPlanner::get_h(env, start, goal) + DefaultPlanner::get_h(env, goal, goal2);
            // else
                cost[i][j] = task_heuristics[j][start];
                //+task_heuristics[j][goal2];
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
                    proposed_schedule[agent_ids[i]] = env->task_pool[task_ids[j]].task_id;
                    env->task_pool[task_ids[j]].agent_assigned = agent_ids[i];
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


void schedule_plan(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env, std::vector<Int4> background_flow)
{

    int maximum_edges = 1000;

    proposed_schedule.resize(env->num_of_agents, -1);

    vector<int>flexible_agent_ids(env->new_freeagents); //storing the agents not doing a opened task
    vector<int>flexible_task_ids; //storing the tasks we consider to swap/assign

    for (auto task: env->task_pool)
    {
        if (task.second.idx_next_loc > 0) //task opened
        {
            proposed_schedule[task.second.agent_assigned] = task.first;
        }
        else
        {
            flexible_task_ids.push_back(task.first);
            if (task.second.agent_assigned != -1)
                flexible_agent_ids.push_back(task.second.agent_assigned);
            
        }
    }

    //prepare for matching

    cout<<"num of flexible agents: "<<flexible_agent_ids.size()<<endl;
    cout<<"num of flexible tasks: "<<flexible_task_ids.size()<<endl;

    int num_workers = flexible_agent_ids.size();
    int num_tasks = flexible_task_ids.size();

    // maximum_edges = num_workers;

    //if (maximum_edges > num_tasks)
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
    unordered_map<int,int> task_id;
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
                    task_id[t_id] = 0;
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

    // //run a* from task pick up to task delivery
    // for (auto task: task_id)
    // {
    //     int id = task.first;
    //     open.clear();
    //     closed.clear();
    //     goal_reach_cnt = 0;
    //     int start = env->task_pool[id].locations[0];
    //     int goal = env->task_pool[id].locations[1];
    //     HNode root(start,0, DefaultPlanner::get_h(env, start, goal));
    //     open.push_back(root);
    //     closed.insert(start);

    //     std::vector<int> neighbors;
    //     int  diff, d, cost, op_flow, total_cross, all_vertex_flow,vertex_flow, depth,p_diff, p_d;
    //     int next_d1, next_d2, next_d1_loc, next_d2_loc;
    //     int temp_op, temp_vertex;
    //     while (!open.empty())
    //     {
    //         HNode curr = open.front();
    //         open.pop_front();
    //         closed.insert(curr.location);
            
    //         if (curr.location == goal)
    //         {
    //             task_id[id] = curr.value;
    //         }
            
    //         neighbors = global_neighbors.at(curr.location);
            
    //         for (int next : neighbors)
    //         {
    //             if (closed.find(next) != closed.end())
    //                 continue;
                
    //             cost = curr.value + 1;
    //             op_flow = 0;
    //             all_vertex_flow = 0;
    //             diff = curr.location-next;
    //             d = get_d(diff,env);
    //             temp_op = ( (background_flow[curr.location].d[d]+1) * background_flow[next].d[(d+2)%4]);
    //             temp_vertex = 1;
    //             for (int j=0; j<4; j++)
    //             {
    //                 temp_vertex += background_flow[next].d[j];                
    //             }
    //             op_flow += temp_op;
    //             all_vertex_flow+= (temp_vertex-1) /2;

    //             cost = cost + op_flow + all_vertex_flow;
    //             cost += DefaultPlanner::get_h(env, next, goal);

    //             if (all_nodes.find(next) != all_nodes.end())
    //             {
    //                 HNode* old = all_nodes[next];
    //                 if (cost < old->value)
    //                 {
    //                     old->value = cost;
    //                 }
    //             }
    //             else
    //             {
    //                 HNode next_node(next,0, cost);
    //                 open.push_back(next_node);
    //                 all_nodes[next] = &next_node;
    //             }
                
    //         }
    //     }
    //     all_nodes.clear();
    // }

    // Start timing
    auto start_time = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < num_workers; i++)
    {
        for (int j = 0; j < num_tasks; j++)
        {
            if (agent_task_heuristic[flexible_agent_ids[i]].find(flexible_task_ids[j]) == agent_task_heuristic[flexible_agent_ids[i]].end())
                continue;
            cost[i][j] = agent_task_heuristic[flexible_agent_ids[i]][flexible_task_ids[j]];
            // + task_id[flexible_task_ids[j]];
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
        // End timing
        auto end_time = std::chrono::high_resolution_clock::now();
        double elapsed_time = std::chrono::duration<double>(end_time - start_time).count();

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
        cout << "Solving time: " << elapsed_time << " seconds" << endl;
    } 
    else 
    {
        std::cout << "No optimal solution found." << std::endl;
    }
    
}

void schedule_plan_parallel(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env, std::vector<Int4> background_flow)
{

    int maximum_edges = 100;
    int num_threads = 4;

    proposed_schedule.resize(env->num_of_agents, -1);

    vector<int>flexible_agent_ids(env->new_freeagents); //storing the agents not doing a opened task
    vector<int>flexible_task_ids; //storing the tasks we consider to swap/assign
    vector<int>assignable_tasks(env->new_tasks);

    for (auto task: env->task_pool)
    {
        if (task.second.idx_next_loc > 0) //task opened
        {
            proposed_schedule[task.second.agent_assigned] = task.first;
        }
        else
        {
            flexible_task_ids.push_back(task.first);
            if (task.second.agent_assigned != -1)
                flexible_agent_ids.push_back(task.second.agent_assigned);
            if (task.second.agent_assigned < 0)
                assignable_tasks.push_back(task.first);

            
        }
    }

    cout<<"num of flexible agents: "<<flexible_agent_ids.size()<<endl;
    cout<<"num of flexible tasks: "<<flexible_task_ids.size()<<endl;

    int num_workers = flexible_agent_ids.size();
    int num_tasks = flexible_task_ids.size();

    if (maximum_edges > num_tasks)
        maximum_edges = num_tasks;

    //quick allocate for new free agents
    for (int agent_id: env->new_freeagents)
    {
        for (int i:assignable_tasks)
        {
            if (env->task_pool[i].agent_assigned >= 0)
                continue;
            if (proposed_schedule[agent_id] < 0) //agent  does not have an assignment yet
            {
                proposed_schedule[agent_id] = i;
            }
            int min_cost = manhattanDistance(env->curr_states[agent_id].location, env->task_pool[proposed_schedule[agent_id]].locations[0],env);
            int cost = manhattanDistance(env->curr_states[agent_id].location, env->task_pool[i].locations[0],env); 
            if (cost < min_cost)
            {
                proposed_schedule[agent_id] = i;
            }
        }
        env->task_pool[proposed_schedule[agent_id]].agent_assigned = agent_id;
        env->curr_task_schedule[agent_id] = proposed_schedule[agent_id];
    }

    unordered_map<int,list<int>> task_loc_ids;
    for (int id: flexible_task_ids)
    {
        task_loc_ids[env->task_pool[id].locations[0]].push_back(id);
    }
    clock_t start = clock();

    //parallel computing heuristics
    int num_workers_per_thread = num_workers / num_threads;
    std::vector<std::future<unordered_map<int,unordered_map<int,int>>>> results;
    
    for (int curr_thread = 0; curr_thread < num_threads; curr_thread++)
    {
        if (curr_thread == num_threads - 1)
            num_workers_per_thread = num_workers - curr_thread * num_workers_per_thread;
        vector<pair<int,int>> current_id_assignment;
        current_id_assignment.resize(num_workers_per_thread);
        for (int i = 0; i < num_workers_per_thread; i++)
        {
            current_id_assignment[i] = make_pair(flexible_agent_ids[curr_thread*num_workers_per_thread + i], env->curr_task_schedule[flexible_agent_ids[curr_thread*num_workers_per_thread + i]]);
        }
        results.push_back(std::async(std::launch::async, DefaultPlanner::compute_heuristics, env, background_flow, current_id_assignment, task_loc_ids, maximum_edges));
    }
    vector<unordered_map<int,int>> agent_task_heuristic;
    agent_task_heuristic.resize(env->num_of_agents);
    for (int curr_thread = 0; curr_thread < num_threads; curr_thread++)
    {
        auto result = results[curr_thread].get();
        for (auto r: result)
        {
            agent_task_heuristic[r.first] = r.second;
        }
    }
    cout << "Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;

    //gurobi matching
    //Create an environment
    GRBEnv grb_env = GRBEnv();
    grb_env.start();

    // Create an empty model
    GRBModel model = GRBModel(grb_env);

    std::vector<std::vector<int>> cost(num_workers,std::vector<int>(num_tasks));

    std::vector<std::vector<GRBVar>> x(num_workers, std::vector<GRBVar>(num_tasks));

    GRBLinExpr objective = 0;

    for (int i = 0; i < num_workers; i++)
    {
        for (int j = 0; j < num_tasks; j++)
        {
            if (agent_task_heuristic[flexible_agent_ids[i]].find(flexible_task_ids[j]) == agent_task_heuristic[flexible_agent_ids[i]].end())
                continue;
            cost[i][j] = agent_task_heuristic[flexible_agent_ids[i]][flexible_task_ids[j]];
            // + task_id[flexible_task_ids[j]];
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

unordered_map<int,unordered_map<int,int>> compute_heuristics(SharedEnvironment* env, std::vector<Int4> background_flow, vector<pair<int,int>> current_id_assignment, unordered_map<int,list<int>> task_loc_ids, int max_num_tasks)
{
    cout<<"start"<<endl;
    //computing heuristics
    unordered_map<int, unordered_map<int,int>> agent_task_heuristic;
    std::deque<HNode> open;
    std::unordered_map<int,HNode*> all_nodes;
    unordered_set<int> closed;
    int goal_reach_cnt;

    for (pair<int,int> id_pair: current_id_assignment)
    {
        int id = id_pair.first;
        int pre_goal = id_pair.second;
        bool pre_goal_reach = false;
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
                    if (t_id == pre_goal)
                    {
                        pre_goal_reach = true;
                    }
                    goal_reach_cnt++;
                }
            }

            if (goal_reach_cnt >= max_num_tasks && pre_goal_reach)
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
    cout<<"finish"<<endl;
    return agent_task_heuristic;
}

void schedule_plan_lemon(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env, std::vector<Int4> background_flow)
{

    auto start_time = std::chrono::high_resolution_clock::now();

    int maximum_edges = 20;

    proposed_schedule.resize(env->num_of_agents, -1);

    vector<int>flexible_agent_ids(env->new_freeagents); //storing the agents not doing a opened task
    vector<int>flexible_task_ids; //storing the tasks we consider to swap/assign

    for (auto task: env->task_pool)
    {
        if (task.second.idx_next_loc > 0) //task opened
        {
            proposed_schedule[task.second.agent_assigned] = task.first;
        }
        else
        {
            flexible_task_ids.push_back(task.first);
            if (task.second.agent_assigned != -1)
                flexible_agent_ids.push_back(task.second.agent_assigned);
            
        }
    }

    //prepare for matching

    cout<<"num of flexible agents: "<<flexible_agent_ids.size()<<endl;
    cout<<"num of flexible tasks: "<<flexible_task_ids.size()<<endl;

    int num_workers = flexible_agent_ids.size();
    int num_tasks = flexible_task_ids.size();

    if (maximum_edges > num_tasks)
        maximum_edges = num_tasks;

    //computing heuristics
    vector<unordered_map<int,int>> agent_task_heuristic;
    agent_task_heuristic.resize(env->num_of_agents);
    std::deque<HNode> open;
    std::unordered_map<int,HNode*> all_nodes;
    unordered_set<int> closed;
    unordered_map<int,list<int>> task_loc_ids;
    int goal_reach_cnt;
    unordered_map<int,int> task_id;
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
                    task_id[t_id] = 0;
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

    cout<<"Dijkstra time: "<<std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start_time).count()<<endl;
    // Start timing
    start_time = std::chrono::high_resolution_clock::now();
    
    // Create the graph
    ListDigraph g;
    ListDigraph::NodeMap<int> supply(g);
    ListDigraph::ArcMap<double> cost(g);
    ListDigraph::ArcMap<int> capacity(g);
    ListDigraph::ArcMap<int> flow(g); // Store the flow for warm start

    // Create worker and task nodes
    vector<ListDigraph::Node> workers(num_workers);
    vector<ListDigraph::Node> tasks(num_tasks);

    ListDigraph::Node source = g.addNode(); // Source node
    ListDigraph::Node sink = g.addNode();   // Sink node

    // Create worker and task nodes
    for (int i = 0; i < num_workers; ++i) workers[i] = g.addNode();
    for (int j = 0; j < num_tasks; ++j) tasks[j] = g.addNode();

    // Set supply/demand values
    supply[source] = num_workers; // Source supplies workers
    supply[sink] = -num_workers;  // Sink absorbs tasks

    for (int i = 0; i < num_workers; ++i) supply[workers[i]] = 0;
    for (int j = 0; j < num_tasks; ++j) supply[tasks[j]] = 0;

    // Connect source to workers
    for (int i = 0; i < num_workers; ++i) 
    {
        ListDigraph::Arc a = g.addArc(source, workers[i]);
        capacity[a] = 1;
        cost[a] = 0; // No cost for assigning workers
    }

    // Connect tasks to sink
    for (int j = 0; j < num_tasks; ++j) 
    {
        ListDigraph::Arc a = g.addArc(tasks[j], sink);
        capacity[a] = 1;
        cost[a] = 0; // No cost for completing tasks
    }

    unordered_map<int, unordered_map<int, ListDigraph::Arc>> edges;
    // Add arcs between workers and tasks with costs and capacities
    for (int i = 0; i < num_workers; ++i) 
    {
        for (int j = 0; j < num_tasks; ++j) 
        {
            if (agent_task_heuristic[flexible_agent_ids[i]].find(flexible_task_ids[j]) == agent_task_heuristic[flexible_agent_ids[i]].end())
                continue;
            ListDigraph::Arc a = g.addArc(workers[i], tasks[j]);
            cost[a] = agent_task_heuristic[flexible_agent_ids[i]][flexible_task_ids[j]]; // Assign the cost from the heuristic
            capacity[a] = 1; // Each worker can be assigned to at most one task
            edges[i][j] = a;

            // Initialize flow based on a warm start (for example, from a heuristic or previous solution)
            if (env->curr_task_schedule[flexible_agent_ids[i]] == flexible_task_ids[j]) 
            {  
                flow[a] = 1;
            } 
            else 
            {
                flow[a] = 0;
            }
        }
    }
    // NetworkSimplex setup
    NetworkSimplex<ListDigraph> ns(g);
    ns.costMap(cost);
    ns.upperMap(capacity);
    ns.supplyMap(supply);
    ns.flowMap(flow); // Use the initial flow (warm start)

    //printDIMACS(g, source, sink, workers, tasks, capacity, cost);
    
    if (ns.run() == NetworkSimplex<ListDigraph>::OPTIMAL) 
    {
        // End timing
        auto end_time = std::chrono::high_resolution_clock::now();
        double elapsed_time = std::chrono::duration<double>(end_time - start_time).count();
        int cnt = 0;

        cout << "Optimal assignment with minimum cost:" << endl;
        for (int i = 0; i < num_workers; ++i) 
        {
            for (int j = 0; j < num_tasks; ++j) 
            {
                if (edges[i].find(j) == edges[i].end()) continue; // Skip invalid pairs
                ListDigraph::Arc a = edges[i][j];

                if (ns.flow(a) > 0) // Flow > 0 means assignment exists
                {  
                    cnt++;
                    // cout << "Worker " << i << " assigned to Task " << j 
                    //      << " with cost " << cost[a] << endl;
                    proposed_schedule[flexible_agent_ids[i]] = flexible_task_ids[j];
                }
            }
        }
        cout << "Total assignment: " << cnt << endl;
        cout << "Total minimum cost: " << ns.totalCost<double>() << endl;
        cout << "Solving time: " << elapsed_time << " seconds" << endl;
    } 
    else 
    {
        cout << "No optimal solution found." << endl;
    }

}
void printDIMACS(ListDigraph& g, 
                 ListDigraph::Node source, 
                 ListDigraph::Node sink, 
                 vector<ListDigraph::Node>& workers, 
                 vector<ListDigraph::Node>& tasks, 
                 ListDigraph::ArcMap<int>& capacity, 
                 ListDigraph::ArcMap<double>& cost) 
{
    int num_workers = workers.size();
    int num_tasks = tasks.size();
    int num_nodes = 2 + num_workers + num_tasks; // source, sink, workers, tasks
    int num_arcs = num_workers + num_tasks + (num_workers * num_tasks); // source→workers + tasks→sink + workers→tasks

    // Print problem line
    cout << "p min " << num_nodes << " " << num_arcs << endl;

    // Print node descriptors
    cout << "n 1 " << num_workers << "   c Source (supplies workers)" << endl;
    cout << "n " << num_nodes << " -" << num_workers << "   c Sink (absorbs workers)" << endl;

    // Print arcs (Source → Workers)
    for (int i = 0; i < num_workers; ++i) {
        cout << "a 1 " << (i + 2) << " 0 1 0   c Source to Worker " << (i + 1) << endl;
    }

    // Print arcs (Workers → Tasks)
    for (int i = 0; i < num_workers; ++i) {
        for (int j = 0; j < num_tasks; ++j) {
            ListDigraph::Arc a = findArc(g, workers[i], tasks[j]);
            if (a == INVALID) continue; // Skip if no valid edge

            cout << "a " << (i + 2) << " " << (num_workers + j + 2) << " 0 " 
                 << capacity[a] << " " << cost[a] 
                 << "   c Worker " << (i + 1) << " to Task " << (j + 1) << endl;
        }
    }

    // Print arcs (Tasks → Sink)
    for (int j = 0; j < num_tasks; ++j) {
        cout << "a " << (num_workers + j + 2) << " " << num_nodes << " 0 1 0   c Task " << (j + 1) << " to Sink" << endl;
    }
}

void schedule_plan_greedy(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env, std::vector<Int4> background_flow)
{

    auto start_time = std::chrono::high_resolution_clock::now();


    proposed_schedule.resize(env->num_of_agents, -1);

    vector<int>flexible_agent_ids(env->new_freeagents); //storing the agents not doing a opened task
    vector<int>flexible_task_ids; //storing the tasks we consider to swap/assign

    for (auto task: env->task_pool)
    {
        if (task.second.idx_next_loc > 0) //task opened
        {
            proposed_schedule[task.second.agent_assigned] = task.first;
        }
        else
        {
            flexible_task_ids.push_back(task.first);
            if (task.second.agent_assigned != -1)
                flexible_agent_ids.push_back(task.second.agent_assigned);
            
        }
    }

    //prepare for matching

    cout<<"num of flexible agents: "<<flexible_agent_ids.size()<<endl;
    cout<<"num of flexible tasks: "<<flexible_task_ids.size()<<endl;

    int num_workers = flexible_agent_ids.size();
    int num_tasks = flexible_task_ids.size();

    //computing heuristics
    vector<unordered_map<int,int>> agent_task_heuristic;
    agent_task_heuristic.resize(env->num_of_agents);
    std::deque<HNode> open;
    std::unordered_map<int,HNode*> all_nodes;
    unordered_set<int> closed;
    unordered_map<int,list<int>> task_loc_ids;
    int goal_reach_cnt;
    unordered_map<int,int> task_id;
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
                    task_id[t_id] = 0;
                    goal_reach_cnt++;
                    if (env->task_pool[t_id].agent_assigned < 0 && env->curr_task_schedule[id] < 0) //no assignment yet
                    {
                        //set an assignment greedily
                        env->curr_task_schedule[id] = t_id;
                        env->task_pool[t_id].agent_assigned = id;
                        proposed_schedule[id] = t_id;
                    }
                }
            }

            if (env->curr_task_schedule[id] >= 0 && agent_task_heuristic[id].find(env->curr_task_schedule[id]) != agent_task_heuristic[id].end())
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

    cout<<"Dijkstra time: "<<std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start_time).count()<<endl;

}

void schedule_plan_cost(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env)
{

    auto start_time = std::chrono::high_resolution_clock::now();

    proposed_schedule.resize(env->num_of_agents, -1);

    vector<int>flexible_agent_ids(env->new_freeagents); //storing the agents not doing a opened task
    vector<int>flexible_task_ids; //storing the tasks we consider to swap/assign

    for (auto task: env->task_pool)
    {
        if (task.second.idx_next_loc > 0) //task opened
        {
            proposed_schedule[task.second.agent_assigned] = task.first;
        }
        else
        {
            flexible_task_ids.push_back(task.first);
            if (task.second.agent_assigned != -1)
                flexible_agent_ids.push_back(task.second.agent_assigned);
            
        }
    }

    //prepare for matching

    cout<<"num of flexible agents: "<<flexible_agent_ids.size()<<endl;
    cout<<"num of flexible tasks: "<<flexible_task_ids.size()<<endl;

    int num_workers = flexible_agent_ids.size();
    int num_tasks = flexible_task_ids.size();

    int maximum_edges = num_tasks;

    //computing heuristics
    vector<unordered_map<int,int>> agent_task_heuristic;
    agent_task_heuristic.resize(env->num_of_agents);
    std::deque<HNode> open;
    std::unordered_map<int,HNode*> all_nodes;
    unordered_set<int> closed;
    unordered_map<int,list<int>> task_loc_ids;
    int goal_reach_cnt;
    unordered_map<int,int> task_id;
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
                    task_id[t_id] = 0;
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

    cout<<"Dijkstra time: "<<std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start_time).count()<<endl;
    // Start timing
    start_time = std::chrono::high_resolution_clock::now();
    
    // Create the graph
    ListDigraph g;
    ListDigraph::NodeMap<int> supply(g);
    ListDigraph::ArcMap<double> cost(g);
    ListDigraph::ArcMap<int> capacity(g);
    ListDigraph::ArcMap<int> flow(g); // Store the flow for warm start

    // Create worker and task nodes
    vector<ListDigraph::Node> workers(num_workers);
    vector<ListDigraph::Node> tasks(num_tasks);

    ListDigraph::Node source = g.addNode(); // Source node
    ListDigraph::Node sink = g.addNode();   // Sink node

    // Create worker and task nodes
    for (int i = 0; i < num_workers; ++i) workers[i] = g.addNode();
    for (int j = 0; j < num_tasks; ++j) tasks[j] = g.addNode();

    // Set supply/demand values
    supply[source] = num_workers; // Source supplies workers
    supply[sink] = -num_workers;  // Sink absorbs tasks

    for (int i = 0; i < num_workers; ++i) supply[workers[i]] = 0;
    for (int j = 0; j < num_tasks; ++j) supply[tasks[j]] = 0;

    // Connect source to workers
    for (int i = 0; i < num_workers; ++i) 
    {
        ListDigraph::Arc a = g.addArc(source, workers[i]);
        capacity[a] = 1;
        cost[a] = 0; // No cost for assigning workers
    }

    // Connect tasks to sink
    for (int j = 0; j < num_tasks; ++j) 
    {
        ListDigraph::Arc a = g.addArc(tasks[j], sink);
        capacity[a] = 1;
        cost[a] = 0; // No cost for completing tasks
    }

    unordered_map<int, unordered_map<int, ListDigraph::Arc>> edges;
    // Add arcs between workers and tasks with costs and capacities
    for (int i = 0; i < num_workers; ++i) 
    {
        for (int j = 0; j < num_tasks; ++j) 
        {
            if (agent_task_heuristic[flexible_agent_ids[i]].find(flexible_task_ids[j]) == agent_task_heuristic[flexible_agent_ids[i]].end())
                continue;
            ListDigraph::Arc a = g.addArc(workers[i], tasks[j]);
            cost[a] = agent_task_heuristic[flexible_agent_ids[i]][flexible_task_ids[j]]; // Assign the cost from the heuristic
            capacity[a] = 1; // Each worker can be assigned to at most one task
            edges[i][j] = a;

            // Initialize flow based on a warm start (for example, from a heuristic or previous solution)
            if (env->curr_task_schedule[flexible_agent_ids[i]] == flexible_task_ids[j]) 
            {  
                flow[a] = 1;
            } 
            else 
            {
                flow[a] = 0;
            }
        }
    }
    // NetworkSimplex setup
    NetworkSimplex<ListDigraph> ns(g);
    ns.costMap(cost);
    ns.upperMap(capacity);
    ns.supplyMap(supply);
    ns.flowMap(flow); // Use the initial flow (warm start)

    //printDIMACS(g, source, sink, workers, tasks, capacity, cost);
    
    if (ns.run() == NetworkSimplex<ListDigraph>::OPTIMAL) 
    {
        // End timing
        auto end_time = std::chrono::high_resolution_clock::now();
        double elapsed_time = std::chrono::duration<double>(end_time - start_time).count();
        int cnt = 0;

        cout << "Optimal assignment with minimum cost:" << endl;
        for (int i = 0; i < num_workers; ++i) 
        {
            for (int j = 0; j < num_tasks; ++j) 
            {
                if (edges[i].find(j) == edges[i].end()) continue; // Skip invalid pairs
                ListDigraph::Arc a = edges[i][j];

                if (ns.flow(a) > 0) // Flow > 0 means assignment exists
                {  
                    cnt++;
                    // cout << "Worker " << i << " assigned to Task " << j 
                    //      << " with cost " << cost[a] << endl;
                    proposed_schedule[flexible_agent_ids[i]] = flexible_task_ids[j];
                }
            }
        }
        cout << "Total assignment: " << cnt << endl;
        cout << "Total minimum cost: " << ns.totalCost<double>() << endl;
        cout << "Solving time: " << elapsed_time << " seconds" << endl;
    } 
    else 
    {
        cout << "No optimal solution found." << endl;
    }

}

void schedule_plan_cost_greedy(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env)
{

    auto start_time = std::chrono::high_resolution_clock::now();

    proposed_schedule.resize(env->num_of_agents, -1);

    vector<int>flexible_agent_ids(env->new_freeagents); //storing the agents not doing a opened task
    vector<int>flexible_task_ids; //storing the tasks we consider to swap/assign

    for (auto task: env->task_pool)
    {
        if (task.second.idx_next_loc > 0) //task opened
        {
            proposed_schedule[task.second.agent_assigned] = task.first;
        }
        else
        {
            flexible_task_ids.push_back(task.first);
            if (task.second.agent_assigned != -1)
            {
                flexible_agent_ids.push_back(task.second.agent_assigned);
                proposed_schedule[task.second.agent_assigned] = task.first;
            }
            
        }
    }

    //prepare for matching

    cout<<"num of flexible agents: "<<flexible_agent_ids.size()<<endl;
    cout<<"num of flexible tasks: "<<flexible_task_ids.size()<<endl;

    //quick allocate for new free agents
    unordered_set<int> temp_assigned;
    for (int agent_id: flexible_agent_ids)
    {
        for (int i:flexible_task_ids)
        {
            if (temp_assigned.find(i) != temp_assigned.end())
                continue;
            if (proposed_schedule[agent_id] < 0) //agent does not have an assignment yet
            {
                proposed_schedule[agent_id] = i;
            }
            int min_cost = DefaultPlanner::get_h(env,env->curr_states[agent_id].location,env->task_pool[proposed_schedule[agent_id]].locations[0]);
            min_cost+= DefaultPlanner::get_h(env,env->task_pool[proposed_schedule[agent_id]].locations[1],env->task_pool[proposed_schedule[agent_id]].locations[0]);

            int cost = DefaultPlanner::get_h(env,env->curr_states[agent_id].location, env->task_pool[i].locations[0]); 
            cost+= DefaultPlanner::get_h(env,env->task_pool[i].locations[1], env->task_pool[i].locations[0]); 

            if (cost < min_cost)
            {
                proposed_schedule[agent_id] = i;
            }
        }
        env->task_pool[proposed_schedule[agent_id]].agent_assigned = agent_id;
        env->curr_task_schedule[agent_id] = proposed_schedule[agent_id];
        temp_assigned.insert(proposed_schedule[agent_id]);
    }
}

}
