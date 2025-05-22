#include "scheduler.h"
#include "gurobi_c++.h"
#include <boost/heap/pairing_heap.hpp>

namespace DefaultPlanner{

std::mt19937 mt;
std::unordered_set<int> free_agents;
std::unordered_set<int> free_tasks;

unordered_map<int,list<int>> agent_guide_path; //agent id, guide path from flow

struct Node
{
    int location;
    int value;

    Node() = default;
    Node(int location, int value) : location(location), value(value) {}
    // the following is used to compare nodes in the OPEN list
    struct compare_node
    {
        // returns true if n1 > n2 (note -- this gives us *min*-heap).
        bool operator()(const Node& n1, const Node& n2) const
        {
            return n1.value >= n2.value;
        }
    };  // used by OPEN (heap) to compare nodes (top of the heap has min f-val, and then highest g-val)
};

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

void schedule_plan_h(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env, bool new_only)
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
            if (new_only)
            {
                if (task.second.agent_assigned == -1)
                {
                    flexible_task_ids.push_back(task.first);
                }
            }
            else
            {
                flexible_task_ids.push_back(task.first);
                if (task.second.agent_assigned != -1)
                    flexible_agent_ids.push_back(task.second.agent_assigned);
            }
        }
    }

    //prepare for matching

    cout<<"num of flexible agents: "<<flexible_agent_ids.size()<<endl;
    cout<<"num of flexible tasks: "<<flexible_task_ids.size()<<endl;

    int num_workers = flexible_agent_ids.size();
    int num_tasks = flexible_task_ids.size();


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
            ListDigraph::Arc a = g.addArc(workers[i], tasks[j]);
            int agent_id = flexible_agent_ids[i];
            int task_id = flexible_task_ids[j];
            int h = manhattanDistance(env->curr_states[agent_id].location,env->task_pool[task_id].locations[0],env);
            //h+= DefaultPlanner::get_h(env,env->task_pool[task_id].locations[1],env->task_pool[task_id].locations[0]);
            cost[a] = h; // Assign the cost from the heuristic
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

//with cost
void schedule_plan_matching(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env, std::vector<Int4> background_flow, bool use_traffic, bool new_only, int maximum_edges)
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
            if (new_only)
            {
                if (task.second.agent_assigned == -1)
                {
                    flexible_task_ids.push_back(task.first);
                }
            }
            else
            {
                flexible_task_ids.push_back(task.first);
                if (task.second.agent_assigned != -1)
                    flexible_agent_ids.push_back(task.second.agent_assigned);
            }
        }
    }

    cout<<"num of flexible agents: "<<flexible_agent_ids.size()<<endl;
    cout<<"num of flexible tasks: "<<flexible_task_ids.size()<<endl;

    int num_workers = flexible_agent_ids.size();
    int num_tasks = flexible_task_ids.size();

    if (maximum_edges > num_workers)
        maximum_edges = num_workers;

    //computing heuristics
    vector<unordered_map<int,int>> agent_task_heuristic;
    agent_task_heuristic.resize(env->num_of_agents);
    unordered_map<int,list<int>> task_loc_ids;
    int goal_reach_cnt;
    unordered_map<int,int> task_id;

    for (int id: flexible_task_ids)
    {
        task_loc_ids[env->task_pool[id].locations[0]].push_back(id);
    }

    if (!use_traffic) //bfs
    {
        std::deque<Node> open;
        std::unordered_map<int,Node*> all_nodes;
        unordered_set<int> closed;

        for (int id: flexible_agent_ids)
        {
            open.clear();
            closed.clear();
            goal_reach_cnt = 0;
            int goal_location = env->curr_states[id].location;
            Node root(goal_location, 0);
            open.push_back(root);
            closed.insert(goal_location);

            std::vector<int> neighbors;
            int cost;

            while (!open.empty())
            {
                Node curr = open.front();
                open.pop_front();
                closed.insert(curr.location);
                
                neighbors = global_neighbors.at(curr.location);
                
                for (int next : neighbors)
                {
                    if (closed.find(next) != closed.end()) continue;

                    cost = curr.value + 1;
                    if (all_nodes.find(next) != all_nodes.end())
                    {
                        Node* old = all_nodes[next];
                        if (cost < old->value) old->value = cost;
                        else continue;
                    }
                    else
                    {
                        Node next_node(next, cost);
                        open.push_back(next_node);
                        all_nodes[next] = &next_node;
                    }
                    if (task_loc_ids.find(next)!= task_loc_ids.end())
                    {
                        for (int t_id: task_loc_ids[next])
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
                }
            }
            all_nodes.clear();
        }
        
    }
    else //dijkstra + traffic cost
    {
        boost::heap::pairing_heap< Node, boost::heap::compare<Node::compare_node> > open;
        std::unordered_map<int,Node*> all_nodes;
        unordered_set<int> closed;
        int  diff, d, cost, op_flow, all_vertex_flow,vertex_flow, temp_op, temp_vertex;

        for (int id: flexible_agent_ids)
        {
            open.clear();
            closed.clear();
            goal_reach_cnt = 0;
            int goal_location = env->curr_states[id].location;
            Node root(goal_location, 0);
            open.push(root);
            closed.insert(goal_location);

            std::vector<int> neighbors;
            int  diff, d, cost, op_flow, total_cross, all_vertex_flow,vertex_flow, depth,p_diff, p_d;
            int next_d1, next_d2, next_d1_loc, next_d2_loc;
            int temp_op, temp_vertex;

            while (!open.empty())
            {
                Node curr = open.top();
                open.pop();
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

                    cost = cost + (op_flow + all_vertex_flow);

                    if (all_nodes.find(next) != all_nodes.end())
                    {
                        Node* old = all_nodes[next];
                        if (cost < old->value)
                        {
                            old->value = cost;
                        }
                    }
                    else
                    {
                        Node next_node(next, cost);
                        open.push(next_node);
                        all_nodes[next] = &next_node;
                    }
                    
                }
            }
            all_nodes.clear();
        }
    }
    
    cout<<"Dijkstra time: "<<std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start_time).count()<<endl;

    //matching
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

void schedule_plan_flow(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env, std::vector<Int4> background_flow, bool use_traffic, bool new_only)
{
    auto start_time = std::chrono::high_resolution_clock::now();

    agent_guide_path.clear();

    proposed_schedule.resize(env->num_of_agents, -1);

    vector<int>flexible_agent_ids(env->new_freeagents); //storing the agents not doing a opened task
    vector<int>flexible_task_ids; //storing the tasks we consider to swap/assign
    unordered_map<int,list<int>> task_loc_ids;

    for (auto task: env->task_pool)
    {
        if (task.second.idx_next_loc > 0) //task opened
        {
            proposed_schedule[task.second.agent_assigned] = task.first;
        }
        else
        {
            if (new_only)
            {
                if (task.second.agent_assigned == -1)
                {
                    flexible_task_ids.push_back(task.first);
                    task_loc_ids[task.second.locations[0]].push_back(task.first);
                }
            }
            else
            {
                flexible_task_ids.push_back(task.first);
                task_loc_ids[task.second.locations[0]].push_back(task.first);
                if (task.second.agent_assigned != -1)
                    flexible_agent_ids.push_back(task.second.agent_assigned);
            }
        }
    }

    cout<<"num of flexible agents: "<<flexible_agent_ids.size()<<endl;
    cout<<"num of flexible tasks: "<<flexible_task_ids.size()<<endl;

    int num_workers = flexible_agent_ids.size();
    int num_tasks = flexible_task_ids.size();

    // Start timing
    start_time = std::chrono::high_resolution_clock::now();
    
    // Create the graph
    ListDigraph g;
    ListDigraph::NodeMap<int> supply(g);
    ListDigraph::ArcMap<double> cost(g);
    ListDigraph::ArcMap<int> capacity(g);
    ListDigraph::ArcMap<int> flow(g); // Store the flow for warm start

    vector<ListDigraph::Node> map_nodes(env->map.size());

    ListDigraph::Node source = g.addNode(); // Source node
    ListDigraph::Node sink = g.addNode();   // Sink node

    unordered_map<int, int> node_to_maploc; // map graph node id to env->map index
    unordered_map<int, int> maploc_to_node; // reverse

    // Create worker and task nodes
    for (int i = 0 ; i < env->map.size(); ++i)
    {
        map_nodes[i] = g.addNode();
        int id = lemon::ListDigraphBase::id(map_nodes[i]);
        node_to_maploc[id] = i;
        maploc_to_node[i] = id;
    } 

    // Set supply/demand values
    supply[source] = num_workers; // Source supplies workers
    supply[sink] = -num_workers;  // Sink absorbs tasks

    for (int i = 0; i < num_workers; ++i) supply[map_nodes[i]] = 0;

    // Connect source to workers
    for (int i = 0; i < num_workers; ++i) 
    {
        ListDigraph::Arc a = g.addArc(source, map_nodes[env->curr_states[flexible_agent_ids[i]].location]);
        capacity[a] = 1;
        cost[a] = 0; // No cost for assigning workers
    }

    unordered_map<int, int> node_to_task_id;


    for (auto task: task_loc_ids)
    {
        int loc = task.first;
        ListDigraph::Arc a = g.addArc(map_nodes[loc], sink);
        node_to_task_id[lemon::ListDigraphBase::id(map_nodes[loc])] = loc;
        capacity[a] = task.second.size();
        cost[a] = 0;
    }

    vector<int> neighbor = {-env->cols, 1, env->cols, -1};

    int  diff, d, op_flow, all_vertex_flow,vertex_flow;
    int temp_op, temp_vertex;

    for (int loc = 0; loc < env->map.size(); loc++)
    {
        if (env->map[loc] != 0) continue;
        //try four directions
        for (int i = 0; i < 4; i++)
        {
            int neighbor_loc = loc + neighbor[i];
            if (neighbor_loc < 0 || neighbor_loc >= env->map.size() || env->map[neighbor_loc] != 0)
                continue;
            ListDigraph::Arc a = g.addArc(map_nodes[loc], map_nodes[neighbor_loc]);

            if (use_traffic)
            {
                op_flow = 0;
                all_vertex_flow = 0;
                diff = loc-neighbor_loc;
                d = get_d(diff,env);
                temp_op = ( (background_flow[loc].d[d]+1) * background_flow[neighbor_loc].d[(d+2)%4]);
                temp_vertex = 1;
                for (int j=0; j<4; j++)
                {
                    temp_vertex += background_flow[neighbor_loc].d[j];                
                }
                op_flow += temp_op;
                all_vertex_flow+= (temp_vertex-1) /2;
                cost[a] = 1 + op_flow + all_vertex_flow;
            }
            else
            {
                cost[a] = 1;
            }

            capacity[a] = num_workers;
        }
    }

    unordered_map<int,int> edge_flows; //arc id, flow count

    // NetworkSimplex setup
    NetworkSimplex<ListDigraph> ns(g);
    ns.costMap(cost);
    ns.upperMap(capacity);
    ns.supplyMap(supply);
    ns.flowMap(flow); // Use the initial flow (warm start)
    
    if (ns.run() == NetworkSimplex<ListDigraph>::OPTIMAL) 
    {
        int cnt = 0;

        cout << "Optimal assignment with minimum cost:" << endl;
        // Iterate over all worker nodes
        for (int i = 0; i < num_workers; i++) 
        {
            ListDigraph::Node current = map_nodes[env->curr_states[flexible_agent_ids[i]].location];

            list<int> path;

            while (node_to_task_id.find(lemon::ListDigraphBase::id(current)) == node_to_task_id.end()) 
            {
                // Check if the current node is a task node
                if (current == sink) break; // Reached sink, no task node found

                int loc = node_to_maploc[lemon::ListDigraphBase::id(current)];
                path.push_back(loc);

                // Follow the flow to the next node
                // Find the next node in the path
                bool found = false;
                for (ListDigraph::OutArcIt arc(g, current); arc != INVALID; ++arc) 
                {
                    if (ns.flow(arc) > 0) 
                    { // Follow the flow
                        if (edge_flows.find(lemon::ListDigraphBase::id(arc)) == edge_flows.end())
                        {
                            edge_flows[lemon::ListDigraphBase::id(arc)] = ns.flow(arc);
                        }
                        if (edge_flows[lemon::ListDigraphBase::id(arc)] <= 0)
                            continue;
                        current = g.target(arc);
                        edge_flows[lemon::ListDigraphBase::id(arc)]--;
                        found = true;
                        break;
                    }
                }
                if (!found) break;  // No path found
            }
            // Now `current` should be a task node
            if (node_to_task_id.find(lemon::ListDigraphBase::id(current)) != node_to_task_id.end()) 
            {
                int task_loc = node_to_task_id[lemon::ListDigraphBase::id(current)];
                int task_id = task_loc_ids[task_loc].front();
                // node_to_task_id[current].pop_front();
                path.push_back(task_loc);
                // cout << "Worker " << i << " is assigned to Task " << task_id  << " through intermediate nodes." << endl;
                proposed_schedule[flexible_agent_ids[i]] = task_id;
                agent_guide_path[flexible_agent_ids[i]] = path;
                task_loc_ids[task_loc].pop_front();
                if (task_loc_ids[task_loc].empty())
                {
                    task_loc_ids.erase(task_loc);
                    node_to_task_id.erase(lemon::ListDigraphBase::id(current));
                }
            }
            else 
            {
                cout << "No solution found." << endl;
            }
        }
    }
    else 
    {
        cout << "No optimal solution found." << endl;
    }
    // End timing
    auto end_time = std::chrono::high_resolution_clock::now();
    double elapsed_time = std::chrono::duration<double>(end_time - start_time).count();
    cout << "Solving time: " << elapsed_time << " seconds" << endl;

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


bool isTaskNode(ListDigraph::Node node, ListDigraph& g, ListDigraph::Node sink) 
{
    for (ListDigraph::OutArcIt outArc(g, node); outArc != INVALID; ++outArc) {
        ListDigraph::Arc arc = outArc;
        if (g.target(arc) == sink) {
            return true;  // Node is connected to sink (task node)
        }
    }
    return false;
}

unordered_map<int,list<int>> get_guide_path()
{ 
    return agent_guide_path; 
}

};
