#ifndef SCHEDULER
#define SCHEDULER

#include "Types.h"
#include "SharedEnv.h"
#include "heuristics.h"
#include <random>
#include <thread>
#include <future>
#include <lemon/list_graph.h>
#include <lemon/network_simplex.h>

using namespace lemon;

namespace DefaultPlanner{

void schedule_initialize(int preprocess_time_limit, SharedEnvironment* env);

void schedule_plan_raw(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env);
void schedule_plan_matching(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env, std::vector<Double4> background_flow, bool use_traffic, bool new_only, int maximum_edges);
void schedule_plan_flow(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env, std::vector<Double4> background_flow, bool use_traffic, bool new_only);
void schedule_plan_h(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env, bool new_only);

void schedule_plan_flow_hist(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env, std::vector<pair<double,int>>& background_flow, bool new_only);

unordered_map<int,unordered_map<int,int>> compute_heuristics(SharedEnvironment* env, std::vector<Int4> background_flow, vector<pair<int,int>> current_id_assignment, unordered_map<int,list<int>> task_loc_ids, int max_num_tasks);
void printDIMACS(ListDigraph& g, ListDigraph::Node source, ListDigraph::Node sink, vector<ListDigraph::Node>& workers, vector<ListDigraph::Node>& tasks, ListDigraph::ArcMap<int>& capacity, ListDigraph::ArcMap<double>& cost);
bool isTaskNode(ListDigraph::Node node, ListDigraph& g, ListDigraph::Node sink);

unordered_map<int,list<int>> get_guide_path();

}

#endif