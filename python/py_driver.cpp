#include "pyMAPFPlanner.hpp"

#include <pybind11/embed.h>

#include "CompetitionSystem.h"
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/tokenizer.hpp>
#include "nlohmann/json.hpp"
#include <signal.h>
#include "Evaluation.h"
#include <cuda_runtime_api.h>  // Add this header for CUDA initialization, comment this if cuda is not used

namespace po = boost::program_options;
using json = nlohmann::json;
po::variables_map vm;
BaseSystem *system_ptr = nullptr;

void sigint_handler(int a)
{
    fprintf(stdout, "stop the simulation...\n");
    if (!vm["evaluationMode"].as<bool>())
    {
        system_ptr->saveResults(vm["output"].as<std::string>());
    }

    _exit(0);
}

void python_driver(int argc, char **argv)
{
    // cudaFree(0);
    // pybind11::scoped_interpreter guard{};
    pybind11::initialize_interpreter();

    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        // ("inputFolder", po::value<std::string>()->default_value("."), "input folder")
        ("inputFile,i", po::value<std::string>()->required(), "input file name")
        ("output,o", po::value<std::string>()->default_value("./exp/test.json"), "output file name")
        ("evaluationMode", po::value<bool>()->default_value(false), "evaluate an existing output file")
        ("simulationTime", po::value<int>()->default_value(5000), "run simulation")
        ("fileStoragePath", po::value<std::string>()->default_value(""), "the path to the storage path")
        ("planTimeLimit", po::value<int>()->default_value(INT_MAX), "the time limit for planner in seconds")
        ("preprocessTimeLimit", po::value<int>()->default_value(INT_MAX), "the time limit for preprocessing in seconds")
        ("logFile,l", po::value<std::string>(), "issue log file name")
        ;
    clock_t start_time = clock();
    po::store(po::parse_command_line(argc, argv, desc), vm);

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return;
    }

    po::notify(vm);

    // std::string base_folder = vm["inputFolder"].as<std::string>();
    boost::filesystem::path p(vm["inputFile"].as<std::string>());
    boost::filesystem::path dir = p.parent_path();
    std::string base_folder = dir.string();
    std::cout << base_folder << std::endl;
    if (base_folder.size() > 0 && base_folder.back()!='/'){
        base_folder += "/";
    }

    Logger* logger = new Logger();
    if (vm.count("logFile"))
        logger->set_logfile(vm["logFile"].as<std::string>());


    DummyPlanner dummy;
    MAPFPlanner competition;
    MAPFPlanner* planner = nullptr;

    if (vm["evaluationMode"].as<bool>()){
        logger->log_info("running the evaluation mode");
        dummy.load_plans(vm["output"].as<std::string>());
        planner = &dummy;
    }else{
        planner = &competition;
    }

    auto input_json_file = vm["inputFile"].as<std::string>();
    json data;
    std::ifstream f(input_json_file);
    try{
        data = json::parse(f);
    }
    catch(json::parse_error error ) {
        std::cerr << "Failed to load " << input_json_file << std::endl;
        std::cerr << "Message: " << error.what() << std::endl;
        exit(1);
    }

    auto map_path = read_param_json<std::string>(data, "mapFile");
    Grid grid(base_folder + map_path);

    planner->env->map_name = map_path.substr(map_path.find_last_of("/") + 1);
    planner->env->file_storage_path = vm["fileStoragePath"].as<std::string>();


    ActionModelWithRotate* model = new ActionModelWithRotate(grid);
    model->set_logger(logger);

    int team_size = read_param_json<int>(data, "teamSize");

    std::vector<int> agents = read_int_vec(base_folder + read_param_json<std::string>(data, "agentFile"),team_size);
    std::vector<int> tasks = read_int_vec(base_folder + read_param_json<std::string>(data, "taskFile"));
    std::cout << agents.size() << " agents and " << tasks.size() << " tasks"<< std::endl;

    std::string task_assignment_strategy = data["taskAssignmentStrategy"].get<std::string>();
    if (task_assignment_strategy=="greedy"){
        system_ptr = new TaskAssignSystem(grid, planner, agents, tasks, model);
    } else if (task_assignment_strategy=="roundrobin"){
        system_ptr = new InfAssignSystem(grid, planner, agents, tasks, model);
    }
    else if (task_assignment_strategy=="roundrobin_fixed"){
        std::vector<vector<int>> assigned_tasks(agents.size());
        for(int i = 0; i < tasks.size(); i++){
            assigned_tasks[i%agents.size()].push_back(tasks[i]);
        }
        system_ptr = new FixedAssignSystem(grid, planner, agents, assigned_tasks, model);
    } else{
        std::cerr << "unkown task assignment strategy " << data["taskAssignmentStrategy"].get<std::string>() << std::endl;
        logger->log_fatal("unkown task assignment strategy " + data["taskAssignmentStrategy"].get<std::string>());
        exit(1);
    }

    system_ptr->set_logger(logger);
    system_ptr->set_plan_time_limit(vm["planTimeLimit"].as<int>());
    system_ptr->set_preprocess_time_limit(vm["preprocessTimeLimit"].as<int>());

    system_ptr->set_num_tasks_reveal(read_param_json<int>(data, "numTasksReveal", 1));

    signal(SIGINT, sigint_handler);

    system_ptr->simulate(vm["simulationTime"].as<int>());

    if (!vm["evaluationMode"].as<bool>()){
        system_ptr->saveResults(vm["output"].as<std::string>());
    }

    delete model;
    delete planner->env;
    // delete planner;
    delete system_ptr;
    // std::cout<<"?????"<<std::endl;
    // return 0;
    // return 0;
}

void test()
{
    pybind11::scoped_interpreter guard{};
    MAPFPlanner *planner = new pyMAPFPlanner();
    std::vector<Action> plan;
    planner->plan(666, plan);
}

int main(int argc, char **argv)
{
    python_driver(argc, argv);
    return 0;
}
