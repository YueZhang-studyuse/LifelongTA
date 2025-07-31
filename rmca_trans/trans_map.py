def parse_map_file(filepath):
    r_locations = {}
    e_locations = {}
    r_count = 0
    e_count = 0

    with open(filepath, 'r') as f:
        lines = f.readlines()

    # Skip first four lines (header)
    map_lines = [line.strip() for line in lines[4:]]
    num_cols = len(map_lines[0])

    for row_idx, line in enumerate(map_lines):
        for col_idx, char in enumerate(line):
            pos = row_idx * num_cols + col_idx
            if char == 'r':
                r_locations[r_count] = pos
                r_count += 1
            elif char == 'e':
                e_locations[e_count] = pos
                e_count += 1

    return r_locations, e_locations


def write_agents_file(r_locations, output_file):
    with open(output_file, 'w') as f:
        f.write("# version for LoRR 2024\n")
        f.write(f"{len(r_locations)}\n")
        for i in range(len(r_locations)):
            f.write(f"{r_locations[i]}\n")


def convert_task_file(e_locations, task_input_file, task_output_file):
    with open(task_input_file, 'r') as f:
        lines = f.readlines()

    num_tasks = int(lines[0].strip())
    task_lines = lines[1:num_tasks+1]

    with open(task_output_file, 'w') as f:
        f.write("# version for LoRR 2024\n")
        f.write(f"{num_tasks}\n")
        for line in task_lines:
            cols = line.strip().split()
            e1_idx = int(cols[1])
            e2_idx = int(cols[2])
            e1_pos = e_locations[e1_idx]
            e2_pos = e_locations[e2_idx]
            f.write(f"{e1_pos},{e2_pos}\n")

# # === Usage ===
# map_file = "kiva-50-500-5.map"
# task_input_file = "kiva-500.task"
# task_output_file = "kiva_500.tasks"

# e_positions = parse_map_for_e(map_file)
# convert_task_file(e_positions, task_input_file, task_output_file)

folders = ["2-500/", "5-500/", "10-500/"]
agents = [50,80,100]

for agent in agents:
    map_file = f"kiva-{agent}-500-5.map"
    agent_output_file = "../instances/agents"+f"kiva_{agent}_agents.agents"
    r_positions, e_positions = parse_map_file(map_file)
    write_agents_file(r_positions, agent_output_file)
    for folder in folders:
        for i in range(0,25):
            task_input_file = f"{folder}{i}.task"
            task_output_file = f"../instances/tasks/{folder}/{i}.tasks"
            convert_task_file(e_positions, task_input_file, task_output_file)