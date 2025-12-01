#!/usr/bin/env python3

import os
import yaml
import shutil
from itertools import product
import numpy as np
import shutil
import shlex
import subprocess
import time

def prepare_experiment_folder(experiment_name):
    """
    Ensure the experiment folder exists.
    If dancers_data/<experiment_name> already exists, prompt for deletion.
    """
    exp_dir = os.path.join("dancers_data", experiment_name)

    if os.path.exists(exp_dir):
        answer = input(f"Experiment folder '{exp_dir}' already exists. Delete it? [y/N]: ").strip().lower()
        if answer == "y":
            shutil.rmtree(exp_dir)
            print(f"Deleted existing folder: {exp_dir}")
        else:
            print("Keeping existing folder and appending new configs.")
    else:
        os.makedirs(exp_dir, exist_ok=True)

    return exp_dir

def configs_match_except_seed(cfg1, cfg2):
    """Return True if two configs are identical except for the 'seed' key."""
    c1 = {k: v for k, v in cfg1.items() if k != "seed"}
    c2 = {k: v for k, v in cfg2.items() if k != "seed"}
    return c1 == c2


def generate_config_file(params, instance_id, filename="experiment.yaml"):
    """
    Generate a YAML configuration file for a given parameter dictionary.
    Saves under:
    dancers_data/<experiment_name>/instance_<instance_id>/config/<filename>.yaml
    """
    params["instance_id"] = instance_id
    experiment_name = params.get("experiment_name", "default_exp")

    output_dir = os.path.join(
        "dancers_data",
        experiment_name,
        f"instance_{instance_id}",
        "config"
    )
    os.makedirs(output_dir, exist_ok=True)
    
    # Also prepare the results folder
    os.makedirs(os.path.join(output_dir, "..", "results"), exist_ok=True)

    file_path = os.path.join(output_dir, filename)
    
    # Check for existing config (ignoring seed)
    if os.path.exists(file_path):
        with open(file_path, "r") as f:
            existing_cfg = yaml.safe_load(f)

        if configs_match_except_seed(existing_cfg, params):
            seed = params.get("seed")
            if seed is None:
                raise ValueError("Multiple runs require a 'seed' parameter.")
            file_path = os.path.join(output_dir, f"experiment_{seed}.yaml")

    # Write file
    with open(file_path, "w") as f:
        yaml.dump(params, f, default_flow_style=False)
    return file_path

def generate_parameter_combinations(base_params, sweep_params=None, paired_params=None):
    """
    Generate a list of parameter sets by combining varying parameters.

    - base_params: dict with fixed parameters
    - sweep_params: dict {key: list of values} -> independent Cartesian product
    - paired_params: dict {key: list, key2: list, ...} -> all lists must have same length,
      varied in sync

    Returns: list of dicts (complete parameter sets)
    """
    sweep_params = sweep_params or {}
    paired_params = paired_params or {}

    # --- Handle sweep (independent combinations) ---
    sweep_keys = list(sweep_params.keys())
    sweep_values = list(sweep_params.values())

    sweep_combos = []
    if sweep_keys:
        for combo in product(*sweep_values):
            sweep_combos.append(dict(zip(sweep_keys, combo)))
    else:
        sweep_combos = [{}]  # single empty dict

    # --- Handle paired parameters (must vary together) ---
    paired_keys = list(paired_params.keys())
    paired_values = list(paired_params.values())

    if paired_keys:
        lengths = [len(v) for v in paired_values]
        if len(set(lengths)) > 1:
            raise ValueError("All paired parameter lists must have the same length")

        paired_combos = [
            dict(zip(paired_keys, values)) for values in zip(*paired_values)
        ]
    else:
        paired_combos = [{}]

    # --- Combine sweep and paired combos ---
    all_combos = []
    for s in sweep_combos:
        for p in paired_combos:
            combo = {}
            combo.update(base_params)
            combo.update(s)
            combo.update(p)
            all_combos.append(combo)

    return all_combos

def launch_sim_components_in_tmux(
    config_file,
    network_connector_package_node="",
    physics_connector_package_node="",
    session_id=1,
    commands=None,
    force=False,
    attach=True,
    remain_on_exit=False,
    headless=False
):
    """
    Launch simulator components in a tmux session.

    Args:
        config_file (str): absolute path to the YAML config file.
        session_name (str): tmux session name to create/use.
        commands (list[str] | None): list of command strings to run (3 by default).
        force (bool): if True and the session exists, kill and recreate without prompting.
        attach (bool): whether to attach to the session at the end.
    """
    # check tmux availability
    if shutil.which("tmux") is None:
        raise EnvironmentError("tmux not found in PATH. Please install tmux to use this function.")
    
    session_name = f"dancers_{session_id}"
    
    # safe-quote the config path for shell usage
    quoted_cfg = shlex.quote(config_file)
    
    # default commands (replace config path)
    if commands is None:
        commands = [
        f"ros2 run coordinator coordinator --ros-args -p config_file:={quoted_cfg} -p use_sim_time:=true",
        f"ros2 run {network_connector_package_node} --ros-args -p config_file:={quoted_cfg} -p use_sim_time:=true",
        f"ros2 run {physics_connector_package_node} --ros-args -p config_file:={quoted_cfg} -p use_sim_time:=true",
        ]
        if not headless:
            commands.append(f"ros2 run rviz2 rviz2 -d src/launch/tutorials/rviz_tutorial_2.rviz --ros-args -p use_sim_time:=true")
        
    if len(commands) < 1:
        raise ValueError("You must provide at least one command to run.")
    
    # prepend ROS_DOMAIN_ID to each command
    env_prefix = f"ROS_DOMAIN_ID={session_id}"
    commands = [f"{env_prefix} {cmd}" for cmd in commands]

    # check if session exists
    session_exists = (subprocess.run(["tmux", "has-session", "-t", session_name],
                                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0)
    
    if session_exists:
        if force:
            subprocess.run(["tmux", "kill-session", "-t", session_name], check=True)
            print(f"Killed existing tmux session '{session_name}' (force=True).")
        else:
            ans = input(
                f"tmux session '{session_name}' already exists. "
                "(a)ttach / (k)ill & recreate / (q)uit [a/k/q]? "
            ).strip().lower()
            if ans.startswith("a"):
                print(f"Attaching to existing session '{session_name}'.")
                subprocess.run(["tmux", "attach-session", "-t", session_name])
                return
            elif ans.startswith("k"):
                subprocess.run(["tmux", "kill-session", "-t", session_name], check=True)
                print(f"Killed existing tmux session '{session_name}'.")
            else:
                print("Aborting tmux launcher.")
                return
            
    # create a new session (detached) with the first command
    subprocess.run(
        ["tmux", "new-session", "-d", "-s", session_name, commands[0]],
        check=True,
    )
    print(f"Created tmux session '{session_name}'.")
    
    if remain_on_exit:
        subprocess.run(["tmux", "set-option", "-t", session_name, "remain-on-exit", "on"], check=True)
        print(f"Session '{session_name}' will remain on exit.")
    
    # split panes for the remaining commands
    for cmd in commands[1:]:
        subprocess.run(["tmux", "split-window", "-t", session_name, cmd], check=True)
        subprocess.run(["tmux", "select-layout", "-t", session_name, "tiled"], check=True)
        time.sleep(0.05)

    # finally attach if requested
    if attach:
        print(f"Attaching to tmux session '{session_name}'...")
        subprocess.run(["tmux", "attach-session", "-t", session_name])
    else:
        print(f"Session '{session_name}' created and commands launched (not attached).")

def run_additional_commands_in_tmux(session_id, commands, attach=False):
    """
    Run additional commands inside an existing tmux session as new panes.

    Args:
        session_id (str): Name of the tmux session (e.g., 'dancers_1').
        commands (list[str]): Commands to execute, each in its own new tmux window.
    """
    if shutil.which("tmux") is None:
        raise EnvironmentError("tmux not found in PATH. Please install tmux to use this function.")
    
    session_name = f"dancers_{session_id}"

    # Check session exists
    session_exists = (
        subprocess.run(["tmux", "has-session", "-t", session_name],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0
    )
    if not session_exists:
        raise RuntimeError(f"tmux session '{session_name}' does not exist. Launch it first.")
    
    # prepend ROS_DOMAIN_ID to each command
    env_prefix = f"ROS_DOMAIN_ID={session_id}"
    commands = [f"{env_prefix} {cmd}" for cmd in commands]


    for i, cmd in enumerate(commands, start=1):
        subprocess.run(["tmux", "split-window", "-t", session_name, cmd], check=True)
        subprocess.run(["tmux", "select-layout", "-t", session_name, "tiled"], check=True)

        print(f"Launched extra command in new pane: {cmd}")
    
    # finally attach if requested
    if attach:
        print(f"Attaching to tmux session '{session_name}'...")
        subprocess.run(["tmux", "attach-session", "-t", session_name])


def main():
    # --- User parameters (easy to edit) ---
    base_params = {
        "experiment_name": "tutorial_3",
        "simulation_length": 100,
        "sync_window": 8000,       # in us | the duration between two position exchange and clock synchronization between physics and network simulators
        "phy_step_size": 4000,     # in us | the duration of 1 physics simulator simulation loop
        "net_step_size": 4000,     # in us
        
        "net_use_uds": True,
        "net_uds_server_address": "/tmp/net_server_socket",
        "net_ip_server_address": "127.0.0.1",
        "net_ip_server_port": 10000,

        "phy_use_uds": True,
        "phy_uds_server_address": "/tmp/phy_server_socket",
        "phy_ip_server_address": "127.0.0.1",
        "phy_ip_server_port": 10000,
        
        "robots_number": 4,

        "save_compute_time": False,
        
        "real_time_factor": 2.0
    }
        
    networking_params = {
        "wifi_standard": "802.11ax",
        "wifi_phy_mode": "HeMcs7",
        "short_guard_interval_supported": True,
        
        "waypoint_flow": {
            "enabled": True,
            "interval": 1000000,    # in us
            "port": 4000,
            "flow_id": 1
        },
        "random_waypoint": {
            "new_waypoint_interval": 10000000,   # in us
            "arena_corner_1": {"x": -10.0, "y": -10.0, "z": 10.0},
            "arena_corner_2": {"x": 10.0, "y": 10.0, "z": 50.0},
        }
    }
    
    initial_position_params = {
        "initial_spacing": 5.0,      # in m | spacing between robots at start (square grid)
        "initial_x": 0.0,            # in m | initial x position of the center of the starting grid
        "initial_y": 0.0,            # in m | initial y position of the center of the starting grid
        "initial_z": 1.0,            # in m | initial z position of the starting grid
        "initial_heading": 0.0,      # in rad | initial heading of all robots
    }

    gazebo_connector_params = {
        "world_file": "src/physics_connectors/Gazebo/worlds/default.sdf",
        "robot_model": "x500",
        "path_to_px4_autopilot": f"{os.getenv('HOME')}/PX4-Autopilot-1.16"
    }
    
    base_params.update(networking_params)
    base_params.update(initial_position_params)
    base_params.update(gazebo_connector_params)

    sweep_params = {
    }

    paired_params = {
    }

    # --- Prepare clean experiment folder ---
    prepare_experiment_folder(base_params["experiment_name"])

    configs = generate_parameter_combinations(base_params, sweep_params, paired_params)

    # Add different seeds for repeated runs
    seeds = [42]
    
    configs_paths = []
    instance_id = 1
    for cfg in configs:
        for seed in seeds:
            cfg_with_seed = cfg.copy()
            cfg_with_seed["seed"] = seed
            path = generate_config_file(cfg_with_seed, instance_id=instance_id)
            configs_paths.append(path)
            print(f"Saved config: {path}")
        instance_id += 1
    
    network_package_node = "ns3_connector basic_wifi_adhoc"
    physics_package_node = "gazebo_connector gazebo_connector"
    
    # Launch the first exp
    launch_sim_components_in_tmux(configs_paths[0], network_package_node, physics_package_node, session_id=1, remain_on_exit=True, headless=True, attach=False)
    
    additional_cmds = [
        f"ros2 run px4_control waypoint_control --ros-args -p robot_name:=px4_{i} -p use_sim_time:=true" for i in range(base_params["robots_number"])
    ] + [
        "gz sim -g",
        "MicroXRCEAgent udp4 -p 8888",
        "QGroundControl.AppImage"
    ]
    run_additional_commands_in_tmux(session_id=1, commands=additional_cmds, attach=True)

if __name__ == "__main__":
    main()