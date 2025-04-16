import sys
import yaml
from pathlib import Path
import shutil
from itertools import product
from os import chmod
from copy import deepcopy

def create_or_clear_experience_folder(experience_name):
    config_path = Path("src") / Path("config")
    experience_path = config_path / experience_name

    # Ensure the config folder exists
    config_path.mkdir(exist_ok=True)

    if experience_path.exists():
        prompt = input(f"The folder '{experience_path}' already exists. Do you want to clear its contents? [y/N]: ").strip().lower()
        if prompt == 'y':
            shutil.rmtree(experience_path)
            experience_path.mkdir()
            print(f"Cleared and recreated folder: {experience_path}")
        else:
            print("Operation cancelled. Folder not modified.")
    else:
        experience_path.mkdir()
        print(f"Created folder: {experience_path}")

def deep_merge(dict1, dict2):
    result = dict1.copy()
    for key, value in dict2.items():
        if (
            key in result
            and isinstance(result[key], dict)
            and isinstance(value, dict)
        ):
            result[key] = deep_merge(result[key], value)
        else:
            result[key] = value
    return result

def unflatten_dict(flat_dict):
    result = {}
    for key, value in flat_dict.items():
        parts = key.split(".")
        d = result
        for part in parts[:-1]:
            d = d.setdefault(part, {})
        d[parts[-1]] = value
    return result

def combine_to_dicts(keys, values_lists):
    combinations = product(*values_lists)
    flat_dicts = [dict(zip(keys, combo)) for combo in combinations]
    return [unflatten_dict(d) for d in flat_dicts]

if __name__ == "__main__":
    
    config_file = sys.argv[1]

    with open(config_file, 'r') as file:
        original_config = yaml.safe_load(file)
    
    create_or_clear_experience_folder(original_config['experience_name'])

    # Define the keys that vary, in dotted keys style for nested objects
    keys = ["robots_number", "broadcast_flow.interval"]
    values_lists = [
        [8*x for x in range(1,5)],
        [100000+500000*x for x in range(6)]
    ]

    configs = combine_to_dicts(keys, values_lists)


    for i, config in enumerate(configs):
        print(f"Config {i+1}:")
        print(config)
    
    # Combine with the original yaml config file
    for i, config in enumerate(configs):
        new_config = deep_merge(original_config, config)
        new_config['run_id'] = i+1
        # Create a new YAML file for each configuration
        with open(Path("src") / Path("config") / original_config['experience_name'] / f"config_{i+1}.yaml", 'w') as file:
            yaml.dump(new_config, file)
    