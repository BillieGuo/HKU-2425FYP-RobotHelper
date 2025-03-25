import os
import glob
import yaml
import math
from scipy.spatial.transform import Rotation as R

PI = math.pi

##################################################################################
# system functions
def get_path(fname=None):
    """Find file path from file name.

    Args:
        fname (str, optional): File name to search. Defaults to None.

    Returns:
        str/None: File path if found, None otherwise.
    """
    if not fname:
        return None
    
    start_dir = os.getcwd()
    # print("fname: ",fname, "start_dir: ",start_dir)
    search_pattern = os.path.join(start_dir, '**', fname)  # Pattern for recursive search
    found_files = glob.glob(search_pattern, recursive=True)  # Perform the search
    if not found_files:
        print(f'File not found: {fname}')
        return None
    path = found_files[0]
    return path


def get_config(config_name=None, config_path='./translator/configs/llm_config.yaml'):
    """Load config file from file path / name.
    
    Args:
        config_name (str, optional): Name of config file. Defaults to None.
        config_path (str, optional): Path to config file. Defaults to './translator/configs/llm_config.yaml'.
    
    Returns:
        dict: Config dictionary
    """
    assert config_name or os.path.exists(config_path), f'config file does not exist ({config_name, config_path})'
    
    if config_name:
        config_path = get_path(config_name)
        
    with open(config_path, 'r') as f:
        config = yaml.load(f, Loader=yaml.FullLoader)
    config = config['lmp_config']
    return config

def load_prompt(prompt_fname:str, prompt_fpath=None)->str:
    """Load prompt file from file name.

    Args:
        prompt_fname (str): Name of prompt file.
        prompt_fpath (str, optional): Path to prompt file. Defaults to None.

    Returns:
        str: Content of prompt file.
    """
    if not prompt_fpath:
        prompt_fpath = get_path(prompt_fname)
    assert prompt_fpath, f'prompt file does not exist ({prompt_fname})'

    with open(prompt_fpath, 'r') as f:
        contents = f.read().strip()
    return contents
##################################################################################


##################################################################################
# Math functions
def value_CLIP(value, min_value, max_value):
    return max(min(value, max_value), min_value)

def value_NORM(value, min_value, max_value):
    if value < min_value:
        value += (max_value - min_value)
    if value > max_value:
        value -= (max_value - min_value)
    return value

def angle_calculation(x1,y1,x2,y2):
    dx = x2 - x1
    dy = y2 - y1
    angle = math.atan2(dy, dx)
    angle = value_NORM(angle, -PI, PI)
    return angle

# Convert quaternion to euler angles
def to_euler(quaternion):
    r = R.from_quat(quaternion)
    euler = r.as_euler('xyz', degrees=False)
    return euler

# # Convert euler angles to quaternion
# def to_quaternion(euler):
#     r = R.from_euler('xyz', euler, degrees=True)
#     quaternion = r.as_quat()
#     return list(quaternion)

##################################################################################


# test
if __name__ == '__main__':
    cfg = get_config(config_name="config.yaml")
    print("="*80)
    print(get_path("llama3-8B-instruct-official-fineTuned"))
    print(type(get_path("llama3-8B-instruct-official-fineTuned")))
    print("="*80)
    print(load_prompt("coder_sys_prompt.txt"))    