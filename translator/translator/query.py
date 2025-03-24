import time
import numpy
import subprocess
import rclpy
from rclpy import Node

from translator.utils import get_config
from translator.LMP import LMP


class QUERY(Node):
    def __init__(self):
        super.__init__('Query')
        
        

def model_init():
	cfg = get_config('capllm/configs/config.yaml')['lmps']
	fixed_vars = {'numpy':numpy, 'subprocess':subprocess, 'time': time} # for third libraries that LLM can access
	variable_vars = {} # for first party libraries (can be other LLM) that a LLM can access
	# allow LMPs to access other LMPs
	# & low-level LLM setup
	lmp_names = [name for name in cfg.keys() if not name in ['coder','previewer']] # cfg=lmps_config
	low_level_lmps = {
		k: LMP(k, cfg[k], fixed_vars, variable_vars) #, debug, env_name)
		for k in lmp_names
	}
	variable_vars.update(low_level_lmps)

	# high-level LLM setup
	coder = LMP("coder", cfg['coder'], fixed_vars, variable_vars)
	previewer = LMP("previewer", cfg['previewer'])
	# previewer = None # 

	return previewer, coder


def main():
    rclpy.init()
    
    pass
    
if __name__ == '__main__':
    main()