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
		self.master = None 
		self.navigator = None 
		self.arm = None 
		self.model_init()
		pass
        
	def model_init(self):
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
		self.master = LMP("master", cfg['master'], fixed_vars, variable_vars)
		self.navigator = LMP("navigator", cfg['navigator'])
		self.arm = LMP("arm", cfg['arm'])
		# previewer = None # 

		self.get_logger().info(f'Models loaded!')
		return

	def run(self):
		while rclpy.ok:
			input_prompt = None
			input_prompt = input("\n\033[1;33m>> Prompt: ")
			print("\033[0m")
			rclpy.spin_once(self, timeout_sec=0.1)

			continue
		pass

def main():
    rclpy.init()
    node = QUERY()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    return
    
if __name__ == '__main__':
    main()