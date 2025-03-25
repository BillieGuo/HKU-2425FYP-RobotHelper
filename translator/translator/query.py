import time
import numpy
import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from translator.utils import get_config
from translator.LMP import LMP


class QUERY(Node):
	def __init__(self):
		super().__init__("Query")
		self.master = None 
		self.navigator = None 
		self.arm = None 
		self.model_init()
		self.publishers_arm = self.create_publisher(
			String,
			'grasp_prompt',
			10)
		self.publishers_navigator = self.create_publisher(
			String,
			'navigator_prompt',
			10)
		self.subscriber_navigator = self.create_subscription(
			String,
			'navigator_response',
			self.navigator_response_callback,
			10)
		self.subscriber_master_text = self.create_subscription(
			String,
			'text_prompt',
			self.master_response_callback,
			10)
		self.subscriber_master_voice = self.create_subscription(
			String,
			'voice_prompt',
			self.master_response_callback,
			10)
		pass
        
	def navigator_response_callback(self, msg):
		pass

	def master_response_callback(self, msg):
		pass
        
	def model_init(self):
		cfg = get_config('translator/configs/llm_config.yaml')['lmps']
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
		stopping_vocab_list = ['exit', 'stop', 'quit', 'terminate', 'end']
		while rclpy.ok:
			input_prompt = None
			input_prompt = input("\n\033[1;33m>> Prompt: ")
			print("\033[0m")
			rclpy.spin_once(self, timeout_sec=0.1)

			# prompt handling
			if not input_prompt:
				continue
			if input_prompt == 'exit':
				break
			if str(input_prompt).lower() in stopping_vocab_list:
				# query.publish_cmd("stop")
				continue
			model_input = f'Query: {input_prompt}'
			result, success = self.master(model_input) # result should be a list of actions (strings)
			continue
		pass

def main():
    rclpy.init()
    node = QUERY()
    node.run()
    node.destroy_node()
    rclpy.shutdown()
    return
    
if __name__ == '__main__':
    main()