import time
import numpy
import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy

from translator.utils import get_config, safe_to_run
from translator.LMP import LMP


class Master(Node):
	def __init__(self):
		super().__init__("Master")
		self.master = None 
		self.navigator = None 
		self.arm = None 
		self.socket_update = False
		self.nagivation_action_done = False
		self.model_init()
  
		qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
		self.llm2arm = self.create_publisher(
			String,
			'grasp_prompt',
			qos_profile)
		self.llm2navigator = self.create_publisher(
			String,
			'llm2navigator',
			qos_profile)
		self.llm2socket = self.create_publisher(
			String,
			'llm2socket',
			qos_profile)

		self.navigator2llm = self.create_subscription(
			String,
			'navigator2llm',
			self.navigator_response_callback,
			qos_profile)
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
		self.socket2llm = self.create_subscription(
			String,
			'socket2llm',
			self.socket_query_callback,
			10)
  
		self.get_logger().info("Master node initialized.")
		pass
        
	def navigator_response_callback(self, msg):
		if msg.data:
			self.get_logger().info(f'navigator response: {msg.data}')
			self.nagivation_action_done = bool(msg.data)
		pass

	def master_response_callback(self, msg):
		pass

	def socket_query_callback(self, msg):
		self.get_logger().info(f'{msg}')
		if not self.socket_update:
			self.socket_update = True
			self.incoming_query = msg.data
		pass

	def response2socket(self, response):
		msg = String()
		msg.data = str(response)
		self.llm2socket.publish(msg)
        
	def model_init(self):
		cfg = get_config('translator/configs/llm_config.yaml')['lmps']
  		# for third libraries that LLM can access
		fixed_vars = {'numpy':numpy, 'subprocess':subprocess, 'time': time} 
  		# for first party libraries (can be other LLM) that a LLM can access
		variable_vars = {} 
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
		self.navigator = LMP("navigator", cfg['navigator'], fixed_vars)
		self.arm = LMP("arm", cfg['arm'], fixed_vars)
		previewer = None # 

		self.get_logger().info(f'Models loaded!')
		return

	def single_cmd_exec(self, llm_delegator, cmd):
		gvars = llm_delegator.fixed_vars | self.variable_vars
		lvars = {}
		code = llm_delegator.code_formatting(cmd)
		success = safe_to_run(code, gvars, lvars)
		# if success: #and self.cfg['save_output']:
		#     print(lvars['result'])
		return success
	
	def get_plan(self, input_prompt):
		model_input = f'Query: {input_prompt}'
		result, success = self.master(model_input) # result should be a list of actions (strings)
		if isinstance(result, str):
			results = result.splitlines()
		return results

	def run(self):
		stopping_vocab_list = ['exit', 'stop', 'quit', 'terminate', 'end']
		while rclpy.ok:
			input_prompt = None
			rclpy.spin_once(self, timeout_sec=0.1)

			if not self.socket_update:
				continue
			self.socket_update = False
   
			# prompt handling
			input_prompt = self.incoming_query
			if not input_prompt:
				continue
			if input_prompt == 'exit':
				break
			if str(input_prompt).lower() in stopping_vocab_list:
				# query.publish_cmd("stop")
				continue
			
			# plans = self.get_plan(input_prompt)
			plans = [f'navigator({input_prompt})', f'arm({input_prompt})'] # hard-coded for now
			# list of action handling
			for action in plans:
				print(f"Executing action: {action}")
				if "navigator" in action:
					# self.navigator(action.split("(")[1].strip(")")) # wait for llm to be implemented
					tx  = String()
					tx.data = action.split("(")[1].strip(")")
					self.llm2navigator.publish(tx)
					while not self.nagivation_action_done:
						rclpy.spin_once(self, timeout_sec=0.1)
						continue
				elif "arm" in action:
					# self.arm(action.split("(")[1].strip(")")) # wait for llm to be implemented
					tx  = String()
					tx.data = action.split("(")[1].strip(")")
					self.llm2arm.publish(tx)
					# wait for arm process to be completed
				else:
					print(f"Unknown action: {action}")

			self.response2socket(plans)
			self.get_logger().info(f'All executed.')
			continue
		pass

def main():
    rclpy.init()
    node = Master()
    node.run()
    node.destroy_node()
    rclpy.shutdown()
    return
    
if __name__ == '__main__':
    main()