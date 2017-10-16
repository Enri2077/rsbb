from os import path, makedirs, rename
from datetime import date, datetime
from exceptions import NotImplementedError
import errno, yaml, platform

import rospy
from rockin_scoring.BmBox import BmBox

class BenchmarkCodeNotImplementedError (NotImplementedError):
	def __init__(self, value):
		self.parameter = value
class ExecuteMethodNotImplementedError (NotImplementedError):
	def __init__(self, value):
		self.parameter = value

def trim(s):
	return s.strip()

def is_valid_dir_path(p):
	return path.exists(p) and path.isdir(p)

def check_and_make_dir(p):
	
	try:
		makedirs(p)
		
	except OSError as e:
	
		if e.errno != errno.EEXIST:
			raise
		else:
		
			if path.isdir(p):
				return True
			elif path.isfile(p):
				
				rospy.logwarn("Renaming file [%s] to create directory"%(p))
				rename(p, p + "_renamed_file")
				
				try:
					makedirs(p)
				except OSError as e:
					rospy.logerr("Directory [%s] could not be created. Please, manually create the directory."%(p))
					raise
				
				return True
				
			else:
				rospy.logerr("Directory [%s] could not be created. Please, manually create the directory."%(p))
				raise
	
	return True


class BaseBenchmarkObject (BmBox):
	
	def __init__(self):
		
#		BmBox.__init__(self)
		
		self._score_object = {'benchmark_info': {'team': "undefined", 'run': 0, 'benchmark_code': "undefined"}, 'score': {}}
		
		try: self._score_base_path = rospy.get_param("base_score_directory")
		except KeyError:
			rospy.logerr("parameter base_score_directory not set in the configuration")
			raise
	
	
	def init_score_file(self):
				
		### Create directories and initialise the score file with the benchmark info
		
		# base path
		base_path = path.normpath(path.expanduser(self._score_base_path))
		print base_path
		
		# check base path
		if not is_valid_dir_path(base_path):
			rospy.logerr("Base directory [%s] does not exist. Please update the base_directory parameter with the directory where the score should be saved"%(base_path))
			# self._current_system_status.status = SystemStatus.ERROR
			# publish_system_status("restarting");
			# TODO restart_node()
			raise Exception
		
		# score directory and path
		#   note that this directory is necessary to ensure that the logs and the score files are saved in different directories,
		#   avoiding the race condition on the check and creation of directories
		score_base_dir = "score"
		score_base_path = path.join(base_path, score_base_dir)
		print score_base_path
		
		# benchmark directory and path
		bm_dir = trim(self.get_benchmark_code())
		bm_path = path.join(score_base_path, bm_dir)
		print bm_path
	
		# check benchmark path
		if not check_and_make_dir(bm_path):
			# self._current_system_status.status = SystemStatus.ERROR
			# publish_system_status("restarting");
			# TODO restart_node()
			raise Exception
		
		# team directory and path
		team_dir = trim(self.get_benchmark_team())
		team_path = path.join(bm_path, team_dir)
		print team_path
	
		# check team path
		if not check_and_make_dir(team_path):
			# self._current_system_status.status = SystemStatus.ERROR
			# publish_system_status("restarting");
			# TODO restart_node()
			raise Exception
		
		# score filename and path of the score file
		score_filename = "score_run_%i_%s.yaml" % (self.get_benchmark_run(), datetime.now().strftime("%Y-%m-%d_%H:%M:%S"))
		self._score_filename_path = path.join(team_path, score_filename)
		print "score_filename_path:", self._score_filename_path
		
		with open(self._score_filename_path, 'w') as outfile:
			yaml.dump(self._score_object, outfile, default_flow_style=False)
	
	
	def set_current_score(self, current_score):
		self._score_object['score'] = current_score
		
		#TODO overwrite file?
		with open(self._score_filename_path, 'w') as outfile:
			yaml.dump(self._score_object, outfile, default_flow_style=False)
	
	def get_current_score(self):
		return self._score_object['score']
	
	def get_benchmark_run(self):
		return self._score_object['benchmark_info']['run']
	
	def get_benchmark_team(self):
		return self._score_object['benchmark_info']['team']

	
	def setup(self, team, run):
		
		BmBox.__init__(self)

		### insert the benchmark informations in the yaml score object
		self._score_object['benchmark_info']['benchmark_code'] = self.get_benchmark_code()
		self._score_object['benchmark_info']['team'] = team
		self._score_object['benchmark_info']['run'] = run
		print "init self._score_object:", self._score_object
		
		self.init_score_file()
	
	
	def wrapped_execute(self):
		print "wrapped_execute"
		
		# Wait for refbox
		print "Waiting for RefBox"
		self._wait_refbox_connection()
		print "RefBox is ready"
		
		self.execute()
		
		self.end_benchmark()
		
	
	def execute(self):
		raise ExecuteMethodNotImplementedError("")
		
#	def get_benchmark_code(self, asdasd = "HI"):
#		print "Benchmark code was not defined in BenchmarkObject"
#		raise BenchmarkCodeNotImplementedError("")


class GoalObject:
	def __init__(self, request = None, timeout = 0):
		self._request = request
		self._timeout = timeout
		self._executed = False
		self._result  = None
		self._has_timed_out = None
	
	def get_request_string(self):
		return yaml.dump(self._request)
	
	def get_timeout(self):
		return self._timeout
	
	def has_been_executed(self):
		return self._executed
	
	def has_timed_out(self):
		return self._has_timed_out
	
	def set_has_timed_out(self, has_timed_out):
		self._has_timed_out = has_timed_out
	
	def get_result(self):
		if self._executed:
			return self._result
		else:
			return None # TODO or raise Exception?
	
	def set_result_string(self, result_yaml_string):
		self._result = yaml.load(result_yaml_string)
		self._executed = True


class ManualOperationObject:
	def __init__(self, request):
		self._request = request
		self._executed = False
		self._result  = None
	
	def get_request(self):
		return self._request
	
	def has_been_executed(self):
		return self._executed
	
	def get_result(self):
		if self._executed:
			return self._result
		else:
			return None # TODO or raise Exception?
	
	def set_result(self, result):
		self._result = result
		self._executed = True
