
from exceptions import NotImplementedError

class BenchmarkCodeNotImplementedError (NotImplementedError):
    def __init__(self, value):
        self.parameter = value
class ExecuteMethodNotImplementedError (NotImplementedError):
    def __init__(self, value):
        self.parameter = value

class BaseBenchmarkObject:
	def __init__(self):
		self.current_score = {}
		
#	def get_benchmark_code(self, asdasd = "HI"):
#		print "Benchmark code was not defined in BenchmarkObject"
#		raise BenchmarkCodeNotImplementedError("")
	
	def get_current_score(self):
		return self.current_score
	
	def set_current_score(self, current_score):
		self.current_score = current_score
	
	def execute(self):
		raise ExecuteMethodNotImplementedError("")
