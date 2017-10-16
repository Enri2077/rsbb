#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, yaml, time # TODO use rospy.time

from rockin_scoring.BenchmarkObjects import BaseBenchmarkObject, GoalObject, ManualOperationObject


class BenchmarkObject (BaseBenchmarkObject):
	
#	def get_benchmark_code(self, hello): return "STB" ### test
#	def get_benchmark_code(self): return 5 ### test
	
	def get_benchmark_code(self): return "STB"
		
#	def execute_(self): ### test
#	def execute(self, hello): ### test
	def execute(self):
		
		print self._is_waiting_to_start()
		
		BENCHMARK_RUNS = 2
		
		rospy.loginfo("simple_test_benchmark started")
		
		# Variables to compute score
		runs = 0
		execution_time = 0.0
		
		# Start the benchmark
		while self.is_benchmark_running() and (runs < BENCHMARK_RUNS):
			runs = runs + 1
			
			now = rospy.Time.now()
			
			##########################################
			#            GOAL                        #
			##########################################
			
#			goal_object = 
#			goal_yaml_string = yaml.dump(goal_object)
#			print "yaml object from string", yaml.load(goal_yaml_string)["goal"]
			
			goal = GoalObject({"goal": "GOAL %d"%runs, "details": 4}, 15.0)
			
			# Send goal
			self.request_goal(goal)
#			self.request_goal() #TODO test this one
			start_time = time.time()
			
			# Wait for result from client
#			result_yaml = self.wait_goal_result()
			self.wait_goal_result()
			end_time = time.time()
			
			print "Goal result received:", goal.get_result()
			
			execution_time = execution_time + (end_time - start_time)
			
			if goal.has_timed_out():
				print "GOAL TIMEOUT"
				continue
			else:
				result = goal.get_result()
			
			if self.is_goal_timedout():
				print "GOAL TIMEOUT"
				rospy.logerr("TIMEOUT from state and not from GoalObject")
				execution_time = execution_time + (end_time - start_time)
				continue
			
			if self.is_benchmark_ended():
				print "BENCHMARK ENDED"
				print self.get_end_reason()
				break
			
			
			# Evaluate execution time
			execution_time = execution_time + (end_time - start_time)
			
			rospy.loginfo("Execution time - %f" % execution_time)
			
#			self.end_benchmark()
		
		
		if not self.is_benchmark_ended():
			print "Starting final Manual Operation"
			
			manual_operation = ManualOperationObject("Notes from the referee:")
			
			self.manual_operation(manual_operation)
			
			print "Finished final Manual Operation: %s" % manual_operation.get_result()
			
			# Evaluate final score
			score = {
				'score_1': "Some Score",
				'score_2': "Some Score",
				'execution_time': execution_time
			}
			
			self.set_current_score(score)
			
		
		else:
			print "Benchmark ended!!!!"

