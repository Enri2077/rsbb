#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
#from rockin_scoring.BmBox3 import BmBox
from rockin_scoring.BenchmarkObject import BaseBenchmarkObject

import time
import yaml


class BenchmarkObject (BaseBenchmarkObject):
	
#	def get_benchmark_code(self, hello): return "STB" ### test
#	def get_benchmark_code(self): return 5 ### test
	
	def get_benchmark_code(self): return "STB"
		
#	def execute_(self): ### test
#	def execute(self, hello): ### test
	def execute(self):
		
		BENCHMARK_RUNS = 2
		
		rospy.loginfo("simple_test_benchmark started")
		
		# Variables to compute score
		runs = 0
		execution_time = 0.0
		
		print "Waiting for Client"
		
		# Wait for client
		self.WaitClient()
		
		print "Client connected"
		
		# Start the benchmark
		while self.isBenchmarkRunning() and (runs < BENCHMARK_RUNS):
			runs = runs + 1
			
			now = rospy.Time.now()
			
			##########################################
			#            GOAL                        #
			##########################################
			
			goal_object = {"goal": "GOAL %d"%runs, "details": 4}
			goal_yaml_string = yaml.dump(goal_object)
			
			print "yaml object from string", yaml.load(goal_yaml_string)["goal"]
			
			# Send goal
			print "Sending goal: %s" % goal_yaml_string
			self.SendGoal(goal_yaml_string, 100)
#			self.SendGoal() #TODO test this one
			start_time = time.time()
			
			# Wait for result from client
			result_yaml = self.WaitResult()
			end_time = time.time()
			
			print "Received result_yaml:", result_yaml
			
			
			
			########### TEST:
#			x = 1/0
			
			
			if self.isGoalTimedout():
				print "GOAL TIMEOUT"
				execution_time = execution_time + (end_time - start_time)
				continue
			
			if self.isBenchmarkEnded():
				print "BENCHMARK ENDED"
				print self.getEndReason()
				break
			
			if result_yaml:
				result = yaml.load(result_yaml)
			
			# Evaluate execution time
			execution_time = execution_time + (end_time - start_time)
			
			rospy.loginfo("Execution time - %f" % execution_time)
		
		
		if not self.isBenchmarkEnded():
			print "Starting final Manual Operation"
			manual_operation_result = self.ManualOperation("Notes from the referee:")
			print "Finished final Manual Operation: %s" % manual_operation_result
			
			# Evaluate final score
			score = {
				'score_1': "Some Score",
				'score_2': "Some Score",
				'execution_time': execution_time
			}
			
			self.set_current_score(score)
			
			print "Score:", score
			score_yaml = yaml.dump(score)
			self.SendScore(score_yaml)
		
			# Benchmark concluded
#			self.End()
		
		else:
			print "Benchmark ended!!!!"

