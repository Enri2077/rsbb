#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from rockin_scoring.BmBox3 import BmBox
from rockin_scoring.BenchmarkObject import BenchmarkObject

import time
import yaml

script_object = BenchmarkObject("STB")

def execute():
	
	BENCHMARK_RUNS = 2
	
	# Init benchmarking node
	benchmark = BmBox()
	rospy.loginfo("simple_test_benchmark benchmarking node started")
	
	# Variables to compute score
	runs = 0
	execution_time = 0.0
	
	print "Waiting for Client"
	
	# Wait for client
	benchmark.WaitClient()
	
	print "Client connected"

	# Start the benchmark
	while benchmark.isBenchmarkRunning() and (runs < BENCHMARK_RUNS):
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
		benchmark.SendGoal(goal_yaml_string, 100)
#		benchmark.SendGoal() #TODO test this one
		start_time = time.time()
		
		# Wait for result from client
		result_yaml = benchmark.WaitResult()
		end_time = time.time()
		
		print "Received result_yaml:", result_yaml
		
		if benchmark.isGoalTimedout():
			print "GOAL TIMEOUT"
			execution_time = execution_time + (end_time - start_time)
			continue
		
		if benchmark.isBenchmarkEnded():
			print "BENCHMARK ENDED"
			print benchmark.getEndReason()
			break
		
		if result_yaml:
			result = yaml.load(result_yaml)
		
		# Evaluate execution time
		execution_time = execution_time + (end_time - start_time)

		rospy.loginfo("Execution time - %f" % execution_time)
	
	
	if not benchmark.isBenchmarkEnded():
		print "Starting final Manual Operation"
		manual_operation_result = benchmark.ManualOperation("Notes from the referee:")
		print "Finished final Manual Operation: %s" % manual_operation_result
	
		# Evaluate final score
		score = {
			'score': "Some Score",
			'execution_time': execution_time
		}
	
		print "Score:", score
		score_yaml = yaml.dump(score)
		benchmark.SendScore(score_yaml)
	
	else:
		print "Benchmark ended!!!!"
	
	# Benchmark concluded
	benchmark.End()
	

script_object.execute = execute
