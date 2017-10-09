#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from rockin_scoring.BmBox3 import BmBox
from rockin_scoring.BenchmarkObject import BenchmarkObject

import time
import yaml

script_object = BenchmarkObject("LSTB")

def execute():
	
	BENCHMARK_RUNS = 2
	
	# Init benchmarking node
	benchmark = BmBox()
	rospy.loginfo("long_simple_test_benchmark benchmarking node started")
	
	# Variables to compute score
	runs = 0
	execution_time = 0.0
	
	print "Waiting for Client"
	
	# Wait for client
	benchmark.WaitClient()
	
	print "Client connected"

	# Start the benchmark
	while benchmark.Running() and (runs < BENCHMARK_RUNS):
		runs = runs + 1

		##########################################
		#            MANUAL OPERATION A          #
		##########################################
		
		print "Starting Manual Operation %d.1" % (runs)
		manual_operation_result = benchmark.ManualOperation("Manual Operation %d.1" % (runs))
		print "Finished Manual Operation %d.1: %s" % (runs, manual_operation_result)
		
		if not benchmark.Running():
			print "NOT RUNNING"
			break
		
		if benchmark.isBenchmarkEnded():
			print "BENCHMARK ENDED"
			break
	
		##########################################
		#            MANUAL OPERATION B          #
		##########################################
		
#		raw_input("press ENTER to start the second MO")
		print "Starting Manual Operation %d.2" % (runs)
		manual_operation_result = benchmark.ManualOperation("Manual Operation %d.2" % (runs))
		print "Finished Manual Operation %d.2: %s" % (runs, manual_operation_result)

		if not benchmark.Running():
			print "NOT RUNNING"
			break

		if benchmark.isBenchmarkEnded():
			print "BENCHMARK ENDED"
			break
		
		now = rospy.Time.now()
		
		##########################################
		#            GOAL A                      #
		##########################################
		
		goal_object = {"goal": "GOAL %d"%runs, "details": 4}
		goal_yaml_string = yaml.dump(goal_object)
		
		print "yaml object from string", yaml.load(goal_yaml_string)["goal"]
		
		# Send goal
		print "Sending goal: %s" % goal_yaml_string
		benchmark.SendGoal(goal_yaml_string, 10)
#		benchmark.SendGoal() #TODO test this one
		start_time = time.time()
		
		# Wait for result from client
		result_yaml = benchmark.WaitResult()
		end_time = time.time()
		
		print "Received result_yaml:", result_yaml
		
		if benchmark.Timeout():
			print "TIMEOUT"
			execution_time = execution_time + (end_time - start_time)
			continue

		if not benchmark.Running():
			print "NOT RUNNING"
			continue
		
		if benchmark.isBenchmarkEnded():
			print "BENCHMARK ENDED"
			break
		
		if result_yaml:
			result = yaml.load(result_yaml)
		
		# Evaluate execution time
		execution_time = execution_time + (end_time - start_time)

		rospy.loginfo("Execution time - %f" % execution_time)
	
		##########################################
		#            GOAL B                      #
		##########################################
		
		goal_object = {"goal": "GOAL %d"%runs, "details": 4}
		goal_yaml_string = yaml.dump(goal_object)
		
		print "yaml object from string", yaml.load(goal_yaml_string)["goal"]
		
		# Send goal
		print "Sending goal: %s" % goal_yaml_string
		benchmark.SendGoal(goal_yaml_string, 10)
#		benchmark.SendGoal() #TODO test this one
		start_time = time.time()
		
		# Wait for result from client
		result_yaml = benchmark.WaitResult()
		end_time = time.time()
		
		print "Received result_yaml:", result_yaml
		
		if benchmark.Timeout():
			print "TIMEOUT"
			execution_time = execution_time + (end_time - start_time)
			continue

		if not benchmark.Running():
			print "NOT RUNNING"
			continue
		
		if benchmark.isBenchmarkEnded():
			print "BENCHMARK ENDED"
			break
		
		if result_yaml:
			result = yaml.load(result_yaml)
		
		# Evaluate execution time
		execution_time = execution_time + (end_time - start_time)

		rospy.loginfo("Execution time - %f" % execution_time)
	
	
	print "Starting final Manual Operation"
	manual_operation_result = benchmark.ManualOperation("Notes from the referee:")
	print "Finished final Manual Operation: %s" % manual_operation_result
	
	# Evaluate final score
	score = {
		'score': "Some Score",
		'execution_time': execution_time
	}
	
	if not benchmark.isBenchmarkEnded():
		print "Score:", score
		score_yaml = yaml.dump(score)
		benchmark.SendScore(score_yaml)
	
	# Benchmark concluded
	benchmark.End()
	

script_object.execute = execute
