#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, yaml, tf, math
from geometry_msgs.msg import Pose2D
from os import path

from rsbb_bmbox.BenchmarkObjects import BaseBenchmarkObject, GoalObject, ManualOperationObject



class BenchmarkObject (BaseBenchmarkObject):
	
	benchmark_code = "HNF"
	
	def execute(self):
		
		start_benchmark_time = rospy.Time.now()
		
		print "params:\n", self.params
		
		# init tf and subscribers
		tf_listener = tf.TransformListener()
		tf_broadcaster = tf.TransformBroadcaster()
		
		# Get the parameters of the benchmark (waypoints, penalty_time, timeout_time)
		starting_robot_pose = Pose2D(*self.params["starting_pose"])
		waypoints = self.params["waypoints"]
		goal_timeout = self.params["goal_timeout"]
		
		# Variables to compute score
		N = len(waypoints)
		i = 0
		sum_d = 0.0 # [m]
		sum_sin = 0.0
		sum_cos = 0.0
		execution_time = rospy.Duration(0.0) # [s]
		timed_out_waypoints = 0
		reached_waypoints = 0
		
		# acquire marker-robot transform from config
		team_name = self.get_benchmark_team()
		transform_path = path.join( path.join(rospy.get_param("~resources_directory"), "transforms" ), "transform-%s.yaml" % team_name )
		with open(transform_path, 'r') as transform_file:
			marker_to_robot_transform = yaml.load(transform_file)["marker_to_robot_transform"]
		print "team_name: ", team_name, "\ttransform_path: ", transform_path, "\tmarker_to_robot_transform: ", marker_to_robot_transform
		
		
		while self.is_benchmark_running() and i < N:
			
			self.score["segment_%i"%(i+1)] = {}
			
			
			##########################################
			#                 GOAL i                 #
			##########################################
			
			goal = GoalObject(waypoints[i], goal_timeout)
			
			print "request_goal"
			self.request_goal(goal)
			
			start_segment_time = rospy.Time.now()
			self.score["segment_%i"%(i+1)]["start_segment_time[s]"] = (start_segment_time - start_benchmark_time).to_sec()
			self.save_and_publish_score()
			
			print "wait_goal_result"
			self.wait_goal_result()
			
			end_segment_time = rospy.Time.now()
			self.score["segment_%i"%(i+1)]["end_segment_time[s]"] = (end_segment_time - start_benchmark_time).to_sec()
			self.save_and_publish_score()
			
			segment_time = end_segment_time - start_segment_time
			execution_time += segment_time
			
			self.score["segment_%i"%(i+1)]["segment_time[s]"] = segment_time.to_sec()
			self.save_and_publish_score()
			rospy.loginfo("segment_time: %f" % segment_time.to_sec())
			
			
			##########################################
			#    CHECK RESULT i AND UPDATE SCORE     #
			##########################################
			
			self.score["segment_%i"%(i+1)]["timeout"] = goal.has_timed_out()
			self.score["segment_%i"%(i+1)]["completed"] = goal.has_been_completed()
			
			if goal.has_timed_out():
				print "GOAL TIMEOUT"
				timed_out_waypoints += 1
			elif goal.has_been_completed():
				print "GOAL COMPLETED:"
				result = goal.get_result()
				reached_waypoints += 1
				print "result:\n", result
				self.score["segment_%i"%(i+1)]["result"] = result
			else:
				print "GOAL NOT COMPLETED"
			
			self.save_and_publish_score()
			
			
			# Collect the target and robot poses (using tf)
			robot_pose_acquired = False
			while not robot_pose_acquired and self.is_benchmark_running():
				try:
					now = rospy.Time.now()
					
					# testbed_origin_to_marker transform
					tf_listener.waitForTransform("/testbed_origin", "/robot_markerset", now, rospy.Duration(1.0))
					marker_transform = tf_listener.lookupTransform("/testbed_origin", "/robot_markerset", now)
					
					# broadcast the robot frame relative to the marker
					tf_broadcaster.sendTransform(marker_to_robot_transform[0], marker_to_robot_transform[1], now, "/robot", "/robot_markerset")
					
					# receive the robot transform and compute the robot pose
					tf_listener.waitForTransform("/testbed_origin", "/robot", now, rospy.Duration(1.0))
					( (robot_x, robot_y, _), robot_rotation ) = tf_listener.lookupTransform("/testbed_origin", "/robot", rospy.Time(0))
					(_, _, robot_theta) = tf.transformations.euler_from_quaternion(robot_rotation)
					
					robot_pose = Pose2D(robot_x, robot_y, robot_theta)
					target_pose = Pose2D(*waypoints[i])
					
					self.score["segment_%i"%(i+1)]["target_pose"] = yaml.dump(target_pose)
					self.score["segment_%i"%(i+1)]["robot_pose"]  = yaml.dump(robot_pose)
					
					robot_pose_acquired = True
				
				except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf.Exception), Argument: # TODO IMPORTANT: tf.Exception when there is no optitrack. To test with optitrack
					robot_pose_acquired = False
					rospy.logwarn("Robot pose could not be acquired")
					rospy.logwarn(Argument)
			
			if robot_pose_acquired:
				# Evaluate position and orientation error between target pose (tp) and robot pose (rp)
				rp = robot_pose	# robot position
				tp = target_pose # target position
			
				d_error = math.sqrt( (rp.x-tp.x)**2 + (rp.y-tp.y)**2 )
				sum_d = sum_d + d_error
			
				sin_error = math.sin(math.fabs(rp.theta - tp.theta)) # TODO: ? fabs(sin(fabs(rp.theta - tp.theta)))
				sum_sin += sin_error
				cos_error = math.cos(math.fabs(rp.theta - tp.theta))
				sum_cos += cos_error
			
				self.score["segment_%i"%(i+1)]["distance_error"] = d_error
				self.score["segment_%i"%(i+1)]["orientation_error"] = math.atan2(sin_error, cos_error)
			
				rospy.loginfo("\n\n - segment_time: %s\n - dinstance_error: %s\n - orientation_error: %s" % (segment_time.to_sec(), self.score["segment_%i"%(i+1)]["distance_error"], self.score["segment_%i"%(i+1)]["orientation_error"]))
			
			self.save_and_publish_score()
			
			
			if not self.is_benchmark_running():
				print self.get_end_description()
				
				if self.has_benchmark_timed_out():
					print "BENCHMARK TIMEOUT"
				elif self.has_benchmark_been_stopped():
					print "BENCHMARK STOPPED"
				else:
					print "BENCHMARK ABORTED"
			
			i += 1
		
		##########################################
		#            UPDATE SCORE                #
		##########################################
		
		# Evaluate final score
		if reached_waypoints > 0:
			position_accuracy = sum_d / reached_waypoints
			orientation_accuracy = math.atan2(sum_sin, sum_cos)
		else:
			position_accuracy = None
			orientation_accuracy = None
		
		self.score["reached_waypoints"] = reached_waypoints
		self.score["timed_out_waypoints"] = timed_out_waypoints
		self.score["execution_time[s]"] = execution_time.to_sec()
		self.score["position_accuracy[m]"] = position_accuracy
		self.score["orientation_accuracy[rad]"] = orientation_accuracy
		self.save_and_publish_score()





