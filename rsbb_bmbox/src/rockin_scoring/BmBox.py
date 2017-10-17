from threading import Condition, Lock, Thread

import rospy, yaml
from rsbb_benchmarking_messages.msg import BmBoxState, RefBoxState, SystemStatus
from rsbb_benchmarking_messages.srv import *
from std_msgs.msg import String, Float32
from FSM import FSM
from StateObserver import StateObserver


STATE_UPDATE_RATE = 10 # 10Hz

bmbox_state_to_str = {
0: "START",
1: "WAITING_CLIENT",
2: "READY",
3: "WAITING_MANUAL_OPERATION",
4: "COMPLETED_MANUAL_OPERATION",
5: "TRANSMITTING_GOAL",
6: "EXECUTING_GOAL",
7: "WAITING_RESULT",
8: "TRANSMITTING_SCORE",
9: "END"
}

refbox_state_to_str = {
0: "START",
1: "EXECUTING_BENCHMARK",
2: "END",
3: "STOP",
4: "EMERGENCY_STOP",
5: "ERROR",
6: "GLOBAL_TIMEOUT",
7: "READY",
8: "TRANSMITTING_GOAL",
9: "EXECUTING_GOAL",
10: "GOAL_TIMEOUT",
11: "EXECUTING_MANUAL_OPERATION",
12: "NONE"
}

class BmBox:
	
	def __init__(self):
		
#		print "BmBox.__init__(self)"
		
		self.__current_goal = None
		self.__current_manual_operation = None
		
		self.__fsm = FSM(
			bmbox_state_to_str,
			(
				(BmBoxState.WAITING_CLIENT, BmBoxState.END), # allowed transitions from BmBoxState.START
				(BmBoxState.READY, BmBoxState.END), # allowed transitions from BmBoxState.WAITING_CLIENT
				(BmBoxState.WAITING_MANUAL_OPERATION, BmBoxState.TRANSMITTING_GOAL, BmBoxState.TRANSMITTING_SCORE, BmBoxState.END), # allowed transitions from BmBoxState.READY
				(BmBoxState.COMPLETED_MANUAL_OPERATION, BmBoxState.READY, BmBoxState.END), # allowed transitions from BmBoxState.WAITING_MANUAL_OPERATION
				(BmBoxState.TRANSMITTING_GOAL, BmBoxState.WAITING_MANUAL_OPERATION, BmBoxState.END), # allowed transitions from BmBoxState.COMPLETED_MANUAL_OPERATION
				(BmBoxState.EXECUTING_GOAL, BmBoxState.END), # allowed transitions from BmBoxState.TRANSMITTING_GOAL
				(BmBoxState.WAITING_RESULT, BmBoxState.READY, BmBoxState.END), # allowed transitions from BmBoxState.EXECUTING_GOAL
				(BmBoxState.READY, BmBoxState.END), # allowed transitions from BmBoxState.WAITING_RESULT
				(BmBoxState.END, ), # allowed transitions from BmBoxState.TRANSMITTING_SCORE
				(BmBoxState.END, ), # allowed transitions from BmBoxState.END
			),
			BmBoxState.START
		)
		
		self.__exception_states = [RefBoxState.END, RefBoxState.STOP, RefBoxState.ERROR, RefBoxState.GLOBAL_TIMEOUT]
		
		self.__refbox_state = RefBoxState()
		self.__status = SystemStatus()
		self.__status.status = SystemStatus.NORMAL
		
#		self.__refbox_state_sub = rospy.Subscriber("bmbox/refbox_state", RefBoxState, self.__refbox_state_cb)

		self.__state_pub = rospy.Publisher("bmbox/bmbox_state", BmBoxState, queue_size=10)
		self.__status_pub = rospy.Publisher("rsbb_system_status/bmbox", SystemStatus, queue_size=10)
		
		self.__refbox_state_observer = StateObserver(self.__exception_states, self.__refbox_state, refbox_state_to_str)

		self.__pub_thread = Thread(name="pub_thread", target=self.__pub_thread)
		self.__pub_thread.start()
		
		self._start_benchmark_server = rospy.Service("bmbox/start_benchmark", StartBenchmark, self.__start_benchmark_callback)
		self._manual_operation_complete_server = rospy.Service("bmbox/manual_operation_complete", ManualOperationComplete, self.__manual_operation_complete_callback)
		self._goal_started_server = rospy.Service("bmbox/goal_execution_started", GoalStarted, self.__goal_started_callback)
		self._goal_complete_server = rospy.Service("bmbox/goal_complete", GoalComplete, self.__goal_complete_callback)
		self._stop_benchmark_server = rospy.Service("bmbox/stop_benchmark", GoalStarted, self.__stop_benchmark_callback)
	
	
	def __pub_thread(self):
		r = rospy.Rate(STATE_UPDATE_RATE)
		
#		while (self.__fsm.state() != BmBoxState.END) and (not rospy.is_shutdown()):
		while not rospy.is_shutdown():
			self.__state_pub.publish(self.__fsm.state(), self.__fsm.payload())
			self.__publish_system_status("executing")
			
#			print "__pub_thread(self)", self.get_benchmark_code()
			r.sleep()
		
		self.__state_pub.unregister()
#		print "__pub_thread(self) return"
	
	
	def __start_benchmark_callback(self, request):
		
		self.__refbox_state = request.refbox_state
#		print "###############################   start_benchmark_callback __refbox_state_observer.update   BEGIN"
		self.__refbox_state_observer.update(request.refbox_state)
#		print "###############################   start_benchmark_callback __refbox_state_observer.update   END"
		
		return StartBenchmarkResponse(True)
	
	
	def __goal_started_callback(self, request):
		
		if self.__current_goal == None:
#			print "###############################   goal_started_callback   return    (self.__current_goal == None)"
			return GoalStartedResponse(False)
		
		self.__refbox_state = request.refbox_state
#		print "###############################   goal_started_callback __refbox_state_observer.update   BEGIN"
		self.__refbox_state_observer.update(request.refbox_state)
#		print "###############################   goal_started_callback __refbox_state_observer.update   END"
		
		return GoalStartedResponse(True)
	
	
	def __goal_complete_callback(self, request):
		
		if self.__current_goal == None:
#			print "###############################   goal_complete_callback   return    (self.__current_goal == None)"
			return GoalCompleteResponse(False)
		else:
			if request.goal_timeout:
				self.__current_goal.set_has_timed_out()
			else:
				self.__current_goal.set_result_string(request.goal_result)
			## TODO better done in the appropriate function ? how to store goal result or timeout otherwise?
		
		self.__refbox_state = request.refbox_state
#		print "###############################   goal_complete_callback __refbox_state_observer.update   BEGIN"
		self.__refbox_state_observer.update(request.refbox_state)
#		print "###############################   goal_complete_callback __refbox_state_observer.update   END"
		
		return GoalCompleteResponse(True)
	
	
	def __manual_operation_complete_callback(self, request):
		
		# TODO check BmBox state is consistent
		
		if self.__current_manual_operation == None:
#			print "###############################   manual_operation_complete_callback   return    (self.__current_manual_operation == None)"
			return ManualOperationCompleteResponse(False)
		else:
			self.__current_manual_operation.set_result(request.manual_operation_result)
			## TODO better done in the appropriate function ? how to store result otherwise?
		
		self.__refbox_state = request.refbox_state
#		print "###############################   manual_operation_complete_callback __refbox_state_observer.update   BEGIN"
		self.__refbox_state_observer.update(request.refbox_state)
#		print "###############################   manual_operation_complete_callback __refbox_state_observer.update   END"
		
		return ManualOperationCompleteResponse(True)
	
	def __stop_benchmark_callback(self, request):
		
		print type(request)
		
		# TODO check BmBox state is consistent
	
		self.__refbox_state = request.refbox_state
		print "###############################   stop_benchmark __refbox_state_observer.update   BEGIN"
		self.__refbox_state_observer.update(request.refbox_state)
		print "###############################   stop_benchmark __refbox_state_observer.update   END"
		
		# TODO call end_benchmark or fsm.state = BmBoxState.END or do nothing?
		
		return True
		
		
	def __refbox_state_cb(self, msg):
		
		self.__refbox_state = msg
#		print "###############################   __refbox_state_cb   begin"
#		self.__refbox_state_observer.update(msg)
#		print "###############################   __refbox_state_cb   end"
		
		if self.__refbox_state.benchmark_state in self.__exception_states \
		or self.__refbox_state.goal_execution_state in self.__exception_states \
		or self.__refbox_state.manual_operation_state in self.__exception_states:
			self.__fsm.update(BmBoxState.END, None)
	
	def __publish_system_status(self, d = ""):
		self.__status.header.stamp = rospy.Time.now()
		self.__status.status_description = d
		self.__status_pub.publish(self.__status)
	
	def _wait_refbox_connection(self):
		rospy.logdebug("BmBox.WaitClient()")
		
		if rospy.is_shutdown(): return
		
		if self.__fsm.state() == BmBoxState.END:
			rospy.loginfo("Benchmark terminated, doing nothing")
			return
		
		if not self.__fsm.check_state(BmBoxState.START): return
		
		rospy.loginfo("Waiting for client...")
		self.__fsm.update(BmBoxState.WAITING_CLIENT)
		self.__refbox_state_observer.wait_benchmark_state_transition(from_state = RefBoxState.START, to_states = [RefBoxState.EXECUTING_BENCHMARK])
		self.__fsm.update(BmBoxState.READY)
		
		# TODO manage exception (stop, global timeout, error)
		
	def manual_operation(self, manual_operation_object=None):
		rospy.logdebug("BmBox.ManualOperation()")
		
		# TODO check manual_operation_object type == ManualOperationObject
		
		if rospy.is_shutdown(): return
		
		if self.__fsm.state() == BmBoxState.END:
			rospy.loginfo("manual_operation: Benchmark terminated, doing nothing")
			return
		
		if not ((self.__fsm.state() == BmBoxState.READY) or (self.__fsm.state() == BmBoxState.COMPLETED_MANUAL_OPERATION)):
			rospy.logerr("manual_operation: expected to be in state BmBoxState.READY or BmBoxState.COMPLETED_MANUAL_OPERATION")
			return
		
		if not (self.__refbox_state.manual_operation_state == RefBoxState.READY):
			rospy.logerr("manual_operation: refbox expected to be in state RefBoxState.READY")
			return
		
		if self.__current_manual_operation != None:
			rospy.logerr("manual_operation: another manual operation request is pending")
		
		
		self.__current_manual_operation = manual_operation_object
		
		self.__fsm.update(BmBoxState.WAITING_MANUAL_OPERATION, self.__current_manual_operation.get_request())
		
		try:
			print "calling execute_manual_operation service: ", self.__current_manual_operation.get_request()
			execute_manual_operation = rospy.ServiceProxy("bmbox/execute_manual_operation", ExecuteManualOperation)
			manual_operation_payload = String()
			manual_operation_payload.data = self.__current_manual_operation.get_request()
			response = execute_manual_operation(manual_operation_payload)
			
			if response.result.data:
				
				self.__refbox_state = response.refbox_state
				self.__refbox_state_observer.update(response.refbox_state)
				
				self.__refbox_state_observer.wait_manual_operation_state_transition(from_state = RefBoxState.READY, to_states = [RefBoxState.EXECUTING_MANUAL_OPERATION])
				self.__refbox_state_observer.wait_manual_operation_state_transition(from_state = RefBoxState.EXECUTING_MANUAL_OPERATION, to_states = [RefBoxState.READY])
#				print "###############################   wait_manual_operation_state_transition   return"
				self.__fsm.update(BmBoxState.READY, None)
				
				# TODO manage exception (stop, global timeout, error)
				
#				self.__current_manual_operation.set_result(self.__refbox_state.manual_operation_payload)
			
			else:
				rospy.logerr("manual_operation: Manual operation FAILED (refbox refused to execute the manual operation)")
				self.__fsm.update(BmBoxState.END, None)
				
		
		except rospy.ServiceException, e:
			rospy.logerr("Service call failed: %s"%e)
		
		
#		print "###############################   manual_operation   return"
		
		self.__current_manual_operation = None
		return
	
	def request_goal(self, goal_object):
		rospy.logdebug("BmBox.SendGoal()")
		
		# TODO check goal_object type == GoalObject
		
		if self.__current_goal != None:
			rospy.logerr("request_goal: another goal request is pending")
		
		if rospy.is_shutdown(): return
		
		if self.__fsm.state() == BmBoxState.END:
			rospy.loginfo("request_goal: Benchmark terminated, doing nothing")
			return
		
		if not (self.__fsm.state() == BmBoxState.READY):
			rospy.logerr("request_goal: expected to be in state BmBoxState.READY")
			return
		
		if not (self.__refbox_state.goal_execution_state in [RefBoxState.READY, RefBoxState.GOAL_TIMEOUT]):
			rospy.logerr("request_goal: refbox expected to be in states RefBoxState.READY or RefBoxState.GOAL_TIMEOUT")
			return
		
		self.__current_goal = goal_object
		
		self.__fsm.update(BmBoxState.TRANSMITTING_GOAL, self.__current_goal.get_request_string())
		
		try:
			print "calling execute_goal service: request:", self.__current_goal.get_request_string(), "timeout:", self.__current_goal.get_timeout()
			execute_goal = rospy.ServiceProxy("bmbox/execute_goal", ExecuteGoal)
			goal_payload = String()
			goal_payload.data = self.__current_goal.get_request_string()
			timeout_payload = Float32()
			timeout_payload.data = self.__current_goal.get_timeout()
			response = execute_goal(goal_payload, timeout_payload)
			
			if response.result.data:
				
				self.__refbox_state = response.refbox_state
				self.__refbox_state_observer.update(response.refbox_state)
				
				self.__refbox_state_observer.wait_goal_execution_state_transition(from_state = RefBoxState.READY, to_states = [RefBoxState.TRANSMITTING_GOAL, RefBoxState.GOAL_TIMEOUT])
				self.__refbox_state_observer.wait_goal_execution_state_transition(from_state = RefBoxState.TRANSMITTING_GOAL, to_states = [RefBoxState.EXECUTING_GOAL, RefBoxState.GOAL_TIMEOUT])
				self.__fsm.update(BmBoxState.EXECUTING_GOAL, self.__current_goal.get_request_string())
				
				# TODO manage exception (stop, global timeout, error)
		
#				if self.__refbox_state.goal_execution_state == RefBoxState.GOAL_TIMEOUT:
#					self.__current_goal.set_has_timed_out(True)
				
			else:
				rospy.logerr("SendGoal: Goal request FAILED (refbox refused to execute the goal)")
				self.__fsm.update(BmBoxState.END, None)
				
		except rospy.ServiceException, e:
			rospy.logerr("Service call failed: %s"%e)
			return
		
		return
	
	
	def wait_goal_result(self):
		rospy.loginfo("BmBox.WaitResult()")
		
		if rospy.is_shutdown(): return
		
		if self.__fsm.state() == BmBoxState.END:
			rospy.loginfo("wait_goal_result: Benchmark terminated, doing nothing")
			return
		
		if not (self.__fsm.state() == BmBoxState.EXECUTING_GOAL):
			rospy.logerr("wait_goal_result: expected to be in state BmBoxState.EXECUTING_GOAL")
			return
		
		if not (self.__refbox_state.goal_execution_state in [RefBoxState.EXECUTING_GOAL, RefBoxState.GOAL_TIMEOUT]):
			rospy.logerr("wait_goal_result: refbox expected to be in state RefBoxState.EXECUTING_GOAL or RefBoxState.GOAL_TIMEOUT")
			return
		
		self.__fsm.update(BmBoxState.WAITING_RESULT)
		self.__refbox_state_observer.wait_goal_execution_state_transition(from_state = RefBoxState.EXECUTING_GOAL, to_states = [RefBoxState.READY, RefBoxState.GOAL_TIMEOUT])
		self.__fsm.update(BmBoxState.READY, None)
		
		# TODO manage exception (stop, global timeout, error)
		
#		if self.__refbox_state.goal_execution_state == RefBoxState.READY:
#			self.__current_goal.set_result_string(self.__refbox_state.goal_execution_payload)
#		
#		elif self.__refbox_state.goal_execution_state == RefBoxState.GOAL_TIMEOUT:
#			self.__current_goal.set_has_timed_out(True)
#		
#		print "wait_goal_result: result:", self.__refbox_state.goal_execution_payload
		self.__current_goal = None
		return
	
	
	def __send_score(self):
		rospy.logdebug("BmBox.SendScore()")
		
		if rospy.is_shutdown(): return
		
		if self.__fsm.state() == BmBoxState.END:
			rospy.loginfo("Benchmark terminated, doing nothing")
			return
		
		if not (self.__fsm.state() == BmBoxState.READY):
			rospy.logerr("SendScore: expected to be in state BmBoxState.READY")
			return
		
		if not (self.__refbox_state.benchmark_state == RefBoxState.EXECUTING_BENCHMARK \
		and self.__refbox_state.goal_execution_state in [RefBoxState.READY, RefBoxState.GOAL_TIMEOUT] \
		and	self.__refbox_state.manual_operation_state == RefBoxState.READY):
			rospy.logerr("SendScore: refbox expected to be in state RefBoxState.EXECUTING_BENCHMARK and (RefBoxState.READY or RefBoxState.GOAL_TIMEOUT)")
			return
		
		score_yaml_string = yaml.dump(self.get_current_score())
		
		self.__fsm.update(BmBoxState.TRANSMITTING_SCORE, score_yaml_string)
		
		
		try:
			print "calling end_benchmark service: ", score_yaml_string
			end_benchmark = rospy.ServiceProxy("bmbox/end_benchmark", EndBenchmark)
			end_benchmark_payload = String()
			end_benchmark_payload.data = score_yaml_string
			response = end_benchmark(end_benchmark_payload)
			
			if response.result.data:
				
				self.__refbox_state = response.refbox_state
				self.__refbox_state_observer.update(response.refbox_state)
				
				self.__refbox_state_observer.wait_benchmark_state_transition(from_state = RefBoxState.EXECUTING_BENCHMARK, to_states = [RefBoxState.END])
				self.__fsm.update(BmBoxState.END, None)
				
			else:
				rospy.logerr("SendScore: request FAILED (refbox refused to accept the score)")
				self.__fsm.update(BmBoxState.END, None)
		
		except rospy.ServiceException, e:
			rospy.logerr("Service call failed: %s"%e)
			return
	
	
	
	
	def end_benchmark(self):
		rospy.logdebug("BmBox.end_benchmark()")
		
		if rospy.is_shutdown(): return
		
		if self.__fsm.state() == BmBoxState.END:
			rospy.loginfo("end_benchmark: Benchmark already ended, doing nothing")
			return
		
		if not (self.__fsm.state() == BmBoxState.READY):
			rospy.logerr("end_benchmark: expected to be in state BmBoxState.READY")
			return
		
		if not (self.__refbox_state.benchmark_state == RefBoxState.EXECUTING_BENCHMARK \
		and self.__refbox_state.goal_execution_state in [RefBoxState.READY, RefBoxState.GOAL_TIMEOUT] \
		and self.__refbox_state.manual_operation_state == RefBoxState.READY):
			rospy.logerr("end_benchmark: refbox expected to be in state RefBoxState.EXECUTING_BENCHMARK and (RefBoxState.READY or RefBoxState.GOAL_TIMEOUT)")
			return
		
		self.__send_score()
		
#		self.__refbox_state_sub.unregister() TODO still usefull
	
	
	
	
	def __is_waiting_to_start(self):
		return self.__fsm.state() in [BmBoxState.START, BmBoxState.WAITING_CLIENT]
	
	def is_benchmark_running(self):
		return not self.is_benchmark_ended()
	
	def is_benchmark_ended(self):
		return rospy.is_shutdown() \
		or     self.__refbox_state.benchmark_state in self.__exception_states \
		or     self.__refbox_state.goal_execution_state in self.__exception_states \
		or     self.__refbox_state.manual_operation_state in self.__exception_states \
		or     self.__fsm.state() == BmBoxState.END
	
	def is_goal_timed_out(self):
		rospy.logdebug("BmBox.is_goal_timed_out()")
		return self.__refbox_state.goal_execution_state == RefBoxState.GOAL_TIMEOUT
	
	def get_end_reason(self):
		if   self.__refbox_state.benchmark_state == RefBoxState.END:            return 'END: benchmark ended normally'
		elif self.__refbox_state.benchmark_state == RefBoxState.STOP:           return 'STOP: benchmark stopped by referee'
		elif self.__refbox_state.benchmark_state == RefBoxState.EMERGENCY_STOP: return 'EMERGENCY_STOP: benchmark stopped due to emergency'
		elif self.__refbox_state.benchmark_state == RefBoxState.ERROR:          return 'ERROR: benchmark terminated due to RefBox error'
		elif self.__refbox_state.benchmark_state == RefBoxState.GLOBAL_TIMEOUT: return 'GLOBAL_TIMEOUT: benchmark ended due to global timeout'
		else:                                                                   return 'not ended or unnown reason'
	
	def can_terminate_benchmark(self):
		return self.is_benchmark_ended() or self.__is_waiting_to_start()
	
	def terminate_benchmark(self):
		if self.can_terminate_benchmark():
			print "signal_shutdown(\"Benchmark terminated\")"
			
			rospy.signal_shutdown("Benchmark terminated")
			self.__fsm.notify_condition_variables()
			self.__refbox_state_observer.notify_condition_variables()
	
	
	
	
	
