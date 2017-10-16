from threading import Condition, Lock, Thread

import rospy, yaml
from rsbb_benchmarking_messages.msg import BmBoxState, RefBoxState, SystemStatus
from rsbb_benchmarking_messages.srv import ExecuteManualOperation, ExecuteGoal, EndBenchmark
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
	def _pub_thread(self):
		r = rospy.Rate(STATE_UPDATE_RATE)
		
#		while (self._fsm.state() != BmBoxState.END) and (not rospy.is_shutdown()):
		while not rospy.is_shutdown():
			self._state_pub.publish(self._fsm.state(), self._fsm.payload())
			self.publish_system_status("executing")
			
#			print "_pub_thread(self)", self.get_benchmark_code()
			r.sleep()
		
		self._state_pub.unregister()
		print "_pub_thread(self) return"
	
	def _refbox_state_cb(self, msg):
		
		self._refbox_state = msg
		self._refbox_state_observer.update(msg)
		
		if self._refbox_state.benchmark_state in self._exception_states \
		or self._refbox_state.goal_execution_state in self._exception_states \
		or self._refbox_state.manual_operation_state in self._exception_states:
			self._fsm.update(BmBoxState.END, None)
	
	def publish_system_status(self, d = ""):
		self._status.header.stamp = rospy.Time.now()
		self._status.status_description = d
		self._status_pub.publish(self._status)
	
	def _wait_refbox_connection(self):
		rospy.logdebug("BmBox.WaitClient()")
		
		if rospy.is_shutdown(): return
		
		if self._fsm.state() == BmBoxState.END:
			rospy.loginfo("Benchmark terminated, doing nothing")
			return
		
		if not self._fsm.check_state(BmBoxState.START): return
		
		rospy.loginfo("Waiting for client...")
		self._fsm.update(BmBoxState.WAITING_CLIENT)
		self._refbox_state_observer.wait_benchmark_state_transition(from_state = RefBoxState.START, to_states = [RefBoxState.EXECUTING_BENCHMARK])
		self._fsm.update(BmBoxState.READY)
		
	def manual_operation(self, manual_operation_object=None):
		rospy.logdebug("BmBox.ManualOperation()")
		
		# TODO check manual_operation_object type == ManualOperationObject
		# TODO check no other manual operation is being executed
		
		if rospy.is_shutdown(): return
		
		if self._fsm.state() == BmBoxState.END:
			rospy.loginfo("Benchmark terminated, doing nothing")
			return
		
		if not ((self._fsm.state() == BmBoxState.READY) or (self._fsm.state() == BmBoxState.COMPLETED_MANUAL_OPERATION)):
			rospy.logerr("ManualOperation: expected to be in state BmBoxState.READY or BmBoxState.COMPLETED_MANUAL_OPERATION")
			return
		
		if not (self._refbox_state.manual_operation_state == RefBoxState.READY):
			rospy.logerr("ManualOperation: refbox expected to be in state RefBoxState.READY")
			return
		
		self._current_manual_operation = manual_operation_object
		
		self._fsm.update(BmBoxState.WAITING_MANUAL_OPERATION, self._current_manual_operation.get_request())
		
		try:
			print "calling execute_manual_operation service: ", self._current_manual_operation.get_request()
			execute_manual_operation = rospy.ServiceProxy('/execute_manual_operation', ExecuteManualOperation)
			manual_operation_payload = String()
			manual_operation_payload.data = self._current_manual_operation.get_request()
			response = execute_manual_operation(manual_operation_payload)
			
			if response.result.data:
				self._refbox_state_observer.wait_manual_operation_state_transition(from_state = RefBoxState.READY, to_states = [RefBoxState.EXECUTING_MANUAL_OPERATION])
				self._refbox_state_observer.wait_manual_operation_state_transition(from_state = RefBoxState.EXECUTING_MANUAL_OPERATION, to_states = [RefBoxState.READY])
				self._fsm.update(BmBoxState.READY, None)
				
				self._current_manual_operation.set_result(self._refbox_state.manual_operation_payload)
			
			else:
				rospy.logerr("ManualOperation: Manual operation FAILED (refbox refused to execute the manual operation)")
				self._fsm.update(BmBoxState.END, None)
				
		
		except rospy.ServiceException, e:
			rospy.logerr("Service call failed: %s"%e)
			return
		
#		return self._refbox_state.manual_operation_payload
		return
	
	def request_goal(self, goal_object):
		rospy.logdebug("BmBox.SendGoal()")
		
		# TODO check goal_object type == GoalObject
		# TODO check no other goal is being executed
		
		if rospy.is_shutdown(): return
		
		if self._fsm.state() == BmBoxState.END:
			rospy.loginfo("Benchmark terminated, doing nothing")
			return
		
		if not (self._fsm.state() == BmBoxState.READY):
			rospy.logerr("SendGoal: expected to be in state BmBoxState.READY")
			return
		
		if not (self._refbox_state.goal_execution_state in [RefBoxState.READY, RefBoxState.GOAL_TIMEOUT]):
			rospy.logerr("SendGoal: refbox expected to be in states RefBoxState.READY or RefBoxState.GOAL_TIMEOUT")
			return
		
		self._current_goal = goal_object
		
		self._fsm.update(BmBoxState.TRANSMITTING_GOAL, self._current_goal.get_request_string())
		
		try:
			print "calling execute_goal service: request:", self._current_goal.get_request_string(), "timeout:", self._current_goal.get_timeout()
			execute_goal = rospy.ServiceProxy('/execute_goal', ExecuteGoal)
			goal_payload = String()
			goal_payload.data = self._current_goal.get_request_string()
			timeout_payload = Float32()
			timeout_payload.data = self._current_goal.get_timeout()
			response = execute_goal(goal_payload, timeout_payload)
			
			if response.result.data:
				self._refbox_state_observer.wait_goal_execution_state_transition(from_state = RefBoxState.READY, to_states = [RefBoxState.TRANSMITTING_GOAL, RefBoxState.GOAL_TIMEOUT])
				self._refbox_state_observer.wait_goal_execution_state_transition(from_state = RefBoxState.TRANSMITTING_GOAL, to_states = [RefBoxState.EXECUTING_GOAL, RefBoxState.GOAL_TIMEOUT])
				self._fsm.update(BmBoxState.EXECUTING_GOAL, self._current_goal.get_request_string())
				
				if self._refbox_state.goal_execution_state == RefBoxState.GOAL_TIMEOUT:
					self._current_goal.set_has_timed_out(True)
					# TODO call timeout_ack service
				
			else:
				rospy.logerr("SendGoal: Goal request FAILED (refbox refused to execute the goal)")
				self._fsm.update(BmBoxState.END, None)
				
		except rospy.ServiceException, e:
			rospy.logerr("Service call failed: %s"%e)
			return
		
#		return self._refbox_state.goal_execution_payload
		return
	
	
	def wait_goal_result(self):
		rospy.loginfo("BmBox.WaitResult()")
		
		if rospy.is_shutdown(): return
		
		if self._fsm.state() == BmBoxState.END:
			rospy.loginfo("Benchmark terminated, doing nothing")
			return
		
		if not (self._fsm.state() == BmBoxState.EXECUTING_GOAL):
			rospy.logerr("WaitResult: expected to be in state BmBoxState.EXECUTING_GOAL")
			return
		
		if not (self._refbox_state.goal_execution_state in [RefBoxState.EXECUTING_GOAL, RefBoxState.GOAL_TIMEOUT]):
			rospy.logerr("WaitResult: refbox expected to be in state RefBoxState.EXECUTING_GOAL or RefBoxState.GOAL_TIMEOUT")
			return
		
		self._fsm.update(BmBoxState.WAITING_RESULT)
		self._refbox_state_observer.wait_goal_execution_state_transition(from_state = RefBoxState.EXECUTING_GOAL, to_states = [RefBoxState.READY, RefBoxState.GOAL_TIMEOUT])
		self._fsm.update(BmBoxState.READY, None)
		
		if self._refbox_state.goal_execution_state == RefBoxState.READY:
			self._current_goal.set_result_string(self._refbox_state.goal_execution_payload)
		
		elif self._refbox_state.goal_execution_state == RefBoxState.GOAL_TIMEOUT:
			self._current_goal.set_has_timed_out(True)
			# TODO call timeout_ack service
		
		print "wait_goal_result: result:", self._refbox_state.goal_execution_payload
		return
#		return self._refbox_state.goal_execution_payload
	
	
	def _send_score(self):
		rospy.logdebug("BmBox.SendScore()")
		
		if rospy.is_shutdown(): return
		
		if self._fsm.state() == BmBoxState.END:
			rospy.loginfo("Benchmark terminated, doing nothing")
			return
		
		if not (self._fsm.state() == BmBoxState.READY):
			rospy.logerr("SendScore: expected to be in state BmBoxState.READY")
			return
		
		if not (self._refbox_state.benchmark_state == RefBoxState.EXECUTING_BENCHMARK \
		and self._refbox_state.goal_execution_state in [RefBoxState.READY, RefBoxState.GOAL_TIMEOUT] \
		and	self._refbox_state.manual_operation_state == RefBoxState.READY):
			rospy.logerr("SendScore: refbox expected to be in state RefBoxState.EXECUTING_BENCHMARK and (RefBoxState.READY or RefBoxState.GOAL_TIMEOUT)")
			return
		
		score_yaml_string = yaml.dump(self.get_current_score())
		
		self._fsm.update(BmBoxState.TRANSMITTING_SCORE, score_yaml_string)
		
		
		try:
			print "calling end_benchmark service: ", score_yaml_string
			end_benchmark = rospy.ServiceProxy('/end_benchmark', EndBenchmark)
			end_benchmark_payload = String()
			end_benchmark_payload.data = score_yaml_string
			response = end_benchmark(end_benchmark_payload)
			
			if response.result.data:
				self._refbox_state_observer.wait_benchmark_state_transition(from_state = RefBoxState.EXECUTING_BENCHMARK, to_states = [RefBoxState.END])
				self._fsm.update(BmBoxState.END, None)
				
			else:
				rospy.logerr("SendScore: request FAILED (refbox refused to accept the score)")
				self._fsm.update(BmBoxState.END, None)
		
		except rospy.ServiceException, e:
			rospy.logerr("Service call failed: %s"%e)
			return
	
	
	
	
	def end_benchmark(self):
		rospy.logdebug("BmBox.end_benchmark()")
		
		if rospy.is_shutdown(): return
		
		if self._fsm.state() == BmBoxState.END:
			rospy.loginfo("end_benchmark: Benchmark already ended, doing nothing")
			return
		
		if not (self._fsm.state() == BmBoxState.READY):
			rospy.logerr("end_benchmark: expected to be in state BmBoxState.READY")
			return
		
		if not (self._refbox_state.benchmark_state == RefBoxState.EXECUTING_BENCHMARK \
		and self._refbox_state.goal_execution_state in [RefBoxState.READY, RefBoxState.GOAL_TIMEOUT] \
		and self._refbox_state.manual_operation_state == RefBoxState.READY):
			rospy.logerr("end_benchmark: refbox expected to be in state RefBoxState.EXECUTING_BENCHMARK and (RefBoxState.READY or RefBoxState.GOAL_TIMEOUT)")
			return
		
		self._send_score()
		
		self._refbox_state_sub.unregister()
	
	
	
	
	def _is_waiting_to_start(self):
		return self._fsm.state() in [BmBoxState.START, BmBoxState.WAITING_CLIENT]
	
	def is_benchmark_running(self):
		return not self.is_benchmark_ended()
	
	def is_benchmark_ended(self):
		return rospy.is_shutdown() \
		or     self._refbox_state.benchmark_state in self._exception_states \
		or     self._refbox_state.goal_execution_state in self._exception_states \
		or     self._refbox_state.manual_operation_state in self._exception_states \
		or     self._fsm.state() == BmBoxState.END
	
	def is_goal_timedout(self):
		rospy.logdebug("BmBox.is_goal_timedout()")
		return self._refbox_state.goal_execution_state == RefBoxState.GOAL_TIMEOUT
	
	def get_end_reason(self):
		if   self._refbox_state.benchmark_state == RefBoxState.END:            return 'END: benchmark ended normally'
		elif self._refbox_state.benchmark_state == RefBoxState.STOP:           return 'STOP: benchmark stopped by referee'
		elif self._refbox_state.benchmark_state == RefBoxState.EMERGENCY_STOP: return 'EMERGENCY_STOP: benchmark stopped due to emergency'
		elif self._refbox_state.benchmark_state == RefBoxState.ERROR:          return 'ERROR: benchmark terminated due to RefBox error'
		elif self._refbox_state.benchmark_state == RefBoxState.GLOBAL_TIMEOUT: return 'GLOBAL_TIMEOUT: benchmark ended due to global timeout'
		else:                                                                  return 'not ended or unnown reason'
	
	def can_terminate_benchmark(self):
		return self.is_benchmark_ended() or self._is_waiting_to_start()
	
	def terminate_benchmark(self):
		if self.can_terminate_benchmark():
			print "signal_shutdown(\"Benchmark terminated\")"
			
			rospy.signal_shutdown("Benchmark terminated")
			self._fsm.notify_condition_variables()
			self._refbox_state_observer.notify_condition_variables()
	
	
	
	
	
	def __init__(self):
		
		print "BmBox.__init__(self)"
		
		self._current_goal = None
		self._current_manual_operation = None
		
		self._fsm = FSM(
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
		
		self._exception_states = [RefBoxState.END, RefBoxState.STOP, RefBoxState.ERROR, RefBoxState.GLOBAL_TIMEOUT]
		
		self._refbox_state = RefBoxState()
		self._status = SystemStatus()
		self._status.status = SystemStatus.NORMAL
		
#		rospy.init_node("benchmark")
		self._refbox_state_sub = rospy.Subscriber("refbox_state", RefBoxState, self._refbox_state_cb)

		self._state_pub = rospy.Publisher("bmbox_state", BmBoxState, queue_size=10)
		self._status_pub = rospy.Publisher("/rsbb_system_status/bmbox", SystemStatus, queue_size=10)
		
		self._refbox_state_observer = StateObserver(self._exception_states, self._refbox_state, refbox_state_to_str)

		self._pub_thread = Thread(name="pub_thread", target=self._pub_thread)
		self._pub_thread.start()
		
