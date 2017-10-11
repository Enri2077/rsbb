from threading import Condition, Lock, Thread

import rospy
from rsbb_benchmarking_messages.msg import BmBoxState, RefBoxState, SystemStatus
from rsbb_benchmarking_messages.srv import ExecuteManualOperation, ExecuteGoal, EndBenchmark
from std_msgs.msg import String, Float32
from FSM import FSM
from StateObserver3 import StateObserver


STATE_UPDATE_RATE = 10 # 10Hz


class BmBox:
	def _pub_thread(self):
		r = rospy.Rate(STATE_UPDATE_RATE)
		
#		while (self._fsm.state() != BmBoxState.END) and (not rospy.is_shutdown()):
		while not rospy.is_shutdown():
			self._state_pub.publish(self._fsm.state(), self._fsm.payload())
			self.publish_system_status()
			r.sleep()
		
		self._state_pub.unregister()
		print "_pub_thread(self) return"
	
	def _refbox_state_cb(self, msg):
		
		self._refbox_state = msg
		self._refbox_state_observer.update(msg)
		
		if self._refbox_state.benchmark_state in self._exception_states\
		or self._refbox_state.goal_execution_state in self._exception_states\
		or self._refbox_state.manual_operation_state in self._exception_states:
			rospy.loginfo("setting state to BmBoxState.END")
			self._fsm.update(BmBoxState.END, None)
	
	def publish_system_status(self, d = ""):
		self._status.header.stamp = rospy.Time.now()
		self._status.status_description = d
		self._status_pub.publish(self._status)
	
	def WaitClient(self):
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
		
	def ManualOperation(self, message=None):
		rospy.logdebug("BmBox.ManualOperation()")
		
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
		
		self._fsm.update(BmBoxState.WAITING_MANUAL_OPERATION, message)
		
		try:
			print "calling execute_manual_operation service: ", message
			execute_manual_operation = rospy.ServiceProxy('/execute_manual_operation', ExecuteManualOperation)
			manual_operation_payload = String()
			manual_operation_payload.data = message
			response = execute_manual_operation(manual_operation_payload)
			
			if response.result.data:
				self._refbox_state_observer.wait_manual_operation_state_transition(from_state = RefBoxState.READY, to_states = [RefBoxState.EXECUTING_MANUAL_OPERATION])
				self._refbox_state_observer.wait_manual_operation_state_transition(from_state = RefBoxState.EXECUTING_MANUAL_OPERATION, to_states = [RefBoxState.READY])
				
				self._fsm.update(BmBoxState.READY, None)
			
			else:
				rospy.logerr("ManualOperation: Manual operation FAILED (refbox refused to execute the manual operation)")
				self._fsm.update(BmBoxState.END, None)
				
		
		except rospy.ServiceException, e:
			rospy.logerr("Service call failed: %s"%e)
			return
		
		return self._refbox_state.manual_operation_payload
	
	def SendGoal(self, goal = "", timeout = 0):
		rospy.logdebug("BmBox.SendGoal()")
		
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
		
		rospy.loginfo("Sending goal...")
		self._fsm.update(BmBoxState.TRANSMITTING_GOAL, goal)
		
		try:
			print "calling execute_goal service: ", goal
			execute_goal = rospy.ServiceProxy('/execute_goal', ExecuteGoal)
			goal_payload = String()
			goal_payload.data = goal
			timeout_payload = Float32()
			timeout_payload.data = timeout
			response = execute_goal(goal_payload, timeout_payload)
			
			if response.result.data:
				self._refbox_state_observer.wait_goal_execution_state_transition(from_state = RefBoxState.READY, to_states = [RefBoxState.TRANSMITTING_GOAL, RefBoxState.GOAL_TIMEOUT])
				self._refbox_state_observer.wait_goal_execution_state_transition(from_state = RefBoxState.TRANSMITTING_GOAL, to_states = [RefBoxState.EXECUTING_GOAL, RefBoxState.GOAL_TIMEOUT])
				self._fsm.update(BmBoxState.EXECUTING_GOAL, goal)
			
			else:
				rospy.logerr("SendGoal: Goal request FAILED (refbox refused to execute the goal)")
				self._fsm.update(BmBoxState.END, None)
				
		except rospy.ServiceException, e:
			rospy.logerr("Service call failed: %s"%e)
			return
		
		return self._refbox_state.goal_execution_payload
	
	def WaitResult(self):
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
		
		return self._refbox_state.goal_execution_payload
	
	def SendScore(self, score):
		rospy.logdebug("BmBox.SendScore()")
		
		if rospy.is_shutdown(): return
		
		if self._fsm.state() == BmBoxState.END:
			rospy.loginfo("Benchmark terminated, doing nothing")
			return
		
		if not (self._fsm.state() == BmBoxState.READY):
			rospy.logerr("SendScore: expected to be in state BmBoxState.READY")
			return
		
		if not (self._refbox_state.benchmark_state == RefBoxState.EXECUTING_BENCHMARK \
		and self._refbox_state.goal_execution_state in [RefBoxState.READY, RefBoxState.GOAL_TIMEOUT]
		and	self._refbox_state.manual_operation_state == RefBoxState.READY):
			rospy.logerr("SendScore: refbox expected to be in state RefBoxState.EXECUTING_BENCHMARK and (RefBoxState.READY or RefBoxState.GOAL_TIMEOUT)")
			return
		
		self._fsm.update(BmBoxState.TRANSMITTING_SCORE, score)
		
		
		try:
			print "calling end_benchmark service: ", score
			end_benchmark = rospy.ServiceProxy('/end_benchmark', EndBenchmark)
			end_benchmark_payload = String()
			end_benchmark_payload.data = score
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
	
	def isGoalTimedout(self):
		rospy.logdebug("BmBox.Timeout()")
				
		return self._refbox_state.goal_execution_state == RefBoxState.GOAL_TIMEOUT
	
	def End(self):
		rospy.logdebug("BmBox.End()")
		
		if rospy.is_shutdown(): return
		
		self._fsm.update(BmBoxState.END)
		
		self._refbox_state_sub.unregister()
	
	
	
	def isWaitingToStart(self):
		return self._fsm.state() in [BmBoxState.START, BmBoxState.WAITING_CLIENT]
		
	def isBenchmarkRunning(self):
		return not self.isBenchmarkEnded()
	
	def isBenchmarkEnded(self):
		return rospy.is_shutdown() \
		or     self._refbox_state.benchmark_state in self._exception_states \
		or     self._refbox_state.goal_execution_state in self._exception_states \
		or     self._refbox_state.manual_operation_state in self._exception_states \
		or     self._fsm.state() == BmBoxState.END
		
	def getEndReason(self):
		if   self._refbox_state.benchmark_state == RefBoxState.END:            return 'END: benchmark ended normally'
		elif self._refbox_state.benchmark_state == RefBoxState.STOP:           return 'STOP: benchmark stopped by referee'
		elif self._refbox_state.benchmark_state == RefBoxState.EMERGENCY_STOP: return 'EMERGENCY_STOP: benchmark stopped due to emergency'
		elif self._refbox_state.benchmark_state == RefBoxState.ERROR:          return 'ERROR: benchmark terminated due to RefBox error'
		elif self._refbox_state.benchmark_state == RefBoxState.GLOBAL_TIMEOUT: return 'GLOBAL_TIMEOUT: benchmark ended due to global timeout'
		else:                                                                  return 'not ended or unnown reason'
	
	
	
	
	def can_terminate_benchmark(self):
		return self.isBenchmarkEnded() or self.isWaitingToStart()
	
	def terminate_benchmark(self):
		if self.can_terminate_benchmark():
			print "signal_shutdown(\"Benchmark terminated\")"
			
			rospy.signal_shutdown("Benchmark terminated")
			self._fsm.notify_condition_variables()
			self._refbox_state_observer.notify_condition_variables()
	
	
	
	
	
	def __init__(self):
		
		print "BmBox.__init__(self)"
		
		self._fsm = FSM(
			('START', 'WAITING_CLIENT', 'READY', 'WAITING_MANUAL_OPERATION', 'COMPLETED_MANUAL_OPERATION', 'TRANSMITTING_GOAL', 'EXECUTING_GOAL', 'WAITING_RESULT', 'TRANSMITTING_SCORE', 'END'),
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
		
		self._refbox_state_observer = StateObserver(self._exception_states, self._refbox_state)

		self._pub_thread = Thread(name="pub_thread", target=self._pub_thread)
		self._pub_thread.start()
		
