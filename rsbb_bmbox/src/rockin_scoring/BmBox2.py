from threading import Condition, Lock, Thread

import rospy
from rsbb_benchmarking_messages.msg import BmBoxState, RefBoxState
from rsbb_benchmarking_messages.srv import ExecuteManualOperation, ExecuteGoal, EndBenchmark
from std_msgs.msg import String, Float32
from FSM import FSM
from StateObserver import StateObserver


STATE_UPDATE_RATE = 10 # 10Hz


class BmBox2:
	def _pub_thread(self):
		r = rospy.Rate(STATE_UPDATE_RATE)
		
		while (self._fsm.state() != BmBoxState.END) and (not rospy.is_shutdown()):
			self._state_pub.publish(self._fsm.state(), self._fsm.payload())
			r.sleep()
	
	def _refbox_state_cb(self, msg):
		
		self._refbox_state_observer.update(msg.state, msg.payload)
				
		self._refbox_state = msg.state
		self._refbox_payload = msg.payload
		
		if self._refbox_state == RefBoxState.END:
			self._fsm.update(BmBoxState.END, None)
		
	
	def WaitClient(self):
		rospy.logdebug("BmBox.WaitClient()")
		
		if not self._fsm.check_state(BmBoxState.START): return
		
		rospy.loginfo("Waiting for client...")
		self._fsm.update(BmBoxState.WAITING_CLIENT)
		self._refbox_state_observer.wait_transition(RefBoxState.WAITING_CLIENT, RefBoxState.READY)
		self._fsm.update(BmBoxState.READY)
		
	def ManualOperation(self, message=None):
		rospy.logdebug("BmBox.ManualOperation()")
		
		if not ((self._fsm.state() == BmBoxState.READY) or (self._fsm.state() == BmBoxState.COMPLETED_MANUAL_OPERATION)):
			rospy.logerr("ManualOperation: expected to be in state BmBoxState.READY or BmBoxState.COMPLETED_MANUAL_OPERATION")
		
		if not (self._refbox_state == RefBoxState.READY):
			rospy.logerr("ManualOperation: refbox expected to be in state RefBoxState.READY")
		
		self._fsm.update(BmBoxState.WAITING_MANUAL_OPERATION, message)
		
		try:
			print "calling execute_manual_operation service: ", message
			execute_manual_operation = rospy.ServiceProxy('/execute_manual_operation', ExecuteManualOperation)
			manual_operation_payload = String()
			manual_operation_payload.data = message
			response = execute_manual_operation(manual_operation_payload)
			
			if response.result.data:
				self._refbox_state_observer.wait_transition(RefBoxState.READY, RefBoxState.EXECUTING_MANUAL_OPERATION)
				self._refbox_state_observer.wait_transition(RefBoxState.EXECUTING_MANUAL_OPERATION, RefBoxState.READY)
				
				self._fsm.update(BmBoxState.READY, None)
			
			else:
				rospy.logerr("ManualOperation: Manual operation FAILED (refbox refused to execute the manual operation)")
				self._fsm.update(BmBoxState.END, None)
				
		
		except rospy.ServiceException, e:
			rospy.logerr("Service call failed: %s"%e)
			return
		
		return self._refbox_payload
	
	def SendGoal(self, goal = "", timeout = 0):
		rospy.logdebug("BmBox.SendGoal()")
		
		if not (self._fsm.state() == BmBoxState.READY):
			rospy.logerr("SendGoal: expected to be in state BmBoxState.READY")
		
		if not (self._refbox_state == RefBoxState.READY):
			rospy.logerr("SendGoal: refbox expected to be in state RefBoxState.READY")
		
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
				self._refbox_state_observer.wait_transition(RefBoxState.READY, RefBoxState.TRANSMITTING_GOAL)
				self._refbox_state_observer.wait_transition(RefBoxState.TRANSMITTING_GOAL, RefBoxState.EXECUTING_GOAL)
				self._fsm.update(BmBoxState.EXECUTING_GOAL, goal)
			
			else:
				rospy.logerr("SendGoal: Goal request FAILED (refbox refused to execute the goal)")
				self._fsm.update(BmBoxState.END, None)
				
		except rospy.ServiceException, e:
			rospy.logerr("Service call failed: %s"%e)
			return
		
		return self._refbox_payload
	
	def WaitResult(self):
		rospy.logdebug("BmBox.WaitResult()")
		
		if not (self._fsm.state() == BmBoxState.EXECUTING_GOAL):
			rospy.logerr("WaitResult: expected to be in state BmBoxState.EXECUTING_GOAL")
		
		if not (self._refbox_state == RefBoxState.EXECUTING_GOAL):
			rospy.logerr("WaitResult: refbox expected to be in state RefBoxState.EXECUTING_GOAL")
		
		self._fsm.update(BmBoxState.WAITING_RESULT)
		self._refbox_state_observer.wait_transition(RefBoxState.EXECUTING_GOAL, RefBoxState.READY)
		self._fsm.update(BmBoxState.READY, None)
		
		return self._refbox_payload
	
	def SendScore(self, score):
		rospy.logdebug("BmBox.SendScore()")
		
		if not (self._fsm.state() == BmBoxState.READY):
			rospy.logerr("SendScore: expected to be in state BmBoxState.READY")
		
		if not (self._refbox_state == RefBoxState.READY):
			rospy.logerr("SendScore: refbox expected to be in state RefBoxState.READY")
		
		self._fsm.update(BmBoxState.TRANSMITTING_SCORE, score)
		
		
		try:
			print "calling end_benchmark service: ", score
			end_benchmark = rospy.ServiceProxy('/end_benchmark', EndBenchmark)
			end_benchmark_payload = String()
			end_benchmark_payload.data = score
			response = end_benchmark(end_benchmark_payload)
			
			if response.result.data:
				self._refbox_state_observer.wait_transition(RefBoxState.READY, RefBoxState.RECEIVED_SCORE)
				self._fsm.update(BmBoxState.END, None)
				
			else:
				rospy.logerr("SendScore: request FAILED (refbox refused to accept the score)")
				self._fsm.update(BmBoxState.END, None)
		
		except rospy.ServiceException, e:
			rospy.logerr("Service call failed: %s"%e)
			return
	
	def Timeout(self):
		rospy.logdebug("BmBox.Timeout()")
		
		return self._refbox_payload == "reason: timeout"
	
	def End(self):
		rospy.logdebug("BmBox.End()")
		
		self._fsm.update(BmBoxState.END)
	
	
	def Running(self):
		return self._refbox_payload != "reason: stop"
	
	def isBenchmarkEnded(self):
		return self._fsm.state() == BmBoxState.END or self._refbox_state == RefBoxState.END
	
	def __init__(self):
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
		
		print "BmBoxState:"
		print BmBoxState.START, "\tSTART"
		print BmBoxState.WAITING_CLIENT,"\tWAITING_CLIENT"
		print BmBoxState.READY,"\tREADY"
		print BmBoxState.WAITING_MANUAL_OPERATION,"\tWAITING_MANUAL_OPERATION"
		print BmBoxState.COMPLETED_MANUAL_OPERATION,"\tCOMPLETED_MANUAL_OPERATION"
		print BmBoxState.TRANSMITTING_GOAL,"\tTRANSMITTING_GOAL"
		print BmBoxState.EXECUTING_GOAL,"\tEXECUTING_GOAL"
		print BmBoxState.WAITING_RESULT,"\tWAITING_RESULT"
		print BmBoxState.TRANSMITTING_SCORE,"\tTRANSMITTING_SCORE"
		print BmBoxState.END,"\tEND"
		
		print "RefBoxState:"
		print RefBoxState.START, "\tSTART"
		print RefBoxState.WAITING_CLIENT,"\tWAITING_CLIENT"
		print RefBoxState.READY,"\tREADY"
		print RefBoxState.EXECUTING_MANUAL_OPERATION,"\tEXECUTING_MANUAL_OPERATION"
		print RefBoxState.TRANSMITTING_GOAL,"\tTRANSMITTING_GOAL"
		print RefBoxState.EXECUTING_GOAL,"\tEXECUTING_GOAL"
		print RefBoxState.RECEIVED_SCORE,"\tRECEIVED_SCORE"
		print RefBoxState.END,"\tEND"
		print RefBoxState.COMPLETED_MANUAL_OPERATION,"\tCOMPLETED_MANUAL_OPERATION"
		
		self._refbox_state = RefBoxState.START
		self._refbox_payload = ""

		rospy.init_node("benchmark")
		self._refbox_state_sub = rospy.Subscriber("refbox_state", RefBoxState, self._refbox_state_cb)
		self._state_pub = rospy.Publisher("bmbox_state", BmBoxState, queue_size=10)
		
		self._refbox_state_observer = StateObserver(RefBoxState.END)

		self._pub_thread = Thread(name="pub_thread", target=self._pub_thread)
		self._pub_thread.start()
		
