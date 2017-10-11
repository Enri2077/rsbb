from threading import Condition, Lock

import rospy

class StateObserver:

	def acquire(self):
		self._lock.acquire()

	def release(self):
		self._lock.release()

	def state(self):
		return self._state
	
	
	def wait_benchmark_state_transition(self, from_state=None, to_states=[], except_states=[]):
		self._condvar.acquire()
		
		while not rospy.is_shutdown()\
		  and not self._state:
			rospy.loginfo("StateObserver: refbox state has not been set yet")
			self._condvar.wait()
		
		rospy.loginfo("StateObserver:          waiting for transition from benchmark state %s to benchmark states %s, except states %s", from_state, to_states, except_states)
		
		# Go to sleep until the state is different from from_state
		while not rospy.is_shutdown()\
		  and from_state != None\
		  and self._state.benchmark_state == from_state\
		  and self._state.benchmark_state not in self._except_states\
		  and self._state.benchmark_state not in except_states:
			self._condvar.wait()
		
		# Go to sleep until the state is equal to to_states
		while not rospy.is_shutdown()\
		  and to_states != []\
		  and self._state.benchmark_state not in to_states\
		  and self._state.benchmark_state not in self._except_states\
		  and self._state.benchmark_state not in except_states:
			self._condvar.wait()
		
		rospy.logdebug("StateObserver: FINISHED waiting for transition from benchmark state", from_state, "to benchmark state", self._state.benchmark_state)
		
		self._condvar.release()



	def wait_goal_execution_state_transition(self, from_state=None, to_states=[], except_states=[]):
		self._condvar.acquire()
		
		while not rospy.is_shutdown()\
		  and not self._state:
			rospy.loginfo("StateObserver: refbox state has not been set yet")
			self._condvar.wait()
		
		rospy.loginfo("StateObserver:          waiting for transition from goal execution state %i to goal execution states %s, except states %s", from_state, to_states, except_states)
		
		# Go to sleep until the state is different from from_state
		while not rospy.is_shutdown()\
		  and from_state != None\
		  and self._state.goal_execution_state == from_state\
		  and self._state.benchmark_state not in self._except_states\
		  and self._state.goal_execution_state not in except_states:
			self._condvar.wait()
		
		# Go to sleep until the state is equal to to_state
		while not rospy.is_shutdown()\
		  and to_states != []\
		  and self._state.goal_execution_state not in to_states\
		  and self._state.benchmark_state not in self._except_states\
		  and self._state.goal_execution_state not in except_states:
			self._condvar.wait()
		
		rospy.logdebug("StateObserver: FINISHED waiting for transition from goal execution state", from_state, "to goal execution state", self._state.goal_execution_state)
		
		self._condvar.release()


	def wait_manual_operation_state_transition(self, from_state=None, to_states=[], except_states=[]):
		self._condvar.acquire()
		
		while not rospy.is_shutdown()\
		  and not self._state:
			rospy.loginfo("StateObserver: refbox state has not been set yet")
			self._condvar.wait()
		
		rospy.loginfo("StateObserver:          waiting for transition from manual operation state %i to manual operation states %s, except states %s", from_state, to_states, except_states)
		
		# Go to sleep until the state is different from from_state
		while not rospy.is_shutdown()\
		  and from_state != None\
		  and self._state.manual_operation_state == from_state\
		  and self._state.benchmark_state not in self._except_states\
		  and self._state.manual_operation_state not in except_states:
			self._condvar.wait()
		
		# Go to sleep until the state is equal to to_state
		while not rospy.is_shutdown()\
		  and to_states != []\
		  and self._state.manual_operation_state not in to_states\
		  and self._state.benchmark_state not in self._except_states\
		  and self._state.manual_operation_state not in except_states:
			self._condvar.wait()
		
		rospy.logdebug("StateObserver: FINISHED waiting for transition from manual operation state", from_state, "to manual operation state", self._state.manual_operation_state)
		
		self._condvar.release()



	def notify_condition_variables(self):
		self._condvar.acquire()
		self._condvar.notifyAll()
		self._condvar.release()

	def update(self, new_state):
		self._condvar.acquire()
		
		if new_state != self._state:
			rospy.loginfo("StateObserver: updated state to %s\n", new_state)
		
		self._state = new_state
		
		self._condvar.notify()
		self._condvar.release()
	
	
	def __init__(self, except_states, initial_state):
		rospy.logdebug("StateObserver init")
		self._state = initial_state
		self._except_states = except_states
		
		self._condvar = Condition()
		
		
