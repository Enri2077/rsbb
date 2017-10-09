from threading import Condition, Lock

import rospy

class StateObserver:

	def acquire(self):
		self._lock.acquire()

	def release(self):
		self._lock.release()

	def state(self):
		return self._state

	def payload(self):
		return self._payload
	
	def wait_transition(self, from_state, to_state):
		self._condvar.acquire()
		
		rospy.loginfo("StateObserver:          waiting for transition from state %i to state %i", from_state, to_state)
#		rospy.logdebug("StateObserver:          waiting for transition from state", from_state, "to state", to_state)
		
		# Go to sleep until the state is different from from_state
		while (from_state != None) and (self._state == from_state) and (self._state != self._end_state):
			self._condvar.wait()
		
		# Go to sleep until the state is equal to to_state
		while (to_state != None) and self._state != to_state and (self._state != self._end_state):
			self._condvar.wait()
		
		rospy.logdebug("StateObserver: FINISHED waiting for transition from state", from_state, "to state", to_state)
		
		self._condvar.release()


	def update(self, new_state, payload = None):
		self._condvar.acquire()
		
		if new_state != self._state:
			rospy.logdebug("StateObserver: updated state to", new_state)
		
		self._state = new_state
		self._payload = payload
		
		self._condvar.notify()
		self._condvar.release()
	
	
	def __init__(self, end_state):
		rospy.logdebug("StateObserver init")
		self._payload = ""
		self._state = None
		self._end_state = end_state
		
		self._condvar = Condition()
		
		
