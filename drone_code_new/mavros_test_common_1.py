import unittest
import rospy
uavstring=''
#from geometry_msgs.msg import float32
from std_msgs.msg import UInt32, Float32
from mavros_msgs.msg import State
from mavros_msgs.srv import ParamGet,SetMode, CommandBool, CommandTOL
class Mavros_test_common(unittest.TestCase):
	def setUp(self):
		super(Mavros_test_common).__init__()
		
		#ROS_topics
		self.current_state=State()
		self.current_state_sub=rospy.Subscriber(self.uavstring+'/mavros/State',State,self.current_state_callback)
		self.gps_satellite = UInt32()
		self.gps_lock_sub=rospy.Subscriber(self.uavstring+'/mavros/global_position/raw/satellites',UInt32,self.gps_lock_func)
		self.amsl_pos = Float32()
		self.altitude_sub=rospy.Subscriber(self.uavstring+'/mavros/altitude',Float32,self.altitude_callback)	
		
		
		#ROS_Services
		try :
			rospy.wait_for_service(self.uavstring+'/mavros/param/get')
			rospy.wait_for_service(self.uavstring+'/mavros/set_mode')
			rospy.wait_for_service(self.uavstring+'/mavros/cmd/arming')
			rospy.wait_for_service(self.uavstring+'/mavros/cmd/arming')
			rospy.wait_for_service(self.uavstring+'/mavros/cmd/takeoff')
			
		except rospy.ROSException as e:
			rospy.loginfo('Services are not up.Error encountered :{}'.format(e))
			
			
		self.param_srv=rospy.ServiceProxy(self.uavstring+'/mavros/param/get',ParamGet)
		self.setmode_srv=rospy.ServiceProxy(self.uavstring+'/mavros/set_mode',SetMode)
		self.arming_srv=rospy.ServiceProxy(self.uavstring+'/mavros/cmd/arming',CommandBool)
		self.takeoff_srv=rospy.ServiceProxy(self.uavstring+'/mavros/cmd/takeoff',CommandTOL)
		
	def current_state_callback(self,data):
		self.previous_state=self.current_state
		self.current_state=data.mode
		if self.previous_state != self.current_state:
			rospy.loginfo("Mode changed from {1} to {2}".format(self.previous_state,self.current_state))
			
	def obtain_mav_sys_id(self,timeout):
		t=0 
		rate=rospy.Rate(1)
		while t<=timeout:
			try:
				res = self.param_srv('MAV_SYS_ID')
				if res.success:
					self.mav_sys_id=res.value.integer
					rospy.loginfo("MAV_SYS_ID obtained:{}".format(self.mav_sys_id))
					break
			except rospy.ServiceException as e:
                		rospy.logerr(e)
			try:
				rate.sleep()
			except rospy.ROSException as e:
				self.fail(e)
			t+=1
		if res.success==False:
			self.fail("Process Timeout: Couldnot obtain mavsys ID")
		res.success==False
		
	def set_mode_function(self,mode,timeout):
		t=0 
		rate=rospy.Rate(1)
		while t<=timeout:
			try:
				res = self.setmode_srv(0,mode)
				if res.success:
					rospy.loginfo("Attempt to set mode to:{}".format(mode))
					break
			except rospy.ServiceException as e:
                		rospy.logerr(e)
			try:
				rate.sleep()
			except rospy.ROSException as e:
				self.fail(e)
			t+=1
			
	
	def set_arm(self, arm, timeout):
		previous_arm = self.current_state.armed
		loop_freq = 1  #1 Hz
		rate = rospy.Rate(loop_freq)
		arm_set = False 
		for i in range(timeout*loop_freq):
			if self.current_state.armed == arm:
				arm_set == True
				rospy.loginfo("Set arm sucess")
				break 
			else:
				try:
					res = self.arming_srv(arm)
					if not res.sucess:
						rospy.loginfo("failed to send arm")						
				except rospy.ServiceException as e:
					rospy.logerr(e)


	def gps_lock_func(self,data):
		self.gps_satellite = data
		
	def wait_for_gps_lock(self, timeout):
		
		loop_freq = 1  #1 Hz
		rate = rospy.Rate(loop_freq)
		gps_lock_flag = True
		for i in range(timeout*loop_freq):
			try :
				if self.gps_satellite > 10:
					rospy.loginfo("GPS is LOCKED")
					gps_lock_flag = True
					break 
			except rospy.ROSException as e:
					self.fail(e) 
			try:
				rate.sleep()
			except rospy.ROSException as e:
				self.fail(e)

	def takeoff_func(self,takeoff_alt, timeout):

		loop_freq = 1  #1 Hz
		rate = rospy.Rate(loop_freq)
		initial_takeoff_altitude = 10 

		for i in range(timeout*loop_freq):
			try :
				if self.amsl_pos == takeoff_alt :
					rospy.loginfo("Took Off")
					takeoff_flag = True
					break 
			except rospy.ROSException as e:
					self.fail(e) 
			try:
				rate.sleep()
			except rospy.ROSException as e:
				self.fail(e)

	def altitude_callback(self,data):
		self.amsl_pos = data