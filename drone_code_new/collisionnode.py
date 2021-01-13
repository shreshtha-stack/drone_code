import rospy
import serial
node_name='uav0'
from mavros_test_common_1 import Mavros_test_common
from mavros_msgs.msg import PositionTarget
from threading import Thread
class main_class(Mavros_test_common):
	def setUp(self):
		super(main_class).setUp()
		
		#Setpoint Updation Thread
		self.pos_data= PositionTarget()
		self.pos_pub=rospy.Publisher(self.uavstring+'setpoint_raw/target_local',PositionTarget,queue_size=10)
		self.pos_thread=Thread(target=self.send_pos,args=())
		self.pos_thread.daemon=True
		self.pos_thread.start()
		self.pos_thread_stop=False
		
		#Communication Thread
		self.serial_port=serial.Serial('/dev/ttyUSB0',57600,timeout=0.1)
		if self.serial_port.isOpen():
			self.serial_port_status==True			
		self.com_thread=Thread(target=self.com_func,args=())
		self.com_thread.daemon=True
		self.com_thread.start()
		
	def com_func(self):
		com_loop_rate=1 # Communication Rate
		rate1=rospy.Rate(com_loop_rate)
		while not rospy.is_Shutdown():
			try:
				if self.serial_port_status==True:
					try:
						rospy.loginfo('Module is ready to communicate')
					except rospy.ROSException as e:
						pass
				else:
					Connection_retries_max=3
					retry_number=1
					while retry_number<=Connection_retries_max:
						self.serial_port=serial.Serial('/dev/ttyUSB0',57600,timeout=0.1)
						if self.serial_port.isOpen():
							self.serial_port_status==True
							rospy.loginfo('Connection Successful on retry {}'.format(retry_number))
							break
						retry_number+=1
					if retry_number==3:
						rospy.loginfo('Connection Unsuccessful.Check the module')

			except rospy.ROSException as e:
				rospy.loginfo('Issue in communication thread.Error encountered:{}'.format(e))
				

	def send_pos(self):
		loop_rate= 20                     					#Setpoint publishing rate
		rate=rospy.Rate(loop_rate)
		while self.pos_thread_stop==True :
			try:
				self.pos_data.header.stamp=rospy.Time.now()
				self.pos_pub.publish(self.pos_data)
				rate.sleep()
			except rospy.ROSException as e:
				rospy.loginfo('Issues sending position setpoints')
				rospy.fail(e)
				
	
	def test_method(self):
		self.obtain_mav_sys_id(30)
		self.wait_for_gps_lock(60)
		self.set_arm(True, 60)
		self.takeoff_func()
		self.set_mode_function("")
		#switch to takeoff
		#takeoff to offboard switch after reaching a certain altitude
		
		
		

if __name__=="__main__":
	import rostest
	rospy.init_node(node_name,anonymous=True)
	rostest.rosrun('formationcontrol_iitb','collisionnode',main_class)
