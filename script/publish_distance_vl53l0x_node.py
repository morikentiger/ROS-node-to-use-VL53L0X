#!/usr/bin/env python
import smbus
import time
import rospy
from std_msgs.msg import String,Float64

def makeuint16(lsb, msb):
    return ((msb & 0xFF) << 8)  | (lsb & 0xFF)

VL53L0X_REG_SYSRANGE_START			= 0x000

VL53L0X_REG_RESULT_INTERRUPT_STATUS 		= 0x0013
VL53L0X_REG_RESULT_RANGE_STATUS 		= 0x0014


address = 0x29
#address = 0x30

#bus = smbus.SMBus(1)
bus = smbus.SMBus(7)

# PublisherSettings

pub = rospy.Publisher('distance', Float64, queue_size=10)
rospy.init_node('VL53L0X', anonymous=True)
rate = rospy.Rate(10) # 10Hz

while not rospy.is_shutdown():#True:
	val1 = bus.write_byte_data(address, VL53L0X_REG_SYSRANGE_START, 0x01)
	#0x01->0x02

	
	cnt = 0
	while (cnt < 100): # 1 second waiting time max
		time.sleep(0.010)
		val = bus.read_byte_data(address, VL53L0X_REG_RESULT_RANGE_STATUS)
		if (val & 0x01):
			break
		cnt += 1

	'''if (val & 0x01):
		print "ready"
	else:
		print "not ready"
	'''

	data = bus.read_i2c_block_data(address, 0x14, 12)
	distance = float(makeuint16(data[11], data[10]))/1000
	#print "distance " + distance
	rospy.loginfo(distance)
	pub.publish(distance)
	rate.sleep()


