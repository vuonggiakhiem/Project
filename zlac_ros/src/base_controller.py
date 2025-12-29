#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf
import math
from zlac8015d import ZLAC8015D   # class bạn đã có

# ================================
# ⚙️ THÔNG SỐ ROBOT (SỬA THEO ROBOT BẠN)
# ================================
WHEEL_BASE = 0.236        # khoảng cách hai bánh
WHEEL_RADIUS = 0.063     # bán kính bánh (m)
ENCODER_CPR = 16385

# ================================
# 🔌 KẾT NỐI DRIVER
# ================================
drv = Controller("/dev/zlac8015d")
drv.set_mode(drv.VEL_CONTROL)
drv.enable_motor()

# ================================
# 📏 TRẠNG THÁI ODOM
# ================================
x = 0.0
y = 0.0
yaw = 0.0
last_time = rospy.Time.now()

# ================================
# 🟦 CMD_VEL → RPM ZLAC
# ================================
def cmd_callback(msg):
    v = msg.linear.x
    w = msg.angular.z

    # Tính rpm từ vận tốc
    wheel_circ = 2 * math.pi * WHEEL_RADIUS      # m / vòng
    v_r = (v + (w * WHEEL_BASE/2)) / wheel_circ  # vòng / sec
    v_l = (v - (w * WHEEL_BASE/2)) / wheel_circ

    rpm_r = v_r * 60.0
    rpm_l = v_l * 60.0

    # ZLAC cần rpm với scale ×0.1
    drv.set_rpm(int(-rpm_l * 10), int(rpm_r * 10))  # R âm vì hướng ngược

# ================================

# ================================
# 🚀 ROS NODE
# ================================
rospy.init_node("zlac_base_controller")

rospy.Subscriber("/cmd_vel", Twist, cmd_callback)
pub_odom = rospy.Publisher("/odom", Odometry, queue_size=20)
br = tf.TransformBroadcaster()

rate = rospy.Rate(50)
while not rospy.is_shutdown():
    update_odometry()
    rate.sleep()

