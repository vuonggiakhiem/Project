#!/usr/bin/env python3
import rospy
import tf
import math
from zlac8015d import ZLAC8015D
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

def main():
    rospy.init_node("move_1m_zlac8015d")

    # --- Thông số robot ---
    WHEEL_RADIUS = 0.063  # m
    WHEEL_BASE = 0.236    # m

    # --- Kết nối driver ---
    motors = ZLAC8015D.Controller(port="/dev/zlac8015d")
    motors.enable_motor()

    # --- ROS publisher và TF ---
    odom_pub = rospy.Publisher("/odom", Odometry, queue_size=50)
    odom_broadcaster = tf.TransformBroadcaster()

    # --- Biến trạng thái ---
    x = 0.0
    y = 0.0
    th = 0.0
    distance_traveled = 0.0
    target_distance = 1.0  # mét

    # --- Tốc độ mong muốn ---
    linear_speed = 0.2  # m/s → đi 1m trong khoảng 5 giây
    rpm_target = (linear_speed / (2 * math.pi * WHEEL_RADIUS)) * 60.0
    rpm_target_int = int(rpm_target)

    rospy.loginfo(f"🚗 rpm_target = {rpm_target_int} rpm (~{linear_speed} m/s)")

    # --- Gửi lệnh chạy thẳng ---
    motors.set_rpm(rpm_target_int, -rpm_target_int)

    last_time = rospy.Time.now()
    rate = rospy.Rate(50)  # 50 Hz

    while not rospy.is_shutdown() and distance_traveled < target_distance:
        rpmL, rpmR = motors.get_rpm()

        # --- Tính vận tốc thực tế ---
        vL = (rpmL * 2 * math.pi / 60.0) * WHEEL_RADIUS
        vR = -(rpmR * 2 * math.pi / 60.0) * WHEEL_RADIUS

        vx = (vR + vL) / 2.0
        vth = (vR - vL) / WHEEL_BASE

        current_time = rospy.Time.now()
        dt = (current_time - last_time).to_sec()

        # --- Cập nhật vị trí ---
        delta_x = vx * math.cos(th) * dt
        delta_y = vx * math.sin(th) * dt
        delta_th = vth * dt

        x += delta_x
        y += delta_y
        th += delta_th

        distance_traveled += math.sqrt(delta_x**2 + delta_y**2)

        # --- Gửi TF ---
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)
        odom_broadcaster.sendTransform(
            (x, y, 0.0),
            odom_quat,
            current_time,
            "base_link",
            "odom"
        )

        # --- Tạo & xuất bản odometry ---
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation = Quaternion(*odom_quat)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vth
        odom_pub.publish(odom)

        # --- In thông tin ---
        print(f"x={x:.3f} y={y:.3f} dist={distance_traveled:.3f} m | rpmL={rpmL:.1f} rpmR={rpmR:.1f}")

        last_time = current_time
        rate.sleep()

    # --- Dừng robot ---
    motors.set_rpm(0, 0)
    motors.disable_motor()
    rospy.loginfo("✅ Đã đi được 1 mét và dừng lại.")

if __name__ == "__main__":
    main()

