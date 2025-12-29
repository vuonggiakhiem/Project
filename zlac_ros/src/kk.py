#!/usr/bin/env python3
import rospy
import tf
import math
import time
from zlac8015d import ZLAC8015D
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

def main():
    rospy.init_node("odometry_zlac8015d")

    # --- Thông số robot ---
    WHEEL_RADIUS = 0.1      # bán kính bánh xe (m)
    WHEEL_BASE = 0.45       # khoảng cách 2 bánh (m)

    # --- Kết nối driver ---
    motors = ZLAC8015D.Controller(port="/dev/zlac8015d")
    motors.disable_motor()
    motors.set_accel_time(1000, 1000)
    motors.set_decel_time(1000, 1000)
    motors.set_mode(3)  # chế độ tốc độ
    motors.enable_motor()

    # --- Publisher và TF broadcaster ---
    odom_pub = rospy.Publisher("/odom", Odometry, queue_size=50)
    odom_broadcaster = tf.TransformBroadcaster()

    # --- Biến trạng thái ---
    x = 0.0
    y = 0.0
    th = 0.0

    # --- Vận tốc mong muốn (có thể chỉnh) ---
    cmds = [-30, 30]   # rpm (trái, phải)
    motors.set_rpm(cmds[0], cmds[1])

    last_time = rospy.Time.now()
    rate = rospy.Rate(50)  # 50 Hz

    while not rospy.is_shutdown():
        try:
            # Đọc tốc độ thực từ 2 bánh (rpm)
            rpmL, rpmR = motors.get_rpm()

            # Chuyển đổi sang m/s
            vL = (rpmL * 2 * math.pi / 60.0) * WHEEL_RADIUS
            vR = (rpmR * 2 * math.pi / 60.0) * WHEEL_RADIUS

            # Tính vận tốc trung bình và góc quay
            vx = (vR + vL) / 2.0
            vth = (vR - vL) / WHEEL_BASE

            current_time = rospy.Time.now()
            dt = (current_time - last_time).to_sec()

            # Cập nhật vị trí robot
            delta_x = vx * math.cos(th) * dt
            delta_y = vx * math.sin(th) * dt
            delta_th = vth * dt

            x += delta_x
            y += delta_y
            th += delta_th

            # Tạo quaternion
            odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

            # --- Gửi transform (odom → base_link) ---
            odom_broadcaster.sendTransform(
                (x, y, 0.0),
                odom_quat,
                current_time,
                "base_link",
                "odom"
            )

            # --- Tạo thông điệp odometry ---
            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"

            odom.pose.pose.position.x = x
            odom.pose.pose.position.y = y
            odom.pose.pose.orientation = Quaternion(*odom_quat)

            odom.twist.twist.linear.x = vx
            odom.twist.twist.angular.z = vth

            # --- Xuất bản ---
            odom_pub.publish(odom)

            print(f"x={x:.3f} y={y:.3f} th={math.degrees(th):.2f}° | rpmL={rpmL:.1f} rpmR={rpmR:.1f}")

            last_time = current_time
            rate.sleep()

        except KeyboardInterrupt:
            motors.disable_motor()
            break

    motors.disable_motor()


if __name__ == "__main__":
    main()

