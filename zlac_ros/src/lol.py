#!/usr/bin/env python3
import rospy
import tf
import tf2_ros          # thêm để broadcast static TF
import math
from zlac8015d import ZLAC8015D
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped

def main():
    rospy.init_node("odometry_zlac8015d")

    # --- Thông số robot ---
    WHEEL_RADIUS = 0.063      # m
    WHEEL_BASE   = 0.259      # m

    # --- Kết nối driver ZLAC8015D ---
    motors = ZLAC8015D.Controller(port="/dev/zlac8015d")

    # --- Publisher & Broadcaster ---
    odom_pub = rospy.Publisher("/odom", Odometry, queue_size=50)
    
    # TF cho odom → base_link (dynamic)
    odom_broadcaster = tf.TransformBroadcaster()
    
    # TF tĩnh: base_link → laser (laser nằm trước 0.045m)
    static_broadcaster = tf2_ros.StaticTransformBroadcaster()

    # === Gửi static transform một lần duy nhất ===
    static_tf = TransformStamped()
    static_tf.header.stamp = rospy.Time.now()
    static_tf.header.frame_id = "base_link"
    static_tf.child_frame_id = "laser"
    
    static_tf.transform.translation.x = 0.16   # laser nằm trước 0.045m
    static_tf.transform.translation.y = 0.0
    static_tf.transform.translation.z = 0.0
    static_tf.transform.rotation.x = 0.0
    static_tf.transform.rotation.y = 0.0
    static_tf.transform.rotation.z = 0.0
    static_tf.transform.rotation.w = 1.0      # không quay

    static_broadcaster.sendTransform(static_tf)
    rospy.loginfo("Static TF base_link → laser (x=+0.045m) đã được publish!")

    # --- Biến odometry ---
    x = y = th = 0.0
    last_time = rospy.Time.now()
    rate = rospy.Rate(50)  # 50 Hz

    rospy.loginfo("Odometry + TF laser node started...")

    while not rospy.is_shutdown():
        try:
            # Đọc tốc độ bánh xe (rpm)
            rpmL, rpmR = motors.get_rpm()

            # Chuyển sang vận tốc tuyến tính (m/s) - chú ý dấu tùy cách đấu dây
            vL = (-rpmL * 2 * math.pi / 60.0) * WHEEL_RADIUS   # bánh trái thường ngược dấu
            vR = ( rpmR * 2 * math.pi / 60.0) * WHEEL_RADIUS

            # Tính vận tốc robot
            vx  = (vR + vL) / 2.0
            vth = (vR - vL) / WHEEL_BASE

            current_time = rospy.Time.now()
            dt = (current_time - last_time).to_sec()
            if dt <= 0: 
                dt = 0.01

            # Cập nhật vị trí
            delta_x  = vx * math.cos(th) * dt
            delta_y  = vx * math.sin(th) * dt
            delta_th = vth * dt

            x  += delta_x
            y  += delta_y
            th += delta_th
            th = math.atan2(math.sin(th), math.cos(th))  # chuẩn hóa góc

            # Quaternion từ yaw
            odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

            # === Gửi TF động: odom → base_link ===
            odom_broadcaster.sendTransform(
                (x, y, 0.0),
                odom_quat,
                current_time,
                "base_link",
                "odom"
            )

            # === Publish Odometry message ===
            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"

            odom.pose.pose.position.x = x
            odom.pose.pose.position.y = y
            odom.pose.pose.position.z = 0.0
            odom.pose.pose.orientation = Quaternion(*odom_quat)

            odom.twist.twist.linear.x = vx
            odom.twist.twist.angular.z = vth

            odom_pub.publish(odom)

            # Log ngắn gọn mỗi 1s
            rospy.loginfo_throttle(1.0,
                f"Pos: x={x:.3f} y={y:.3f} θ={math.degrees(th):.1f}° | "
                f"v={vx:.3f} m/s ω={vth:.3f} rad/s | rpmL={rpmL:+.0f} rpmR={rpmR:+.0f}"
            )

            last_time = current_time
            rate.sleep()

        except Exception as e:
            rospy.logwarn(f"Lỗi đọc ZLAC8015D: {e}")
            rate.sleep()

    rospy.loginfo("Node dừng, tắt motor an toàn...")
    motors.disable_motor()

if __name__ == "__main__":
    main()
