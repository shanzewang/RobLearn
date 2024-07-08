#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

scan_realsense = None
publisher_scan_fusion = None

RANGE_OFFFSET = 0.03 # in Meter

def callback_scan_realsense(data):
    """
    Callback of the realsense subscriber. Save the laserscan to fused it with the hokuyo laserscan.
    :param data: LaserScan.
    :return:
    """
    global scan_realsense
    scan_realsense = data

def callback_scan_hokuyo(scan_hokuyo):
    """
    Callback of the hokuyo subscriber. Fused the laserscan from the realsense with the hokuyo laserscan. Insert into the
    hokuyo laserscan the realsense laserscan when the range is smaller. Published the fused laserscan.
    :param scan_hokuyo: Hokuyo laserscan.
    :return:
    """
    if scan_realsense is not None:

        scan_hokuyo_ranges = list(scan_hokuyo.ranges)

        index_hokuyo_start = int(len(scan_hokuyo.ranges) - len(scan_realsense.ranges)) / 2


        for index_realsense in range(len(scan_realsense.ranges)):
            index_hokuyo = index_hokuyo_start + index_realsense
            range_hokuyo = scan_hokuyo.ranges[index_hokuyo]
            range_realsense = scan_realsense.ranges[index_realsense] - RANGE_OFFFSET

            # if the laserscan range from the realsense smaler the hokuyo then take the hokuyo laserscan range.
            if range_realsense < scan_realsense.range_max and range_realsense < range_hokuyo:
                scan_hokuyo_ranges[index_hokuyo] = range_realsense

        scan_hokuyo.ranges = scan_hokuyo_ranges

    publisher_scan_fusion.publish(scan_hokuyo)



# 增加可读性：
def callback_scan(scan):
    """
    LiDAR订阅者的回调函数。
    融合来自相机和LiDAR的激光扫描数据。
    当相机激光扫描的距离较小时，将其替换LiDAR的对应数据。
    最后发布融合后的激光扫描数据。

    :param scan: LiDAR激光扫描数据 (LaserScan)
    :return: None
    """
    if scan_camera is not None:
        lidar_scan = list(scan.ranges)
        
        # 计算相机扫描数据的一半长度
        half_length_camera = len(scan_camera.ranges) // 2
        
        # 处理前半部分数据
        for i in range(half_length_camera):
            if scan_camera.ranges[half_length_camera + i] < scan_camera.range_max and scan_camera.ranges[half_length_camera + i] < scan.ranges[i]:
                lidar_scan[i] = scan_camera.ranges[half_length_camera + i]

        # 处理后半部分数据
        for i in range(half_length_camera):
            index_lidar = len(scan.ranges) - 1 - i
            if scan_camera.ranges[half_length_camera - i - 1] < scan_camera.range_max and scan_camera.ranges[half_length_camera - i - 1] < scan.ranges[index_lidar]:
                lidar_scan[index_lidar] = scan_camera.ranges[half_length_camera - i - 1]

        # 更新扫描数据
        scan.ranges = lidar_scan

        # 发布融合后的扫描数据
        publisher_scan_fusion.publish(scan)



def start():
    """
    Start the scan fusion node to fused the hokuyo and realsense laserscan.
    :return:
    """
    rospy.init_node('scan_fusion')

    global publisher_scan_fusion
    publisher_scan_fusion = rospy.Publisher('scan', LaserScan, queue_size=1)
    rospy.Subscriber("scan_hokuyo", LaserScan, callback_scan_hokuyo)
    rospy.Subscriber("scan_realsense", LaserScan, callback_scan_realsense)

    rospy.spin()

if __name__ == '__main__':
    start()
