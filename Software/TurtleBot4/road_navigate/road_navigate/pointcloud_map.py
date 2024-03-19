import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
import cv2
import struct
import math
from nav_msgs.msg import OccupancyGrid

class PointCloudSubscriber(Node):
    def __init__(self):
        super().__init__('point_cloud_subscriber')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/oakd/points',
            self.point_cloud_callback,
            rclpy.qos.QoSProfile(
                reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT, 
                depth=10
            )
        )

        
        self.map_publisher = self.create_publisher(OccupancyGrid, '/myRoad', 10)

        self.mapSizeForward = 2.5 #map size in meters
        self.mapSizeWidth = 5.0
        self.grid_resolution = 0.05  # 5cm per pixel
        self.robot_base_frame = "base_link"


    def point_cloud_callback(self, msg):

        base_grid = self.collect_points_into_grid(msg)
        
        avg_image = self.average_collected_points_in_grid(base_grid)

        filtered_image = self.filter_lines_ground(avg_image, 150)
        cohesion_road = self.road_post_processing(filtered_image)
        road = self.filter_lines_ground(cohesion_road, 170)

        self.map_publisher.publish(self.numpy_array_to_occupancy_grid(road))



    def numpy_array_to_occupancy_grid(self, array2d):

        occupancy_grid = OccupancyGrid()
        occupancy_grid.header.frame_id = self.robot_base_frame
        occupancy_grid.info.width = array2d.shape[1]
        occupancy_grid.info.height = array2d.shape[0]
        occupancy_grid.info.resolution = self.grid_resolution
        occupancy_grid.info.origin.position.x = 0.0
        occupancy_grid.info.origin.position.y = self.mapSizeWidth / 2
        occupancy_grid.info.origin.position.z = 0.0
        occupancy_grid.info.origin.orientation.x = 0.0
        occupancy_grid.info.origin.orientation.y = 0.0
        occupancy_grid.info.origin.orientation.z = 0.0
        occupancy_grid.info.origin.orientation.w = 1.0

        flattened_array = array2d.flatten()
        occupancy_grid.data = []
        for value in flattened_array:
            if value >= 100:
                occupancy_grid.data.append(100)  # 100 represents occupied
            else:
                occupancy_grid.data.append(value)    # 0 represents free space

        return occupancy_grid



    def collect_points_into_grid(self, msg):
        data = msg.data
        point_step = msg.point_step
        x_offset = msg.fields[0].offset
        y_offset = msg.fields[1].offset
        z_offset = msg.fields[2].offset
        intensity_offset = msg.fields[3].offset 

        grid_size_x = int(np.ceil((self.mapSizeWidth / self.grid_resolution)))
        grid_size_y = int(np.ceil((self.mapSizeForward / self.grid_resolution)))
        map_grid = np.zeros((grid_size_y, grid_size_x, 2))  # 2 channels: accumulated intensity and point count

        for i in range(0, len(data), point_step):
            x = struct.unpack_from('f', data, i + x_offset)[0]
            y = struct.unpack_from('f', data, i + y_offset)[0]
            z = struct.unpack_from('f', data, i + z_offset)[0]
            intensity = struct.unpack_from('f', data, i + intensity_offset)[0]

            self.add_points_for_road(x, y, z, intensity, map_grid)

        return map_grid


    def add_points_for_road(self, x,y,z,intensity, map_grid):
        
        if not (math.isnan(x) or math.isnan(y) or math.isnan(z) or math.isnan(intensity)):
            if(z < (self.mapSizeForward) and z > 0 and x < (self.mapSizeWidth / 2) and x > (-(self.mapSizeWidth / 2))):

                grid_x = int((x + self.mapSizeWidth / 2) / self.grid_resolution)
                grid_y = int((z) / self.grid_resolution)

                # Accumulate intensity and number of points gatheter in the corresponding grid cell
                map_grid[grid_y, grid_x, 0] += intensity
                map_grid[grid_y, grid_x, 1] += 1



    def average_collected_points_in_grid(self, map_grid):
        avg_image = np.zeros_like(map_grid[..., 0])
        mask = (map_grid[..., 1] != 0)
        avg_image[mask] = map_grid[..., 0][mask] / map_grid[..., 1][mask]

        return avg_image

    def filter_lines_ground(self, map, gray_scale_threshold):
        filtered_map = np.where(map < gray_scale_threshold, 0, map)
        return filtered_map

    def road_post_processing(self, map):
        # Define a kernel for dilation
        kernel = np.ones((5, 2), np.uint8)  # Adjust the kernel size based on your image and requirements

        # Apply dilation to connect white regions
        dilated_image = cv2.dilate(map, kernel, iterations=1)

        return dilated_image


def main(args=None):
    rclpy.init(args=args)
    point_cloud_subscriber = PointCloudSubscriber()
    rclpy.spin(point_cloud_subscriber)
    point_cloud_subscriber.destroy_node()
    rclpy.shutdown()
