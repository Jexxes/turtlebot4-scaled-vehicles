# Config Files

## Overview
1. [Road Navigation](#road-navigation)
    1. [PC (PointCloud)](#pc-pointcloud)
    2. [RTabMap](#rtabmap)


## Road Navigation

### PC (PointCloud)

The [Pointcloud yaml-file](../../../Software/TurtleBot4/road_navigate/config/pointcloud.yaml) is used for setting the parameters of the pointcloud. It's used in [thepointcloud.launch.py](../../../Software/TurtleBot4/road_navigate/launch/thepointcloud.launch.py). Note that the values are just example values set as we were trying to get data from the pointcloud node. 

| Parameter Group | Parameter Name | Value | Description |
| --- | --- | --- | --- |
| camera | i_enable_imu | false | Enable/Disable imu sensors |
| camera | i_enable_ir | false | Enable/Disable infrared LED for low-light environments |
| camera | i_floodlight_brightness | 0 | Brightness of IR LED |
| camera | i_laser_dot_brightness | 100 | Strength of active stereo (i.e for solid surfaces)|
| camera | i_nn_type | none | |
| camera | i_pipeline_type | RGBD | Which pipeline should be used for image processing/data interpretation |
| camera | i_usb_speed | SUPER_PLUS | |
| rgb | i_board_socket_id | 0 | |
| rgb | i_fps | 30.0 | FPS of RGB Camera |
| rgb | i_height | 720 | Height value for RGB Camera resolution |
| rgb | i_interleaved | false | |
| rgb | i_max_q_size | 10 | |
| rgb | i_preview_size | 250 | |
| rgb | i_enable_preview | true | |
| rgb | i_low_bandwidth | true | Low Bandwidth may be helpful for clogged networks. |
| rgb | i_keep_preview_aspect_ratio | true | |
| rgb | i_publish_topic | false | |
| rgb | i_resolution | '1080' | |
| rgb | i_width | 1280 | Width value for RGB Camera resolution |
| stereo | i_align_depth | true | |
| stereo | i_board_socket_id | 2 | |
| stereo | i_subpixel | true | |
| stereo | i_publish_right_rect | true | |
| ros__parameters | use_sim_time | false | |


### RTabMap 
The [RTabMap yaml-file](../../../Software/TurtleBot4/road_navigate/config/rtabmap.yaml) is used for setting the parameters of the pointcloud. It's used in [rtabmap.launch.py](../../../Software/TurtleBot4/road_navigate/launch/rtabmap.launch.py). Note that the values are just example values set as we were trying to get data from the RTabMap node. 

| Parameter Group | Parameter Name           | Value       | Description |
|-----------------|--------------------------|-------------|-------------|
| camera          | i_enable_imu             | false       |Enable/Disable imu sensors            |
| camera          | i_enable_ir              | false       |Enable/Disable infrared LED for low-light environments|
| camera          | i_floodlight_brightness  | 0           |             |
| camera          | i_laser_dot_brightness   | 100         |             |
| camera          | i_nn_type                | none        |             |
| camera          | i_pipeline_type          | RGBD        |             |
| camera          | i_usb_speed              | SUPER_PLUS  |             |
| rgb             | i_board_socket_id        | 0           |             |
| rgb             | i_fps                    | 30.0        |             |
| rgb             | i_height                 | 720         |             |
| rgb             | i_interleaved            | false       |             |
| rgb             | i_max_q_size             | 10          |             |
| rgb             | i_preview_size           | 250         |             |
| rgb             | i_enable_preview         | true        |             |
| rgb             | i_low_bandwidth          | false       |             |
| rgb             | i_low_bandwidth_quality  | 50          |             |
| rgb             | i_keep_preview_aspect_ratio | true    |             |
| rgb             | i_publish_topic          | true        |             |
| rgb             | i_resolution             | 1080P       |             |
| rgb             | i_width                  | 1280        |             |
| stereo          | i_subpixel               | true        |             |
| /oakd           | use_sim_time             | false       |             |

