Here we can see a scheme of the nodes and the data flow or the communications between them using topics:

![node_data_flow_scheme](doc/node_data_flow_scheme.png)

To execute this, you will need first to have an instance of gazebo running with a robot or something that publishes an image in the topic "/camera/raw_image" e.g. a turtlebot3 waffle model.

You can launch rviz2 too to see the images and debug information.

Once these steps have been done we could run the nodes one by one using the following commands:

Run the image filter node:
```
ros2 run follow_beacon image_color_filter --ros-args -p display_gui:=false
```

Run the centroid finder:
```
ros2 run follow_beacon operator_image
```

Run the image velocity publisher:
```
ros2 run follow_beacon sink_motors
```

If you want to adjust the color filter to match your beacon use the first command with the display_gui parameter to true, to display the color filter GUI:
```
ros2 run follow_beacon image_color_filter --ros-args -p display_gui:=true
```
