sicks300
========

ROS package for reading continuous data output of the SICK S300 Professional laser scanner via RS-422. Note that it supports both the old (v.1.02) and the new (v.1.03) protocol.

Please note that this is an extension of the original "fuerte" driver that was developed by the Autonomous Intelligent Systems Department at Bonn University.

Getting started
===============

Make sure that all dependencies are satisfied:

    $ rosmake sicks300 sicks300

Run the node:

    $ rosrun sicks300 sick300_driver

Messages
========

Published Topics
----------------

* `/laserscan (sensor_msgs/LaserScan)`

    Scan data 

Parameters
----------

* `~frame (String, default: "base_laser_link")`

    Laser frame 

* `~send_transform (int, default: 1)`

    Send Transform from base_link to base_laser_link. 

* `~tf_x (double, default: 0.115)`

    Transformation along x-axis 

* `~tf_y (double, default: 0.0)`

    Transformation along y-axis 

* `~tf_z (double, default: 0.21)`

    Transformation along z-axis 

* `~field_of_view (int, default: 270)`

    The field of view to publish. The maximum view angle is 270 degrees, setting this to a value lower than 270 will remove the left and right sides of the field of view. 

* `~devicename (String, default: "/dev/sick300")`

    Port connected to laser. 

* `~baudrate (int, default: 500000)`

    Sets the baud rate
