# my_util_scripts

A collection of ROS scripts with functionality that I frequently use.


## What's Included

### current_time.py

#### What it does

Publishes the current ROS time (retrieved via `rospy.Time.now()`) and
publishes it as a `jsk_rviz_msgs/OverlayText` in a human-readable format.

#### What I use it for

When I conduct experiments using actual robots, I often record video footages
and show a clock at the beginning of the video. This way I can later
synchronize the videos and also see the corresponding log data from a bag
file.

However, it is sometimes hard to tell the ROS time because it's in UNIX time
stamp. With this script I know exactly when the current ROS time is. As a
bonus, showing the current time in rviz via OverlayText makes it easier to
time-sync with other videos when you capture the screen.


### pub_mesh.py

#### What it does

Publishes a mesh file (stl, obj, and other ROS-supported types) as a
`visualization_msgs/MarkerArray` message for display in rviz.

#### What I use it for

The most general use case for me is to publish the stl mesh file of the
environment. For example, I could compare the point cloud data from a 3D LiDAR
against the stl file that represents the environment and see how much they
differ.


### tf_static_from_bag.py

#### What it does

Given a bag file, this script publishes the `/tf_static` topic (to a latched
topic).

#### What I use it for

I use this when replaying log data taken from an experiment. Most of the times
I want to start playing the log file after some time into it (`rosbag play
/path/to/bag -s 42` will start playing from the 42-second mark).

However, if I do that the `tf_static` data does not get published, and some
important transforms such as fixed links of the robot would be missing (i.e. I
get a disconnected TF tree). To solve this problem, I created this script
which reads the bag file and collects all static transforms (published to
`/tf_static`) and publishes them to the `/tf_static` topic.


## License

MIT


## Author

Naoki Mizuno (naoki.mizuno.256@gmail.com)
