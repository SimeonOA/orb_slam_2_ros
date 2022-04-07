# Client
docker run --rm -it -v /home/gdpmobile7/rgbd_dataset_freiburg1_xyz:/dataset -v $(pwd)/output:/output mjd3/orbslam-ros ros2 launch orb_slam2_ros orb_slam2_d435_rgbd_client_launch.py dataset:=/dataset compress:=0 fps:=20

# Node 
docker run -it --rm mjd3/orbslam-ros ros2 launch orb_slam2_ros orb_slam2_d435_rgbd_launch.py compress:=0

fps controls client-side publishing rate (can turn down if too fast). Results for processing time (proc_times.npy) and round-trip times (rt_times.npy) can be found in the specified output folder.
