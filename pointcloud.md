# How to prepare data
1. Download and extract all sequences with both pointcloud and 3d pose
   ```
   python ./python/get_ptdb.py --seq_list ./python/kinoptic_3d_pose_list.txt
   ```

2. Generate pointclouds from multiple kinect views (**MATLAB**): run `matlab/get_pt_database`
