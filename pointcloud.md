# How to prepare data
1. Download and extract all sequences with both pointcloud and 3d pose
   ```
   python ./python/get_ptdb.py --seq_list ./python/kinoptic_3d_pose_list.txt
   ```
2. Generate pointclouds from multiple kinect views (**MATLAB**):run `matlab/get_pt_database.m`
3. Generate `json` file for pytorch dataset
   ```
   python ./python/gen_ptdb_json.py \
    --data_dir '/mnt/sdb/dataset/pointcloud/cmu-panoptic-view-1-2-3-4-5' \
    --train_file ./config/train_list.txt \
    --valid_file ./config/valid_list.txt \
    --test_file ./config/test_list.txt
   ```
