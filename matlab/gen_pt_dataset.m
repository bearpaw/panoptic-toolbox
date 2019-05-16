close all; clear all;

addpath('jsonlab');
addpath('plotcube');
addpath('kinoptic-tools');

%% parameter settings
root_path               = '/mnt/sdb/dataset/human/cmu_panoptic';
num_points              = 8192;
point_in_bbox_thresh    = 2048;
cam_id                  = [1, 2, 3, 4, 5];
out_path                = '/mnt/sdb/dataset/pointcloud/cmu-panoptic-view-' + string(mat2str(cam_id));

%% parse file
fid = fopen('multi_person_point_pose_all.txt');
data_seq = {}; line_cnt = 1;
tline = fgetl(fid);
while ischar(tline)
    disp(tline)
    data_seq{line_cnt} = tline;
    line_cnt = line_cnt + 1;
    tline = fgetl(fid);
end
fclose(fid);

%% generate data
for i = 1:length(data_seq)
    frames = gen_ptcloud_xyz(root_path, out_path, data_seq{i}, num_points, cam_id, point_in_bbox_thresh);
end