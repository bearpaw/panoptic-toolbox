close all; clear all; clc;

use_rgb = 0;

addpath('jsonlab');
addpath('plotcube');
addpath('kinoptic-tools');

%% parameter settings
root_path               = '/mnt/sdb/dataset/human/cmu_panoptic';
cam_id                  = [1, 2, 3, 4, 5];
% cam_id                  = [1, 2];
% cam_id                  = [1];
num_points              = 2048 * length(cam_id);
point_in_bbox_thresh    = 128 * length(cam_id);
if use_rgb
    out_path                = '/mnt/sdb/dataset/pointcloud/cmu-panoptic-view-rgb';
else
    out_path                = '/mnt/sdb/dataset/pointcloud/cmu-panoptic-view';
end

for cam = 1:length(cam_id)
    out_path = sprintf('%s-%d', out_path, cam_id(cam));
end

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
    gen_ptcloud_xyzrgb(root_path, out_path, data_seq{i}, num_points, cam_id, point_in_bbox_thresh, use_rgb);
end