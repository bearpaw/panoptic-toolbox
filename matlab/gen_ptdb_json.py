import os
import glob
import json

def parse_list_file(file_path):
    with open(file_path, 'r') as f:
        content = f.readlines()
    return [x.strip() for x in content]

def parse_list(data_dir, seq_list):
    dataset = []
    for seq in seq_list:
        print(seq)
        cur_dir = os.path.join(data_dir, seq, 'pose3d')
        cur_frames = sorted(glob.glob('%s/*.json' % cur_dir))
        for file in cur_frames:
            basename = os.path.basename(file)
            frame = int(os.path.splitext(basename)[0])
            dataset.append({
                'sequence': seq,
                'frame': frame
            })
    return dataset

def main():
    data_dir = '/mnt/sdb/dataset/pointcloud/cmu-panoptic-v2'
    train_file = 'single_person_point_pose_train.txt'
    valid_file = 'single_person_point_pose_valid.txt'
    test_file = 'single_person_point_pose_test.txt'

    train_list = parse_list_file(train_file)
    valid_list = parse_list_file(valid_file)
    test_list = parse_list_file(test_file)

    dataset = {}
    dataset['train'] = parse_list(data_dir, train_list)
    dataset['valid'] = parse_list(data_dir, valid_list)
    dataset['test'] = parse_list(data_dir, test_list)

    with open(os.path.join(data_dir, 'dataset.json'), 'w') as outfile:
        json.dump(dataset, outfile)

if __name__ == '__main__':
    main()
