import os
import glob
import json
import argparse

def parse_args():
    parser = argparse.ArgumentParser(description='Panoptic data fetcher')
    parser.add_argument('--data_dir',
                        default='/mnt/sdb/dataset/pointcloud/cmu-panoptic-view-1-2-3-4-5',
                        help='path to save data')
    parser.add_argument('--train_file',
                        default='./config/full/train_list.txt',
                        help='train file')
    parser.add_argument('--valid_file',
                        default='./config/full/valid_list.txt',
                        help='valid file')
    parser.add_argument('--test_file',
                        default='./config/full/test_list.txt',
                        help='test file')
    parser.add_argument('--out_file',
                        default='/mnt/sdb/dataset/pointcloud/cmu-panoptic-view-1-2-3-4-5/multi_person_dataset.json',
                        help='output file')

    return parser.parse_args()

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
    args = parse_args()
    data_dir = args.data_dir
    train_file = args.train_file
    valid_file = args.valid_file
    test_file = args.test_file

    train_list = parse_list_file(train_file)
    valid_list = parse_list_file(valid_file)
    test_list = parse_list_file(test_file)

    dataset = {}
    dataset['train'] = parse_list(data_dir, train_list)
    dataset['valid'] = parse_list(data_dir, valid_list)
    dataset['test'] = parse_list(data_dir, test_list)

    with open(args.out_file, 'w') as outfile:
        json.dump(dataset, outfile)

if __name__ == '__main__':
    main()
