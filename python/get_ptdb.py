"""
Python script to fetch the Kinoptic dataset (CMU pointcloud db)
Sequences wiouth 3D human pose annotations will be filtered out
Wei Yang (platero.yang@gmail.com)
"""

import subprocess
import os
import argparse
from os.path import join
from tqdm import tqdm

def args():
    parser = argparse.ArgumentParser(description='Panoptic data fetcher')
    parser.add_argument('--data_dir',
                        default='./data',
                        help='path to save data')
    parser.add_argument('--seq_list',
                        default='./python/kinoptic_list.txt',
                        help='list of sequences to be download')
    parser.add_argument('--url_base',
                        default='http://domedb.perception.cs.cmu.edu/webdata/dataset',
                        help='base url')
    parser.add_argument('--download_video',
                        action='store_true',
                        help='download all video?')

    return parser.parse_args()

def download(url='', dst=''):
    rc = subprocess.call(['wget', '-c', '-O', dst, url])
    # rc = subprocess.call(['wget', '-c', '-q', '-O', dst, url])
    if os.stat(dst).st_size == 0:
        os.system('rm -rf {}'.format(dst))
        return False
    return True if rc == 0 else False

def extract(src, directory, is_video=False):
    if is_video:
        pass
    else:
        rc = subprocess.call(['tar', 'xf', src, '-C', directory])
        rc = True if rc == 0 else False
    return rc

def get_sequence(url_base, data_dir, seq):
    # create dir
    seq_dir = join(data_dir, seq)
    os.makedirs(seq_dir, exist_ok=True)

    # Download 3D pose reconstruction results (by vga index, coco19 format)
    success = download(
        url='{0}/{1}/hdPose3d_stage1_coco19.tar'.format(url_base, seq),
        dst='{}/hdPose3d_stage1_coco19.tar'.format(seq_dir, seq)
    )

    if success:
        # 3D face
        download(
            url='{0}/{1}/hdFace3d.tar'.format(url_base, seq),
            dst='{}/hdFace3d.tar'.format(seq_dir, seq)
        )

        # 3D hand
        download(
            url='{0}/{1}/hdHand3d.tar'.format(url_base, seq),
            dst='{}/hdHand3d.tar'.format(seq_dir, seq)
        )

        # Download calibration and sync data
        download(
            url='{0}/{1}/calibration_{1}.json'.format(url_base, seq),
            dst='{}/calibration_{}.json'.format(seq_dir, seq)
        )
        download(
            url='{0}/{1}/kinect_shared_depth/kcalibration_{1}.json'.format(url_base, seq),
            dst='{}/kcalibration_{}.json'.format(seq_dir, seq)
        )
        download(
            url='{0}/{1}/kinect_shared_depth/synctables.json'.format(url_base, seq),
            dst='{}/synctables_{}.json'.format(seq_dir, seq)
        )
        download(
            url='{}/{}/kinect_shared_depth/ksynctables.json'.format(url_base, seq),
            dst='{}/ksynctables_{}.json'.format(seq_dir, seq)
        )

        # Download kinect rgb videos
        kv_dir = join(seq_dir, 'kinectVideos')
        os.makedirs(kv_dir, exist_ok=True)
        for i in range(1, 11):
            dst = '{}/kinect_50_{:02d}.mp4'.format(kv_dir, i)
            url = '{0}/{1}/videos/kinect_shared_crf20/{1}_kinect{2}.mp4'.format(
                url_base, seq, i
            )
            download(url=url, dst=dst)

        # Download kinect depth videos
        for i in range(1, 11):
            cur_dir = join(seq_dir, 'kinect_shared_depth', 'KINECTNODE{}'.format(i))
            os.makedirs(cur_dir, exist_ok=True)
            dst = '{}/depthdata.dat'.format(cur_dir, i)
            url = '{}/{}/kinect_shared_depth/KINECTNODE{}/depthdata.dat'.format(
                url_base, seq, i
            )
            download(url=url, dst=dst)
    else:
        os.system('rm -rf {}'.format(seq_dir))


def extract_sequence(data_dir, seq):
    seq_dir = join(data_dir, seq)

    # 3d body
    src = join(data_dir, seq, 'hdPose3d_stage1_coco19.tar')
    if os.path.isfile(src):
        extract(src, seq_dir)

    # 3d face
    src = join(data_dir, seq, 'hdFace3d.tar')
    if os.path.isfile(src):
        extract(src, seq_dir)

    # 3d hand
    src = join(data_dir, seq, 'hdHand3d.tar')
    if os.path.isfile(src):
        extract(src, seq_dir)


def get_all_data(conf):
    print('Download all data')
    os.makedirs(conf.data_dir, exist_ok=True)
    with open(conf.seq_list) as seq_list:
        seq_list = [line.rstrip() for line in seq_list]

    for seq in tqdm(seq_list, ascii=True):
        get_sequence(conf.url_base, conf.data_dir, seq)

def extract_all_data(conf):
    print('Extract all data')
    with open(conf.seq_list) as seq_list:
        seq_list = [line.rstrip() for line in seq_list]

    for seq in tqdm(seq_list, ascii=True):
        extract_sequence(conf.data_dir, seq)

if __name__ == '__main__':
    conf = args()
    get_all_data(conf)
    extract_all_data(conf)
