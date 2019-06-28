import os
from subprocess import call

if __name__ == '__main__':
    data_root = '/mnt/sdb/dataset/human/cmu_panoptic'
    fname = '../matlab/multi_person_point_pose_all.txt'
    with open(fname) as f:
        content = f.readlines()
    # you may also want to remove whitespace characters like `\n` at the end of each line
    content = [x.strip() for x in content]
    for seq_name in content:
        input_dir = os.path.join(data_root, seq_name, 'kinectVideos')
        output_dir = os.path.join(data_root, seq_name, 'kinectImgs')
        for p in [50]:
            for c in range(1, 11):
                video_name = os.path.join(input_dir, "kinect_%02d_%02d.mp4" % (p, c))
                image_dir = os.path.join(output_dir, "%02d_%02d"% (p, c))

                if os.path.isdir(image_dir):
                    print('{} has already been processed'.format(video_name))
                    continue

                os.makedirs(image_dir)
                print("Generate Images from {}".format(video_name))

                img_name = "%s/%02d_%02d_%%08d.jpg" % (image_dir, p, c)

                # Use ffmpeg to extract frames into a temporary directory
                call([
                    'ffmpeg',
                    '-nostats', '-loglevel', '0',
                    '-i', video_name,
                    '-q:v', '16',  # 2 - 31
                    '-f', 'image2',
                    '-start_number', '1',
                    img_name
                ])
    # 			print(img_name)
    # 			ffmpeg -i $videoFileName -q:v 1 -f image2 -start_number 1 "$fileName"  #1-based to be same as Matlab
    # 		else
    # 			echo "$videoFileName (File is missing: $videoFileName)"
    # 		fi
    # 	done
    # done
