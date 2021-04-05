import numpy as np
import keypoint2img
import cv2
import moviepy.editor as mpe
import json
import copy
import glob
import os
import shutil
import sys
import re
import string

#input
import re
import string
from zhon.hanzi import punctuation

#input dict table
person=sys.argv[2]
input=sys.argv[1]
stripped_space = re.sub(' ', '', input)
stripped_input = re.sub(r'[%s]+' %punctuation, '', stripped_space)

file_name = stripped_input[:10]
# print('file_name_interp', file_name)
test='tmp'

keypoints_dir='*phoneme_data/VidTIMIT/{person}/keypoints_{person}/'.format(person=person)

test_dir = '../vid2vid/datasets/{person}/'.format(person=person)

pose_dir = test_dir + 'test_openpose/{test}'.format(test=test)
img_dir= test_dir + 'test_img/{test}/'.format(test=test)

pose_smooth_dir= test_dir + 'test_openpose/{test}_smooth'.format(test=test)
img_smooth_dir= test_dir + 'test_img/{test}_smooth/'.format(test=test)

#test dict table
# word_ts = np.genfromtxt('./input_timestamp/{person}/{file_name}.txt'.format(person=person, file_name=file_name), dtype='str')

# #manual timestamp
# phoneme_ts = np.genfromtxt('./input_timestamp/{person}/phoneme/{file_name}.txt'.format(person=person, file_name=file_name), dtype='str')

#aligner timestamp
phoneme_ts = np.genfromtxt('./input_timestamp/{person}/phones/{file_name}.txt'.format(person=person, file_name=file_name), dtype='str')

phoneme_frame = np.genfromtxt('./*phoneme_data/VidTIMIT/{person}.txt'.format(person=person), dtype='str')

dict={}
phoneme_folder={}

# phoneme folder_name frame_number 
# ex. AA0 sa1 038
for p in phoneme_frame:
    dict[p[0]] = int(p[2])
    phoneme_folder[p[0]] = str(p[1])

first_didx = int(phoneme_ts[0][0])
last_didx = int(phoneme_ts[-1][0])
first_sidx = dict[phoneme_ts[0][1]]
last_sidx = dict[phoneme_ts[-1][1]]

first_sidx_name = phoneme_folder[phoneme_ts[0][1]]
last_sidx_name = phoneme_folder[phoneme_ts[-1][1]]

print('total_frame_num', last_didx)

total_frame_num = last_didx
motion_width = 3
transition_width = 5
min_key_dist = 4

#smooth
smooth_width = 4

#VidTIMIT input pic size
width=512 
height=384

kp_file = keypoints_dir + first_sidx_name + '_' + str(first_sidx).zfill(3)+'_keypoints.json'
#if dict
with open(kp_file) as f:
    json_data = json.loads(f.read())
for idx in range(0, first_didx):
    output_file = pose_dir + '/%05d.json' % idx
    with open(output_file, "w") as jsonFile:
        json.dump(json_data, jsonFile)

def interp_pose(js1, wt1, js2, wt2):
    inter_data = copy.deepcopy(js1)
    inter_dict = inter_data["people"][0]
    f1 = js1['people'][0]['face_keypoints_2d']
    f2 = js2['people'][0]['face_keypoints_2d']
    inter_dict['face_keypoints_2d'] = [x1 * wt1 + x2 * wt2 for x1, x2 in zip(f1, f2)]

    p1 = js1['people'][0]['pose_keypoints_2d']
    p2 = js2['people'][0]['pose_keypoints_2d']
    inter_dict['pose_keypoints_2d'] = [x1 * wt1 + x2 * wt2 for x1, x2 in zip(p1, p2)]

    return inter_data

#mouth center: based on opnepose keypoints_face.png
def mouth_center(pt_array):
    pt_list = pt_array.reshape((70, 3))
    cpt = np.average(pt_list[48:60, :], axis=0)
    return cpt

def mouth_shift(pt_array, offset):
    for i in range(48, 68):
        pt_array[i*3] = pt_array[i*3] + offset[0]
        pt_array[i * 3 + 1] = pt_array[i * 3 + 1] + offset[1]

    return pt_array


interp_json_data = copy.deepcopy(json_data)
interp_dict = interp_json_data["people"][0]
idx = 0
while idx < len(phoneme_ts)-1:
    # print(idx)
    didx1 = int(phoneme_ts[idx][0])
    sidx1 = dict[phoneme_ts[idx][1]]
    sidx1_name = phoneme_folder[phoneme_ts[idx][1]]

    didx2 = int(phoneme_ts[idx+1][0])
    if didx2 - didx1 >= min_key_dist:
        didx2 = int(phoneme_ts[idx + 1][0])
        sidx2 = dict[phoneme_ts[idx+1][1]]
        sidx2_name = phoneme_folder[phoneme_ts[idx+1][1]]
        idx = idx + 1

    elif idx == len(phoneme_ts)-2:
        didx2 = int(phoneme_ts[idx + 1][0])
        sidx2 = dict[phoneme_ts[idx+1][1]]
        sidx2_name = phoneme_folder[phoneme_ts[idx+1][1]]
        idx = idx + 2

    else:
        print("skip %d" % didx2)
        didx2 = int(phoneme_ts[idx + 2][0])
        sidx2 = dict[phoneme_ts[idx + 2][1]]
        sidx2_name = phoneme_folder[phoneme_ts[idx+2][1]]
        idx = idx + 2

    interval_len = float(didx2 - didx1)
    inter_frame_num = interval_len - 1

    #ramp overlapping weighted sum when interval short
    if inter_frame_num<2*motion_width+transition_width:
        for n in range(didx1, didx2+1):
            w2 = float(n - didx1)/ interval_len
            w1 = 1.0 - w2

            kp_file1 = keypoints_dir + sidx1_name + '_' + str(sidx1+n-didx1).zfill(3) + '_keypoints.json'
            with open(kp_file1) as f:
                json_data1 = json.loads(f.read())
            kp_file2 = keypoints_dir + sidx2_name + '_' + str(sidx2+n-didx2).zfill(3) + '_keypoints.json'
            with open(kp_file2) as f:
                json_data2 = json.loads(f.read())

            f1 = json_data1['people'][0]['face_keypoints_2d']
            f2 = json_data2['people'][0]['face_keypoints_2d']
            interp_dict['face_keypoints_2d'] = [x1*w1+x2*w2 for x1,x2 in zip(f1, f2)]

            p1 = json_data1['people'][0]['pose_keypoints_2d']
            p2 = json_data2['people'][0]['pose_keypoints_2d']
            interp_dict['pose_keypoints_2d'] = [x1 * w1 + x2 * w2 for x1, x2 in zip(p1, p2)]

            output_file = pose_dir + '/%05d.json' % n
            # output_file = 'C:/Baidu/RealtimeVirtualAssist/results/landmark_interp/%06d.json' % n
            with open(output_file, "w") as jsonFile:
                json.dump(interp_json_data, jsonFile)

    #motion ramp direct copy + weighted sum in the middle
    else:
        print("connecting interp %d - %d"%(didx1, didx2))
        for n in range(didx1, didx1 + motion_width+1):
            kp_file1 = keypoints_dir + sidx1_name + '_' + str(sidx1 + n - didx1).zfill(3) + '_keypoints.json'        
            with open(kp_file1) as f:
                json_data1 = json.loads(f.read())
            output_file = pose_dir + '/%05d.json' % n
            with open(output_file, "w") as jsonFile:
                json.dump(json_data1, jsonFile)

        for n in range(didx2, didx2-motion_width-1, -1):
            kp_file2 = keypoints_dir + sidx2_name + '_' + str(sidx2 + n - didx2).zfill(3) + '_keypoints.json'
            with open(kp_file2) as f:
                json_data2 = json.loads(f.read())
            output_file = pose_dir + '/%05d.json' % n
            with open(output_file, "w") as jsonFile:
                json.dump(json_data2, jsonFile)

        intv_len = didx2-motion_width - (didx1+motion_width)
        for n in range(didx1+motion_width+1, didx2-motion_width):
            w2 = float(n - (didx1+motion_width))/float(intv_len)
            w1 = 1.0 - w2
            inter_js = interp_pose(json_data1, w1, json_data2, w2)
            output_file = pose_dir + '/%05d.json' % n
            with open(output_file, "w") as jsonFile:
                json.dump(inter_js, jsonFile)

kp_file = keypoints_dir + last_sidx_name + '_' +str(last_sidx).zfill(3) +'_keypoints.json'
with open(kp_file) as f:
    json_data = json.loads(f.read())
for idx in range(last_didx+1, total_frame_num):
    output_file = pose_dir + '/%05d.json' % idx
    with open(output_file, "w") as jsonFile:
        json.dump(json_data, jsonFile)

#output skeleton images
flist = glob.glob(pose_dir + '/*.json'.format(person=person))
flist.sort()

idx = -1
for f in flist:
    # print('f', f)
    idx = idx + 1
    kp_img = keypoint2img.read_keypoints(f, (width, height))
    out_nm = img_dir + str(idx).zfill(4) + '.jpg'
    cv2.imwrite(out_nm, kp_img)

#temporal smooth of face (68 keypoints) and pose, to reduce mouth "jump"
jsonlist = []
for f in flist:
    with open(f) as fid:
        json_data = json.loads(fid.read())
        jsonlist.append(copy.deepcopy(json_data))

for idx in range(len(jsonlist)):
    sum_w = 0.0
    sum_fc = np.zeros((1, 210), dtype=float)
    sum_ps = np.zeros((1, 75), dtype=float)
    # sum_nx = 0.0
    # sum_ny = 0.0
    for s in range(-smooth_width, smooth_width):
        sidx = s + idx
        if 0<=sidx and sidx<len(jsonlist):
            json_data = jsonlist[sidx]
            fc = np.asarray(json_data['people'][0]['face_keypoints_2d'])
            ps = np.asarray(json_data['people'][0]['pose_keypoints_2d'])
            wt = 1.0/(abs(s) + 1.0)
            sum_fc += fc * wt
            sum_ps += ps * wt
            sum_w += wt

    ave_fc = sum_fc/sum_w
    ave_ps = sum_ps/sum_w

    orig_fc = np.asarray(jsonlist[idx]['people'][0]['face_keypoints_2d'])
    c_t = mouth_center(ave_fc)
    c_s = mouth_center(orig_fc)
    orig_fc=mouth_shift(orig_fc, c_t - c_s)
    ave_fc[0, 48*3:68*3] = orig_fc[48*3:68*3]


    jsonlist[idx]['people'][0]['face_keypoints_2d'] = ave_fc.tolist()
    jsonlist[idx]['people'][0]['pose_keypoints_2d'] = ave_ps.tolist()

    output_file = pose_smooth_dir + '/smooth_%05d.json' % idx
    with open(output_file, "w") as jsonFile:
        json.dump(jsonlist[idx], jsonFile)

    kp_img = keypoint2img.read_keypoints(output_file, (width, height))
      
    out_nm = img_smooth_dir + 'smooth_'+str(idx).zfill(4) + '.jpg'
    cv2.imwrite(out_nm, kp_img)

