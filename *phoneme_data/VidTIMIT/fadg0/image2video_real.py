import cv2
import numpy as np
import glob
import moviepy.editor as mpe
import sys

#input
import re
import string


fps=25

#no smooth version
# anamelist=glob.glob('./results/{person}/test_latest/{test}/fake_B_*.jpg'.format(person=person, test=test))

anamelist=glob.glob('./images_fadg0/sa1_*.png')
anamelist.sort()

out = []
for a_name in anamelist:
    a_img = cv2.imread(a_name)
    a_height, a_width, layers = a_img.shape

    if out==[]:
        out=cv2.VideoWriter('sa1.mp4',
                        cv2.VideoWriter_fourcc(*'MP4V'), fps, (a_width, a_height))
    out.write(a_img)

if out != []:
    out.release()

#add audio
my_clip = mpe.VideoFileClip('./sa1.mp4')

# my_clip.write_videofile('./results/{person}/{person}_{test}.mp4'.format(person=person, test=test),
my_clip.write_videofile('./fadgo_sa1.mp4',
                        audio='./sa1.mp3')
