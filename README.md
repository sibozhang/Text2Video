# Text2Video
This is code for ICASSP 2022: "Text2Video: Text-driven Talking-head Video Synthesis with Phonetic Dictionary".
[Project Page](https://sites.google.com/view/sibozhang/text2video) 

[![ICASSP 2022: Text2Video]([https://res.cloudinary.com/marcomontalbano/image/upload/v1598335918/video_to_markdown/images/youtube--_pcqH1illCU-c05b58ac6eb4c4700831b2b3070cd403.jpg](https://i9.ytimg.com/vi/VGQ4pH1vlcA/mq1.jpg?sqp=CLCO75kG&rs=AOn4CLCQXN9KBqqHL-Vb5zJt6dPldOQsiw))]([https://www.youtube.com/watch?v=_pcqH1illCU&feature=youtu.be](https://www.youtube.com/watch?v=VGQ4pH1vlcA))

## Introduction
With the advance of deep learning technology, automatic video generation from audio or text has become an emerging and promising research topic. In this paper, we present a novel approach to synthesize video from text. The method builds a phoneme-pose dictionary and trains a generative adversarial network (GAN) to generate video from interpolated phoneme poses. Compared to audio-driven video generation algorithms, our approach has a number of advantages: 1) It only needs a fraction of the training data used by an audio-driven approach; 2) It is more ﬂexible and not subject to vulnerability due to speaker variation; 3) It signiﬁcantly reduces the preprocessing, training and inference time. We perform extensive experiments to compare the proposed method with state-of-the-art talking face generation methods on a benchmark dataset and datasets of our own. The results demonstrate the effectiveness and superiority of our approach.

## Data / Preprocessing

## Set up
1. Git clone repo
```
git clone git@github.com:sibozhang/Text2Video.git
```

2. Download and install modified vid2vid repo [vid2vid](https://github.com/sibozhang/vid2vid) 

3. Download Trained model

Please build 'checkpoints' folder in vid2vid folder and put trained model in it.

VidTIMIT fadg0 (English, Female) 

Dropbox: https://www.dropbox.com/sh/lk6et49v2uyfzjx/AADAFAp02_b3FQchaYxOZ0EMa?dl=0

百度云链接: https://pan.baidu.com/s/1SSkMKOK9LhClW2JvDCSiLg?pwd=bevj 提取码: bevj

Xuesong (Chinese, Male) 

Dropbox: https://www.dropbox.com/sh/qz3zoma5ac9mw5p/AAARiR8xKvATN4CBSyjWt_uOa?dl=0

百度云链接: 链接: https://pan.baidu.com/s/1DvuBbThYo4n5RIZsc-92rg?pwd=am7d 提取码: am7d

4. Prepare data and folder in the following order

    ```
    Text2Video
    ├── *phoneme_data
    ├── model
    ├── ...
    vid2vid
    ├── ...
    venv
    ├── vid2vid
    ```
5. Setup env 
```
sudo apt-get install sox libsox-fmt-mp3
pip install zhon
pip install moviepy
pip install ffmpeg
pip install dominate
pip install pydub
```

For Chinese, we use vosk to get timestamp of each words.
Please install vosk from https://alphacephei.com/vosk/install and unpack as 'model' in the current folder.
or install:

```
pip install vosk
pip install cn2an
pip install pypinyin
```

## Testing
1. Activate vitrual environment vid2vid
```
source ../venv/vid2vid/bin/activate
```
2. Generate video with real audio in English
```
sh text2video_audio.sh $1 $2
```

Generate video with TTS audio in English
```
sh text2video_tts.sh $1 $2 $3
```

Generate video with TTS audio in Chinese
```
sh text2video_tts.sh $1 $2 $3
```

$1: "input text"
$2: person
$3: fill f for female or m for male (gender)

Example 1. test VidTIMIT data with real audio.
```
sh text2video_audio.sh "She had your dark suit in greasy wash water all year." fadg0 f
```
    
Example 2. test VidTIMIT data with TTS audio.
```
sh text2video_tts.sh "She had your dark suit in greasy wash water all year." fadg0 f
```

Example 3. test with Chinese female TTS audio.
```
sh text2video_tts_chinese.sh "正在为您查询合肥的天气情况。今天是2020年2月24日，合肥市今天多云，最低温度9摄氏度，最高温度15摄氏度，微风。" henan f
```
    
## Training with your own data

English Phoneme / Chinese Pinyin model:

1. Modeling

1.1 Video recording:

Invite models to read all phoneme or pinyin, you can refer to prompts.docx or all_pinyin.txt. Pause 0.5 second between each pronunciation. Use the camera to record 1280x720 video.

1.2 Phoneme-Mouth/ Pinyin-Mouth Shape Dictionary: 

Use montreal-forced-aligner (google STT) for Phoneme/ vosk for pinyi to get timestamp of each words and put it in a dictionary file, which stores the data structure is as follows, each line saves a pair of [phoneme/ pinyin, frame number]:

phoneme, frame:
AA 52
AA0 52
AA1 52
AA2 52
AE 90
AE0 90
AE1 90
AE2 90
AH 127
AH0 127
AH1 127
AH2 127
AO 146
AO0 146
AO1 146
AO2 146
AW 227
AW0 227
AW1 227
AW2 227
...

pinyin, frame:
ba 61
bo 86
bi 540
bu 110
bai 130
bao 154
ban 178
bang 202
ou 225
pa 272
po 298
...

1.3 Openpose:

Use openpose to fit each frame of the video, find out the skeleton result of the human body, and save it to a separate folder. The code of openpose can be downloaded at:
https://github.com/CMU-Perceptual-Computing-Lab/openpose 
After the compilation is successful, run:
```
./build/examples/openpose/openpose.bin --image_dir ./images --face --hand --write_json ./keypoints
```

1.4 Use the generated human skeleton model and the corresponding video to train the vid2vid model, which is used to generate a realistic human skeleton model portrait video. The code for vid2vid can be downloaded from here: https://github.com/NVIDIA/vid2vid

Train:
```
python train.py --name xx --dataroot datasets/xx --dataset_mode pose --input_nc 3
--openpose_only --num_D 2 --resize_or_crop randomScaleHeight_and_scaledCrop
--loadSize 544 --fineSize 512 --gpu_ids 0,1,2,3,4,5,6,7 --batchSize 8
--max_frames_per_gpu 2 --niter 500 --niter_decay 5 --no_first_img --n_frames_total 12
--max_t_step 4 --niter_step 100 --save_epoch_freq 100 --add_face_disc
--random_drop_prob 0
```

2. Video generation

2.1 Generate audio files from text: 

Use Baidu TTS cloud service to generate the required voice. Baidu voice cloud service address:
http://wiki.baidu.com/pages/viewpage.action?pageId=342334101. Please refer to the specific call
tts_request.py

2.2 Analyze the audio and find out the time stamp of each text: 

use the Chinese version of VOSK speech recognition to process the speech generated by TTS Line recognition, and generate the following <frame number, pinyin> file. The VOSK code can be downloaded here: https://github.com/alphacep/vosk-api. For specific calls, please refer to pinyin_timestamping.py

25 xu
29 yao
38 zuo
46 Geng
53 jia
60 chong
65 fen
70 de
75 zun
80 bei
100 yin
105 ci
111 ne
116 wei
118 le

2.3 For each text in the audio, use its pinyin to find the corresponding 2D skeleton model, and splice it into a dynamic 2D skeleton. For video, intermediate frames are obtained by interpolation. Please refer to interp_landmarks_motion.py for details.

2.4 Use the vid2vid model to generate the final portrait video from the 2D skeleton video. The example is as follows:

```
CUDA_VISIBLE_DEVICES=1 python test.py --name xx --dataroot datasets/xx
--dataset_mode pose --input_nc 3 --resize_or_crop scaleHeight --loadSize 512
--openpose_only --how_many 1200 --no_first_img --random_drop_prob 0
```

## Citation
Please cite our paper in your publications.

Sibo Zhang, Jiahong Yuan, Miao Liao, Liangjun Zhang. [PDF](https://arxiv.org/pdf/2104.14631.pdf) [Result Video](https://youtu.be/TQJCyQ4ISEg)
```
@INPROCEEDINGS{9747380,  
author={Zhang, Sibo and Yuan, Jiahong and Liao, Miao and Zhang, Liangjun},  
booktitle={ICASSP 2022 - 2022 IEEE International Conference on Acoustics, Speech and Signal Processing (ICASSP)},   
title={Text2video: Text-Driven Talking-Head Video Synthesis with Personalized Phoneme - Pose Dictionary},   
year={2022},  
volume={},  
number={},  
pages={2659-2663},  
doi={10.1109/ICASSP43922.2022.9747380}
}
```
```
@article{zhang2021text2video,
  title={Text2Video: Text-driven Talking-head Video Synthesis with Phonetic Dictionary},
  author={Zhang, Sibo and Yuan, Jiahong and Liao, Miao and Zhang, Liangjun},
  journal={arXiv preprint arXiv:2104.14631},
  year={2021}
}
```

## Appendices
ARPABET
![](./ARPABET.png)

## Ackowledgements
This code is based on the vid2vid framework.
