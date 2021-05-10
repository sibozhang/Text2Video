# Text2Video
This is code for "Text2Video: Text-driven Talking-head Video Synthesis with Phonetic Dictionary".

## Introduction
With the advance of deep learning technology, automatic video generation from audio or text has become an emerging and promising research topic. In this paper, we present a novel approach to synthesize video from text. The method builds a phoneme-pose dictionary and trains a generative adversarial network (GAN) to generate video from interpolated phoneme poses. Compared to audio-driven video generation algorithms, our approach has a number of advantages: 1) It only needs a fraction of the training data used by an audio-driven approach; 2) It is more ﬂexible and not subject to vulnerability due to speaker variation; 3) It signiﬁcantly reduces the preprocessing, training and inference time. We perform extensive experiments to compare the proposed method with state-of-the-art talking face generation methods on a benchmark dataset and datasets of our own. The results demonstrate the effectiveness and superiority of our approach.

## Data / Preprocessing
    For Chinese, we use vosk to get timestamp of each words.
    Please download the model from https://github.com/alphacep/vosk-api/blob/master/doc/models.md and unpack as 'model' in the current folder.
    
## Set up
1. Download modified vid2vid repo [vid2vid](https://github.com/sibozhang/vid2vid) 

2. Prepare data and folder in the following order

    ```
    Text2Video
    ├── *phoneme_data
    ├── model
    ├── ...
    vid2vid
    ├── ...
    ```
    
## Testing
1. Activate vitrual environment venv_vid2vid
```
source venv_vid2vid/bin/activate
```
2. Generate video with real audio in English
```
sh text_to_video_audio.sh $1 $2
```

Generate video with TTS audio in English
```
sh text_to_video_tts.sh $1 $2 $3
```

Generate video with TTS audio in Chinese
```
sh text_to_video_tts.sh $1 $2 $3
```

$1: "input text"
$2: person
$3: fill f for female or m for male (gender)

Example 1. test VidTIMIT data with real audio
    ```
    sh text_to_video_audio.sh "She had your dark suit in greasy wash water all year." fadg0
    ```
    
Example 2. test VidTIMIT data with TTS audio
    ```
    sh text_to_video_tts.sh "She had your dark suit in greasy wash water all year." fadg0
    ```

Example 3. test with Chinese female TTS audio
    ```
    sh text2video_tts_chinese.sh "正在为您查询合肥的天气情况。今天是2020年2月24日，合肥市今天多云，最低温度9摄氏度，最高温度15摄氏度，微风。" henan f
    ```
    
## Training with your own data

## Citation
Please cite our paper in your publications.

```
@article{zhang2021text2video,
  title={Text2Video: Text-driven Talking-head Video Synthesis with Phonetic Dictionary},
  author={Zhang, Sibo and Yuan, Jiahong and Liao, Miao and Zhang, Liangjun},
  journal={arXiv preprint arXiv:2104.14631},
  year={2021}
}
```

## Ackowledgements
This code is based on the vid2vid framework.
