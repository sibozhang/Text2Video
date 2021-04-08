# Text2Video
This is code for "Text2Video: Text-driven Talking-head Video Synthesis with Phonetic Dictionary".

## Introduction
With the advance of deep learning technology, automatic video generation from audio or text has become an emerging and promising research topic. In this paper, we present a novel approach to synthesize video from text. The method builds a phoneme-pose dictionary and trains a generative adversarial network (GAN) to generate video from interpolated phoneme poses. Compared to audio-driven video generation algorithms, our approach has a number of advantages: 1) It only needs a fraction of the training data used by an audio-driven approach; 2) It is more ﬂexible and not subject to vulnerability due to speaker variation; 3) It signiﬁcantly reduces the preprocessing, training and inference time. We perform extensive experiments to compare the proposed method with state-of-the-art talking face generation methods on a benchmark dataset and datasets of our own. The results demonstrate the effectiveness and superiority of our approach.

## Data / Preprocessing
    Text2Video
    ├── *phoneme_data
    ├── ...
    vid2vid
    
## Set up
1. Download modified vid2vid repo

2. Prepare data and folder


## Testing
Example 1. test VidTIMIT data with real audio
    
    ```
    sh text_to_video_audio.sh "She had your dark suit in greasy wash water all year." fadg0
    ```
    
## Citation
Please cite our paper in your publications.

