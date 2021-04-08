export LANG="en_US.UTF-8"
# source venv_vid2vid/bin/activate

echo "$1"
echo $2
echo $3

rm -f input/$2/"$1".txt
echo "$1" >> input/$2/"$1".txt

python tts_request.py "$1" $2 $3

## Chinese audio
# python tts_request.py 正在为您查询合肥的天气情况

# eng
cd aligner
python align_english.py "$1" $2 
# python align_english_states.py "$1" $2 

cd ..

# chn
# python pinyin_timestamping.py $1 $2

# generate video
rm -f ../vid2vid/datasets/$2/test_openpose/tmp/*
rm -f ../vid2vid/datasets/$2/test_img/tmp/*

rm -f ../vid2vid/datasets/$2/test_openpose/tmp_smooth/*
rm -f ../vid2vid/datasets/$2/test_img/tmp_smooth/*

## dataset and smooth method 
python interp_landmarks_motion_phoneme_VidTIMIT_smooth.py "$1" $2
# python interp_landmarks_motion_phoneme_VidTIMIT.py "$1" $2
# python interp_landmarks_motion_phoneme_dict.py "$1" $2
# python interp_landmarks_motion.py $1 $2
# python interp_landmarks_motion_phoneme.py "$1" $2

cd ../vid2vid

rm -f results/$2/test_latest/tmp/*
rm -f results/$2/test_latest/tmp_smooth/*

CUDA_VISIBLE_DEVICES=1 python test.py --name $2 --dataroot datasets/$2 --dataset_mode pose --input_nc 3 --resize_or_crop scaleHeight --loadSize 512 --openpose_only --how_many 1200 --no_first_img --random_drop_prob 0


python image2video.py "$1" $2
