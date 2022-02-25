#!/usr/bin/python3

from vosk import Model, KaldiRecognizer
import sys
import os
import wave
import json
import cn2an
from pypinyin import pinyin, lazy_pinyin, Style

#input
import re
import string
from zhon.hanzi import punctuation

# define punctuation
punctuations = '''!()-[]{};:'"\,<>./?@#$%^&*_~'''

input = sys.argv[1]
stripped_input = re.sub(r'[%s]+' %punctuation, '', input)
file_name = stripped_input[:10]
person = sys.argv[2]

fps = 30.0

# wf = wave.open(sys.argv[1], "rb")

#input
print('stripped_chn_punct', stripped_input)

stripped_eng_punct = ""
for char in stripped_input:
   if char not in punctuations:
       stripped_eng_punct = stripped_eng_punct + char 
print('stripped_eng_punct', stripped_eng_punct)

# stripped = re.sub(r'[%s]+' %punctuation, '', stripped_eng_punct)
# print('stripped_chn_punct', stripped)
len_stripped=len(stripped_eng_punct)
print('len_stripped', len_stripped)

py_input=lazy_pinyin(stripped_eng_punct)
# print('input_pinyin', py_input)
input_length=len(py_input)

text_file = open('./input_timestamp/{person}/{file_name}.txt'.format(person=person, file_name=file_name), "w")

wf = wave.open('./input_audio/{person}/{file_name}.wav'.format(person=person, file_name=file_name), "rb")

def digi_to_py(word):
    digidict = {
        '1': u'yi', '2': u'er', '3': u'san', '4': u'si', '5': u'wu', 
        '6': u'liu', '7': u'qi', '8': u'ba', '9': u'jiu', '0': u'ling'
        }
    patten = '([0-9]{1})'
    for i in word:
        if re.search(patten, i):
            py = digidict[i]
            word = word.replace(i, (py))
    return word

if wf.getnchannels() != 1 or wf.getsampwidth() != 2 or wf.getcomptype() != "NONE":
    print ("Audio file must be WAV format mono PCM.")
    exit (1)

#https://github.com/alphacep/vosk-api/blob/37fbe1a52b274e980fba992af73991529e524568/python/example/test_text.py

if not os.path.exists("model"):
    print ("Please download the model from https://github.com/alphacep/vosk-api/blob/master/doc/models.md and unpack as 'model' in the current folder.")
    exit (1)

model = Model("model")
rec = KaldiRecognizer(model, wf.getframerate())
rec.SetWords(True)

while True:
    data = wf.readframes(1000000)
    if len(data) == 0:
        break
    if rec.AcceptWaveform(data):
        res =json.loads(rec.Result())
        # print('res', res)
        print('AcceptWaveform rec.Result.text', res['text'])
    else:
        print('rec.PartialResult', rec.PartialResult())
        res = json.loads(rec.FinalResult())
        print('rec.FinalResult', res['text'])

i=0
j=0
for item in res['result']:
    print('item', item)
    #print(item['word'])
    py = lazy_pinyin(item['word'])

    st = item['start']
    et = item['end']

    nc = len(py)
    # print('nc', nc)

    step = (et-st)/(nc+1)

    for idx in range(nc):
        tstamp=st + step * (idx + 1)
        # print(tstamp, py[idx])
        frame_idx = int(tstamp * fps + 0.5)

        #no number, only chinese input        
        # n = text_file.write(str(frame_idx) + ' ' + py_input[i] + '\n')
        # print(str(frame_idx), py_input[i], 'py_input[i]')
        # i=i+1
        if i > len_stripped-1:
            break

        if py_input[i].isnumeric():
            # output = cn2an.an2cn(py_input[i], "low")
            # print('to_chn', output)
            len_digit= len(py_input[i])
            # print('isdigit', py_input[i], 'len', len_digit, 'j', j)

            if j<len_digit:
                # print('digit', py_input[i][0])
                # print(list(py_input[i]))
                # digit_cn= cn2an.an2cn(py_input[i][j], "low")
                # print(str(frame_idx), py_input[i][j], 'py_input[i][j]', i)
                py_digit_cn = digi_to_py(py_input[i][j])
                n = text_file.write(str(frame_idx) + ' ' + py_digit_cn + '\n')
                print(str(frame_idx), py_digit_cn, 'py_digit_cn', i)
                digit = int(py_input[i])
                j=j+1
            else:
                j=0
                i=i+1

        else:
            n = text_file.write(str(frame_idx) + ' ' + py_input[i] + '\n')
            print(str(frame_idx), py_input[i], 'input', i)
            i=i+1
        
    
text_file.close()


