import requests, sys
from pydub import AudioSegment
#input
import re
import string
from zhon.hanzi import punctuation

url = ' http://tts.baidu.com/text2audio'

# print('file_name', sys.argv[1])
# file_name= sys.argv[1]

print('input', sys.argv[1])
input = sys.argv[1]
stripped_space = re.sub(' ', '', input)
# print('stripped_space', stripped_space)
stripped_input = re.sub(r'[%s]+' %punctuation, '', stripped_space)
print('stripped_input', stripped_input)
file_name = stripped_input[:10]

print('person', sys.argv[2])
person = sys.argv[2]

sex = sys.argv[3]

# input = '正在为您查询合肥的天气情况。今天是2020年2月24日，合肥市今天多云，最低温度9摄氏度，最高温度15摄氏度，微风。'

#woman's voice Chinese
if person == 'henan':
    myobj = {'tex':input, 'lan':'zh', 'cuid':'XXX', 'ctp':'1', 'pdt':'9918', 'key':'com.baidu.tts.pre-online', 'per':'100'}
# myobj = {'body': 'tex=你好&lan=zh&cuid=XXX&ctp=1&pdt=9918&key=com.baidu.tts.pre-online&per=100'}

#men's voice Chinese
if person == 'xuesong':
    myobj = {'tex':input, 'lan':'zh', 'cuid':'XXX', 'ctp':'1', 'pdt':'9918', 'key':'com.baidu.tts.pre-online', 'per':'3'}

#woman/men's voice English
if sex == 'f':
    myobj = {'tex':input, 'lan':'zh', 'cuid':'XXX', 'ctp':'1', 'pdt':'9918', 'key':'com.baidu.tts.pre-online', 'per':'4100'}
else:
    myobj = {'tex':input, 'lan':'zh', 'cuid':'XXX', 'ctp':'1', 'pdt':'9918', 'key':'com.baidu.tts.pre-online', 'per':'4106'}


x = requests.post(url, data = myobj)

#print(x.headers)
# s = io.BytesIO(x.content)
# audiosegment.from_file(s).export('test.mp3', format='mp3')
# File.WriteAllBytes('test.mp3', x.content)
f = open('./input_audio/{person}/{file_name}.mp3'.format(person=person, file_name=file_name), 'w+b')

f.write(x.content)
f.close()
sound = AudioSegment.from_mp3('./input_audio/{person}/{file_name}.mp3'.format(person=person, file_name=file_name))
sound.export('./input_audio/{person}/{file_name}.wav'.format(person=person, file_name=file_name), format="wav")
