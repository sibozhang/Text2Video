#!/usr/bin/env python

""" Usage:
      align_english.py wavfile trsfile outwordfile outphonefile
"""

import os
import sys
import wave

#input
import re
import string
from zhon.hanzi import punctuation

PHONEME = '../tools/english2phoneme/phoneme'
MODEL_DIR = './english'
HVITE = '../tools/htk/HTKTools/HVite'
HCOPY = '../tools/htk/HTKTools/HCopy'

### sibo
# define punctuation
punctuations = '''!()-[]{};:'"\,<>./?@#$%^&*_~'''

input = sys.argv[1]
# print('input', input)
stripped_space = re.sub(' ', '', input)
stripped_input = re.sub(r'[%s]+' %punctuation, '', stripped_space)

file_name = stripped_input[:10]
print('file_name', file_name)
person = sys.argv[2]

fps = 25

def prep_txt(trsfile, tmpbase, dictfile):
 
    words = []
    with open(trsfile, 'r') as fid:
        for line in fid:
            line = line.strip()
            for pun in [',', '.', ':', ';', '!', '?', '"', '(', ')', '--', '---']:
                line = line.replace(pun, ' ')
            for wrd in line.split():
                if (wrd[-1] == '-'):
                    wrd = wrd[:-1]
                if (wrd[0] == "'"):
                    wrd = wrd[1:]
                if wrd:
                    words.append(wrd)

    ds = set([])
    with open(dictfile, 'r') as fid:
        for line in fid:
            ds.add(line.split()[0])

    unk_words = set([])
    with open(tmpbase + '.txt', 'w') as fwid:
        for wrd in words:
            if (wrd.upper() not in ds):
                unk_words.add(wrd.upper())
            fwid.write(wrd + ' ')
        fwid.write('\n')

    #generate pronounciations for unknows words using 'letter to sound'
    with open(tmpbase + '_unk.words', 'w') as fwid:
        for unk in unk_words:
            fwid.write(unk + '\n')
    try:
        os.system(PHONEME + ' ' + tmpbase + '_unk.words' + ' ' + tmpbase + '_unk.phons')
    except:
        print('english2phoneme error!')
        sys.exit(1)

    #add unknown words to the standard dictionary, generate a tmp dictionary for alignment 
    fw = open(tmpbase + '.dict', 'w')
    with open(dictfile, 'r') as fid:
        for line in fid:
            fw.write(line)
    f = open(tmpbase + '_unk.words', 'r')
    lines1 = f.readlines()
    f.close()
    f = open(tmpbase + '_unk.phons', 'r')
    lines2 = f.readlines()
    f.close()
    for i in range(len(lines1)):
        wrd = lines1[i].replace('\n', '')
        phons = lines2[i].replace('\n', '').replace(' ', '')
        seq = []
        j = 0
        while (j < len(phons)):
            if (phons[j] > 'Z'):
                if (phons[j] == 'j'):
                    seq.append('JH')
                elif (phons[j] == 'h'):
                    seq.append('HH')
                else:
                    seq.append(phons[j].upper())
                j += 1
            else:
                p = phons[j:j+2]
                if (p == 'WH'):
                    seq.append('W')
                elif (p in ['TH', 'SH', 'HH', 'DH', 'CH', 'ZH', 'NG']):
                    seq.append(p)
                elif (p == 'AX'):
                    seq.append('AH0')
                else:
                    seq.append(p + '1')
                j += 2

        fw.write(wrd + ' ')
        for s in seq:
            fw.write(' ' + s)
        fw.write('\n')
    fw.close()

def prep_mlf(txt, tmpbase):

    with open(tmpbase + '.mlf', 'w') as fwid:
        fwid.write('#!MLF!#\n')
        fwid.write('"' + tmpbase + '.lab"\n')
        fwid.write('sp\n')
        wrds = txt.split()
        for wrd in wrds:
            fwid.write(wrd.upper() + '\n')
            fwid.write('sp\n')
        fwid.write('.\n')

def gen_res(tmpbase, outfile1, outfile2):
    with open(tmpbase + '.txt', 'r') as fid:
        words = fid.readline().strip().split()
    words = txt.strip().split()
    words.reverse()

    with open(tmpbase + '.aligned', 'r') as fid:
        lines = fid.readlines()
    i = 2
    times1 = []
    times2 = []
    phones_frame = []
    while (i < len(lines)):
        if (len(lines[i].split()) >= 4) and (lines[i].split()[0] != lines[i].split()[1]):
            phn = lines[i].split()[2]
            pst = (int(lines[i].split()[0])/1000+125)/10000
            pen = (int(lines[i].split()[1])/1000+125)/10000
            times2.append([phn, pst, pen])
            frame = int(0.5*(pen+pst)*fps)
            # print(frame, phn)
            phones_frame.append([frame, phn])

        if (len(lines[i].split()) == 5):
            if (lines[i].split()[0] != lines[i].split()[1]):
                wrd = lines[i].split()[-1].strip()
                st = (int(lines[i].split()[0])/1000+125)/10000
                j = i + 1
                while (lines[j] != '.\n') and (len(lines[j].split()) != 5):
                    j += 1
                en = (int(lines[j-1].split()[1])/1000+125)/10000
                times1.append([wrd, st, en])
        i += 1

    with open(outfile1, 'w') as fwid:
        for item in times1:
            if (item[0] == 'sp'):
                fwid.write(str(item[1]) + ' ' + str(item[2]) + ' SIL\n')
            else:
                wrd = words.pop()
                fwid.write(str(item[1]) + ' ' + str(item[2]) + ' ' + wrd + '\n')
    if words:
        print('not matched::' + alignfile)
        sys.exit(1)

    # with open(outfile2, 'w') as fwid:
    #     for item in times2:
    #         fwid.write(str(item[1]) + ' ' + str(item[2]) + ' ' + item[0] + '\n')

    with open(outfile2, 'w') as fwid:
        # for item in times2:
        #     fwid.write(str(item[1]) + ' ' + str(item[2]) + ' ' + item[0] + '\n')        
        for item in phones_frame:
            print(item[0], item[1])
            fwid.write(str(item[0]) + ' ' + item[1] + '\n')

if __name__ == '__main__':

    #python align_english.py ../input_audio/sibo/The-way-to.wav ../input/sibo/The-way-to.txt ../input_timestamp/sibo/words/The-way-to.words ../input_timestamp/sibo/phones/The-way-to.phones

    # wf = './input_audio/{person}/{file_name}.wav'.format(person=person, file_name=file_name)
    # text_file = './input/{person}/{input}.txt'.format(person=person, input=input)

    try:
        # wavfile = '../input_audio/sibo/The-way-to.wav'
        wavfile = '../input_audio_real/{person}/{file_name}.wav'.format(person=person, file_name=file_name)

        # trsfile = '../input/sibo/The-way-to.txt'
        trsfile = '../input/{person}/{input}.txt'.format(person=person, input=input)

        # outfile1 = '../input_timestamp/sibo/words/The way to.txt'
        # outfile2 = '../input_timestamp/sibo/phones/The way to.txt'
        outfile1 = '../input_timestamp/{person}/words/{file_name}.txt'.format(person=person, file_name=file_name)
        outfile2 = '../input_timestamp/{person}/phones/{file_name}.txt'.format(person=person, file_name=file_name)

        # wavfile = sys.argv[1]
        # trsfile = sys.argv[2]
        # outfile1 = sys.argv[3]
        # outfile2 = sys.argv[4]

    except IndexError:
        print(__doc__)
   
    tmpbase = '/tmp/' + os.environ['USER'] + '_' + str(os.getpid())

    #prepare wav and trs files
    try:
        print('Preparing wav file...', wavfile)
        os.system('sox ' + wavfile + ' -r 16000 -b 16 ' + tmpbase + '.wav remix -')
    except:
        print('sox error!')
        sys.exit(1)
    
    #prepare clean_transcript file
    try:
        print('Processing unknown words...')
        prep_txt(trsfile, tmpbase, MODEL_DIR + '/dict')
    except:
        print('prep_txt error!')
        sys.exit(1)

    #prepare mlf file
    try:
        print('Preparing mlf file...')
        with open(tmpbase + '.txt', 'r') as fid:
            txt = fid.readline()
        prep_mlf(txt, tmpbase)
    except:
        print('prep_mlf error!')
        sys.exit(1)

    #prepare scp
    try:
        print('Extracting features...')
        os.system(HCOPY + ' -C ' + MODEL_DIR + '/16000/config ' + tmpbase + '.wav' + ' ' + tmpbase + '.plp')
    except:
        print('HCopy error!')
        sys.exit(1)

    #run alignment
    try:
        print('Running alignment...')
        os.system(HVITE + ' -a -m -t 10000.0 10000.0 100000.0 -I ' + tmpbase + '.mlf -H ' + MODEL_DIR + '/16000/macros -H ' + MODEL_DIR + '/16000/hmmdefs -i ' + tmpbase +  '.aligned '  + tmpbase + '.dict ' + MODEL_DIR + '/monophones ' + tmpbase + '.plp 2>&1 > /dev/null') 
    except:
        print('HVite error!')
        sys.exit(1)

    #generate results
    try:
        print('Generating results...')
        gen_res(tmpbase, outfile1, outfile2)
    except:
        print('gen_res error!')
        sys.exit(1)

    #clean up
    print('All done!')
    os.system('rm -f ' + tmpbase + '*')
