import sys
import matplotlib.pyplot as plot
from scipy.io import wavfile 
from pydub import AudioSegment
from pydub.utils import mediainfo
import os
import random
import winsound

#Plays random samples and displays info on screen
yes_list = ['y', 'Y', 'yes', 'Yes', '']

def splitall(path):
    allparts = []
    while 1:
        parts = os.path.split(path)
        if parts[0] == path:  # sentinel for absolute paths
            allparts.insert(0, parts[0])
            break
        elif parts[1] == path: # sentinel for relative paths
            allparts.insert(0, parts[1])
            break
        else:
            path = parts[0]
            allparts.insert(0, parts[1])
    return allparts

def display_play_sample(file):

    # Print info about the file, show as spectrogram too
    audio_sample = AudioSegment.from_wav(file)
    samplingFrequency, signalData = wavfile.read(file)
    filename = splitall(file)
    if 'animalia' in filename:
        filename = filename[filename.index('animalia'):]
    species = '-'.join(filename[:-1])
    print("File: {}".format(file))
    print("Species: {}".format(species))
    print("Duration: {}".format(len(audio_sample)))
    print("Sample rate: {}".format(mediainfo(file)['sample_rate']))
    print("Bit-depth: {}\n".format(8*audio_sample.sample_width))

    #Play audio non-blocking
    winsound.PlaySound(file, winsound.SND_ASYNC | winsound.SND_ALIAS )

    
    plot.subplot(211)
    plot.title(species)
    plot.plot(signalData)
    plot.xlabel('Sample')
    plot.ylabel('Amplitude')
    plot.subplot(212)
    plot.specgram(signalData,Fs=samplingFrequency)
    plot.xlabel('Time')
    plot.ylabel('Frequency')
    plot.show(block = False)

    input("Enter key to exit.")
    plot.clf()
    plot.cla()
    winsound.PlaySound(None, winsound.SND_ASYNC)
 
def main():
    try:
        display_play_sample(sys.argv[1])
    except:
        print("Error, file couldn't be found.")
    sys.exit()
        
if __name__ == '__main__':
    main()