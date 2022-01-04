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

def display_play_sample(file, full_auto):

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
 
    plot.subplot(211)
    plot.title(species)
    plot.plot(signalData)
    plot.xlabel('Sample')
    plot.ylabel('Amplitude')
    plot.subplot(212)
    plot.specgram(signalData,Fs=samplingFrequency)
    plot.xlabel('Time')
    plot.ylabel('Frequency')

    if (full_auto): 
        winsound.PlaySound(file, winsound.SND_ALIAS )
        again = 'y'
    else:
        plot.show(block = False)
        winsound.PlaySound(file, winsound.SND_ASYNC | winsound.SND_ALIAS )
        again = input("Another? (Y/n): ")
        
    print("\n")
    plot.clf()
    plot.cla()
    winsound.PlaySound(None, winsound.SND_ASYNC)
    return again
 
def main():
    try:
        dir = sys.argv[1]
        if not os.path.exists(dir):
            dir = os.getcwd()
            print("Requested input directory cannot be found")
    except:
        dir = os.getcwd()
        
    try:
        full_auto = sys.argv[2]
    except:
        full_auto = 0
            
    print("Using directory {}\n\n".format(dir))

    again = 'y'
    
    poss_files = [os.path.join(path, filename)
            for path, dirs, files in os.walk(dir)
            for filename in files]
            
    while (again in yes_list):
        
        # Retrieve file
        file = None
        attempts_left = 100
        while ((file == None) and (attempts_left > 0)):
        
            # Choose random file from directory
            file = random.choice(poss_files)
            if not file.endswith('.wav'):
                file = None
            
        if (attempts_left == 0):
            print("ERROR: no '.wav' sample was found")
            sys.exit()
            
        again = display_play_sample(file, full_auto)
        
if __name__ == '__main__':
    main()