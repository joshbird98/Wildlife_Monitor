#iterate trough every file
#rewrite sample as 16kHz

import librosa
import soundfile as sf
import datetime
import os
import sys


###########also make mono
def resample(audio_filename, sample_rate):
    print("Resampling: {}".format(audio_filename))
    y, s = librosa.load(audio_filename, sr=sample_rate) # Downsample 44.1kHz to 8kHz
    os.remove(audio_filename)
    y = librosa.to_mono(y)
    sf.write(audio_filename, y, s)

def main():
    
    abs_start_time = datetime.datetime.now()
    try:
        rootdir = sys.argv[1]
        if not os.path.exists(rootdir):
            rootdir = os.getcwd()
            print("Requested input directory cannot be found")
    except:
        rootdir = os.getcwd()
    
    try:
        sample_rate = int(sys.argv[2])
    except:
        sample_rate = 16000
        
    print("\nINPUT DIRECTORY: {}".format(rootdir))
    print("NEW SAMPLE RATE: {}".format(sample_rate))
    
    total_files = sum([len(files) for r, d, files in os.walk(rootdir)])
    print("Starting to resample {} files...\n".format(total_files))
    files_processed = 0
    total_proc_time = 0

    for subdir, dirs, files in os.walk(rootdir):
        for file in files:
            audio_filename = os.path.join(subdir, file)
            if audio_filename.endswith('.wav'):
                    resample(audio_filename, sample_rate)
            else:
                print("Ignoring {} {}".format(subdir, file))
            files_processed += 1
            files_remaining = total_files - files_processed
            print("\nFiles resampled: {}, and {} files remain.\n".format(files_processed, files_remaining))
            
    elapsed_time = datetime.datetime.now() - abs_start_time
    print("\nDONE! Total time elapsed: {}".format(str(datetime.timedelta(seconds=elapsed_time.total_seconds()))))
    
if __name__ == '__main__':
    main()