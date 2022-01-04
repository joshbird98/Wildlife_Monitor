#iterate trough every file
#if file contains human speech, add to list
#iterate through list, play out loud then delete y/n

import datetime
import os
import sys
import speech_recognition as sr
import winsound
yes_list = ['y', 'Y', 'yes', 'Yes', '']


def detect_speech(audio_filename, files_with_speech):
    r = sr.Recognizer()
    print("Scanning {} for speech".format(audio_filename))
    sound = sr.AudioFile(audio_filename)
    with sound as source:
        r.adjust_for_ambient_noise(source, duration = 0.2)
        audio = r.record(source)
    try:
        r.recognize_google(audio)
    except:
        print("NO SPEECH DETECTED")
        return files_with_speech
    
    print("SPEECH DETECTED")
    files_with_speech.append(audio_filename)
    return files_with_speech

def main():
    
    abs_start_time = datetime.datetime.now()
    try:
        rootdir = sys.argv[1]
        if not os.path.exists(rootdir):
            rootdir = os.getcwd()
            print("Requested input directory cannot be found")
    except:
        rootdir = os.getcwd()
        
    print("\nINPUT DIRECTORY: {}".format(rootdir))

    total_files = sum([len(files) for r, d, files in os.walk(rootdir)])
    print("Starting to scan {} files...\n".format(total_files))
    files_processed = 0
    total_proc_time = 0
    files_with_speech = []
    for subdir, dirs, files in os.walk(rootdir):
        for file in files:
            audio_filename = os.path.join(subdir, file)
            if audio_filename.endswith('.wav'):
                    files_with_speech = detect_speech(audio_filename, files_with_speech)
            else:
                print("Ignoring {} {}".format(subdir, file))
            files_processed += 1
            files_remaining = total_files - files_processed
            print("\nFiles scanned: {}, and {} files remain.\n".format(files_processed, files_remaining))
            
    elapsed_time = datetime.datetime.now() - abs_start_time
    print("\nAll files scanned for speech! Total time elapsed: {}".format(str(datetime.timedelta(seconds=elapsed_time.total_seconds()))))
    
    i = len(files_with_speech)
    print("{} files were thought to contain speech.\n".format(i))
    
    f = open("files_with_possible_speech.txt", 'w')
    for item in files_with_speech:
        f.write(item)
        f.write('\n')
    f.close()
    
    input("Press Enter when you are ready to review files for deletion.")
    
    for file in files_with_speech:
        print("FILE: {}".format(file))
        winsound.PlaySound(file, winsound.SND_ASYNC | winsound.SND_ALIAS )
        delete = input("?Delete (Y/n): ")
        winsound.PlaySound(None, winsound.SND_ASYNC)
        if delete in yes_list:
            os.remove(file)
            print("File deleted.")
        i-= 1
        print("\n{} files left to look at.\n\n".format(i))
    
if __name__ == '__main__':
    main()