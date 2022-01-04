#seprate but important, collect lots of data of noise/wind/general outdoor noises https://zenodo.org/record/4279220
from pydub import AudioSegment, silence
import itertools
import random
from pathlib import Path
import os
import sys
import datetime
from time import sleep

SAMPLE_OUT_DURATION = 4000
SILENCE_THRESHOLD = -30
WIND_DIRECTORY = r"C:\Users\Josh\Documents\Audio_Dataset\testing_resectioning\wind_noise_database"

# split the audio sample into separate 4.0 second clips (128kB), centred around loud sections
# shift the audio around a random amount to create three new samples per sample
def get_correct_length_loud_samples(audiofile):
    
    correct_length_samples = []
    
    for section in get_loud_sections(audiofile):
        
        a = section[0]
        b = section[1]
        duration = b - a
        print("Section [{}->{}] has duration {} ms".format(a, b, duration))
        
        if duration < SAMPLE_OUT_DURATION:          # pad either side until correct length, then append padded sample to list
            
            if (len(audiofile) >= SAMPLE_OUT_DURATION):
                mid = (a + b) / 2
                start = max(0, mid - (SAMPLE_OUT_DURATION / 2))
                end = start + SAMPLE_OUT_DURATION
                if (end > len(audiofile)):
                    end = len(audiofile)
                    start = end - SAMPLE_OUT_DURATION
                    start = max(0, start)# little check
                    end = min(len(audiofile), end)
                print("Expanding to [{}->{}]\n".format(start, end))
                correct_length_samples.append(audiofile[start:end])

            else: #audiofile is literally too short
                print("audiofile is too short! We can't expand the loud section, so we will pad it")
                silence_pad = AudioSegment.silent(duration=((SAMPLE_OUT_DURATION-len(audiofile[a:b])+1)/2))
                correct_length_sample = silence_pad + audiofile[a:b] + silence_pad # Adding silence either side
          
                # add some random wind throughout, so the silent parts aren't awkward
                print("And also add some wind overlay.")
                result_1 = correct_length_sample.overlay(random_wind(SAMPLE_OUT_DURATION))
                correct_length_samples.append(result_1)
                result_2 = correct_length_sample.overlay(random_wind(SAMPLE_OUT_DURATION))
                correct_length_samples.append(result_2)
                
        elif duration == SAMPLE_OUT_DURATION:    #append to list
            print("section is perfect length, no padding!\n")
            correct_length_sample = audiofile[a:b]
            correct_length_samples.append(correct_length_sample)
        else:                                       #split into smaller subsections, and append each to list
            print("Splitting into.. ", end = ' ')
            a = section[0]
            b = section[1]
            while (a <= (b - SAMPLE_OUT_DURATION)):
                correct_length_sample = audiofile[a:a+SAMPLE_OUT_DURATION]
                print("[{}->{}], ".format(a, a+SAMPLE_OUT_DURATION),end= '')
                correct_length_samples.append(correct_length_sample)
                a += 1000
            print("\n")    
    return correct_length_samples
    

# returns list of loud sections as 2d array ie [[1,3],[6,11]] for loud sections at 1s->3s and at 6s->11s
def get_loud_sections(audiofile):
    loud_sections = detect_nonsilent(audiofile, min_silence_len=500, silence_thresh=SILENCE_THRESHOLD)
    print("Loud sections found at...")
    for section in loud_sections:
        a = section[0]
        b = section[1]
        section = [a-750, b+750]
    for section in loud_sections:
        print(section, end=' ')
    print("\n")
    return loud_sections


def detect_silence(audio_segment, min_silence_len=1000, silence_thresh=-16, seek_step=1):
    """
    Returns a list of all silent sections [start, end] in milliseconds of audio_segment.
    Inverse of detect_nonsilent()
    audio_segment - the segment to find silence in
    min_silence_len - the minimum length for any silent section
    silence_thresh - the upper bound for how quiet is silent in dFBS
    seek_step - step size for interating over the segment in ms
    """
    seg_len = len(audio_segment)

    # you can't have a silent portion of a sound that is longer than the sound
    if seg_len < min_silence_len:
        return []

    # convert silence threshold to a float value (so we can compare it to rms)
    silence_thresh = db_to_float(silence_thresh) * audio_segment.max_possible_amplitude
    
    # find silence and add start and end indicies to the to_cut list
    silence_starts = []

    # check successive (1 sec by default) chunk of sound for silence
    # try a chunk at every "seek step" (or every chunk for a seek step == 1)
    last_slice_start = seg_len - min_silence_len
    slice_starts = range(0, last_slice_start + 1, seek_step)

    # guarantee last_slice_start is included in the range
    # to make sure the last portion of the audio is searched
    if last_slice_start % seek_step:
        slice_starts = itertools.chain(slice_starts, [last_slice_start])

    for i in slice_starts:
        audio_slice = audio_segment[i:i + min_silence_len]
        if audio_slice.rms <= silence_thresh:
            silence_starts.append(i)

    # short circuit when there is no silence
    if not silence_starts:
        return []

    # combine the silence we detected into ranges (start ms - end ms)
    silent_ranges = []

    prev_i = silence_starts.pop(0)
    current_range_start = prev_i

    for silence_start_i in silence_starts:
        continuous = (silence_start_i == prev_i + seek_step)

        # sometimes two small blips are enough for one particular slice to be
        # non-silent, despite the silence all running together. Just combine
        # the two overlapping silent ranges.
        silence_has_gap = silence_start_i > (prev_i + min_silence_len)

        if not continuous and silence_has_gap:
            silent_ranges.append([current_range_start,
                                  prev_i + min_silence_len])
            current_range_start = silence_start_i
        prev_i = silence_start_i

    silent_ranges.append([current_range_start,
                          prev_i + min_silence_len])

    return silent_ranges


def detect_nonsilent(audio_segment, min_silence_len=1000, silence_thresh=-16, seek_step=1):
    """
    Returns a list of all nonsilent sections [start, end] in milliseconds of audio_segment.
    Inverse of detect_silent()
    audio_segment - the segment to find silence in
    min_silence_len - the minimum length for any silent section
    silence_thresh - the upper bound for how quiet is silent in dFBS
    seek_step - step size for interating over the segment in ms
    """
    silent_ranges = detect_silence(audio_segment, min_silence_len, silence_thresh, seek_step)
    len_seg = len(audio_segment)

    # if there is no silence, the whole thing is nonsilent
    if not silent_ranges:
        return [[0, len_seg]]

    # short circuit when the whole audio segment is silent
    if silent_ranges[0][0] == 0 and silent_ranges[0][1] == len_seg:
        return []

    prev_end_i = 0
    nonsilent_ranges = []
    for start_i, end_i in silent_ranges:
        nonsilent_ranges.append([prev_end_i, start_i])
        prev_end_i = end_i

    if end_i != len_seg:
        nonsilent_ranges.append([prev_end_i, len_seg])

    if nonsilent_ranges[0] == [0, 0]:
        nonsilent_ranges.pop(0)

    return nonsilent_ranges
    
def random_wind(duration):
    sample_duration = 0
    attempts_left = 10
    while ((sample_duration < duration) and (attempts_left > 0)):
    
        #choose random file from directory
        windfiles = [os.path.join(path, filename)
        for path, dirs, files in os.walk(WIND_DIRECTORY)
        for filename in files
        if not filename.endswith(".bak")]
        
        sample_name = random.choice(windfiles)
        if sample_name.endswith('.mp3'):
            sample = AudioSegment.from_mp3(sample_name)
        elif sample_name.endswith('.wav'):
            sample = AudioSegment.from_wav(sample_name)
        else: sample = []
        sample_duration = len(sample)
        
    if (attempts_left == 0):
        print("no wind sample of good duration found.")
        return AudioSegment.silence(duration) #give up, use silence
    
    #cut to right length (random segment of length duration)
    half_dur = duration / 2
    rand_mid_point = random.randrange(half_dur, (sample_duration-half_dur ))
    start_point = rand_mid_point - half_dur
    end_point = rand_mid_point + half_dur
    sample = sample[start_point:end_point]
    print("with wind sample {} between {} and {}".format(sample_name, start_point, end_point))
    # tone down the volume
    attenuation = 1.5 #measured in dB
    return sample - attenuation

def db_to_float(db, using_amplitude=True):
    """
    Converts the input db to a float, which represents the equivalent
    ratio in power.
    """
    db = float(db)
    if using_amplitude:
        return 10.0 ** (db / 20.0)
    else:  # using power
        return 10.0 ** (db / 10.0)

def process_audiofile(audio_filename, output_dir):
    print("Processing sample {}".format(audio_filename))
    audiofile = AudioSegment.from_wav(audio_filename)
    sections = get_correct_length_loud_samples(audiofile)   #these are all right length, without long silent periods, and padded with wind if needed
    counter = 1
    for section in sections:
        #write the new files somewhere else (as USB will defo run out of space!)
        filename = Path(audio_filename).stem + "_" + str(counter) + ".wav"
        drive, path = os.path.splitdrive(audio_filename)
        path_wout_filename = os.path.dirname(path)
        dir_name = os.path.join(output_dir, path_wout_filename)
        if not os.path.exists(dir_name):
            os.makedirs(dir_name)
        output_path_name = os.path.join(dir_name, filename)
        print("Writing new sample {}".format(output_path_name))
        section.export(output_path_name, format="wav")
        counter += 1

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
    
def get_already_done_dirs(rootdir, targetdir):
    
    done_dirs = []
    split_targetdir = splitall(targetdir)
    
    for subdir, dirs, files in os.walk(targetdir):
        for file in files:
            split_subdir = splitall(subdir)
            
            i = 0
            while ((split_targetdir[i] == split_subdir[i]) and (i < (len(split_targetdir) - 1))):
                i += 1
            split_subdir = os.path.join(*split_subdir[i+1:])
            dir_already_done = os.path.join(rootdir,split_subdir)
            if dir_already_done not in done_dirs:
                done_dirs.append(dir_already_done)
    
    return done_dirs

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
        outputdir = sys.argv[2]
        if not os.path.exists(outputdir):
            outputdir = os.getcwd()
            print("Requested output directory cannot be found")
    except:
        outputdir = os.getcwd()
    
    print("\nINPUT DIRECTORY: {}".format(rootdir))
    print("OUTPUT DIRECTORY: {}".format(outputdir))
    total_files = sum([len(files) for r, d, files in os.walk(rootdir)])
    print("Starting to process {} files...\n".format(total_files))
    files_processed = 0
    total_proc_time = 0
    
    paths_to_ignore = get_already_done_dirs(rootdir, outputdir)
    print("\nLooks like these have dirs have previously been done {}\n".format(paths_to_ignore))
    
    for subdir, dirs, files in os.walk(rootdir):
        for file in files:
            audio_filename = os.path.join(subdir, file)
            if subdir not in paths_to_ignore:
                if audio_filename.endswith('.wav'):
                    process_audiofile(audio_filename, outputdir)
            else:
                print("Ignoring {} {}".format(subdir, file))
            files_processed += 1
            files_remaining = total_files - files_processed
            print("\nFiles processed: {}, and {} files remain.\n".format(files_processed, files_remaining))
            
    elapsed_time = datetime.datetime.now() - abs_start_time
    print("\nDONE! Total time elapsed: {}".format(str(datetime.timedelta(seconds=elapsed_time.total_seconds()))))
    
main()