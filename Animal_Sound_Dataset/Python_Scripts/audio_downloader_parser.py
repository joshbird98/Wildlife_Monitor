# iterate through files, download audio, and also add column for sample duration - then add audio to relevant directory (create dir if needed)

import pandas as pd
import os
import requests
from pydub import AudioSegment
from mutagen.mp3 import MP3
import numpy as np
import datetime

df = pd.read_csv('dataset_2.csv')
df['duration'] = np.nan
num_rows = len(df.index)

average_proc_time_per_sample = 0
total_proc_time = 0
success_count = 0
fail_count = 0

for x in df.index:
	start_time = datetime.datetime.now()
    
	kingdom = str(df.loc[x, 'kingdom']).lower()
	phylum = str(df.loc[x, 'phylum']).lower()
	class_ = str(df.loc[x, 'class']).lower()
	order = str(df.loc[x, 'family']).lower()
	genus = str(df.loc[x, 'genus']).lower()
	subgenus = str(df.loc[x, 'subgenus']).lower()

	print("Working on {} {} {} {} {} {}".format(kingdom,phylum,class_,order,genus,subgenus))
	
	folder_path = os.path.join("D:",kingdom,phylum,class_,order,genus,subgenus)
	if not os.path.exists(folder_path):
		print("Creating new folder for this subgenus.")
		os.makedirs(folder_path)
	
	num_files_in_dir = len([name for name in os.listdir(folder_path) if os.path.isfile(os.path.join(folder_path, name))])

	link = str(df.loc[x, 'link'])
	print(link)
	audio = requests.get(link)
	if audio.status_code != 404:
		    
		audio_filename_mp3 = "sample_{}.mp3".format(num_files_in_dir + 1)
		with open(os.path.join(folder_path, audio_filename_mp3), 'wb') as f:
			f.write(audio.content)

		audio_filename_wav = "sample_{}.wav".format(num_files_in_dir + 1)
		sound = AudioSegment.from_mp3(os.path.join(folder_path, audio_filename_mp3))
		sound.set_frame_rate(16000)
		sound.set_sample_width(2)
		sound.export(os.path.join(folder_path, audio_filename_wav), format="wav")

		audio_duration = MP3(os.path.join(folder_path, audio_filename_mp3)).info.length
		df.loc[x, 'duration'] = audio_duration
        
		os.remove(os.path.join(folder_path, audio_filename_mp3))
        
		print("Success")
		success_count += 1
    
	else:
		print("Link unavailable.")
		fail_count += 1
    
	end_time = datetime.datetime.now()
	elapsed_time = end_time - start_time 
	elapsed_time_ms = elapsed_time.total_seconds() * 1000
	total_proc_time += elapsed_time_ms
	average_proc_time_per_sample = total_proc_time / (success_count+fail_count)
	rem_time = (num_rows - (success_count+fail_count)) * average_proc_time_per_sample
	print("Estimated remaining time: {} seconds".format(int(rem_time / 1000)))

df = df.dropna()
df.to_csv('dataset_out.csv', index=False)

print("Complete. {} audio samples sucessfully stored, {} failed.".format(success_count, fail_count))
print("Total processing time: {} seconds".format(int(total_proc_time / 1000)))