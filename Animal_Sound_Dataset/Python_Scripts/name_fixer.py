#python script to edit filename on mass
# add *string* infront of all filenames of a directory

import os

counter = 0
path = "C:\\Users\\Josh\\Documents\\Wildlife_Monitor\\dataset\\voices"
myString = "voices_noise"
for file in os.listdir(path):
    if file.endswith(".WAV"):
        counter += 1
        os.rename(os.path.join(path, file), os.path.join(path, (myString + file)))

if counter == 0:
    print("No files have been found")
else:
    print(counter, " files have been renamed.")
    
    