~~~~~~~~~~~~~~~~~~~~~~~Dataset Description~~~~~~~~~~~~~~~~~~~~~~~

The animal audio sample dataset contains 116,131 audio samples,
and totals 14.1GB.
Because it is so large, we cannot host it on GitHub.
It is however available on a USB stick, just ask.
Yes, ofc there is a relevant xkcd https://what-if.xkcd.com/31/

Every audio sample is...
	- .wav format
	- duration exactly 4 seconds
	- sample rate of 16kHz
	- bit depth of 16 bits, or 2 bytes
	- exactly 128kB of data
	- set to "Read-only" mode

The audio samples are placed in directories relating to their
taxonomical naming system.

For example...
Samples in \animalia\chordata\mammalia\elephantidae\elephas\maximus
Kingdom: Animlia
Phylum: Chordata
Class: Mammalia
Family: Elephantidae
Genus: Elephas
Sub-genus: Maximus
Common Name: Asian Elephant


~~~~~~~~~~~~~~~~~~~~~~~~~~~Dataset Usage~~~~~~~~~~~~~~~~~~~~~~~~~~~

The data samples are ready-to-use with Edge Impulse.

The edge-impulse-cli is very good, and allows easy uploading of
many samples. I recommend installing it and trying it.


~~~~~~~~~~~~~~~~~~~~Dataset Generation Process~~~~~~~~~~~~~~~~~~~~~

The audio samples have been scraped from the Animal Sound Archive
https://www.gbif.org/dataset/b7ec1bf8-819b-11e2-bad2-00145eb45e9a

The original files downloaded from GBIF are in Misc.

Downloading them required cross-referencing a spreadsheet of audio
sample information with a separate spreadsheet containing links to
the relevant online mp3's. For this task I created a Python script.
It scraped out all the 36000+ mp3's and placed them into the 
relevant directory. It took 14 hours to complete.

These mp3 samples were varied in length, some with minutes of silence
and only brief sounds from the actual animal, others only 1 second 
long. Many samples also contained human speech as narration.
The sampling rate and the bit-depth were also highly inconsistent. 

I created multiple Python scripts that...
	- Resampled all .mp3 files to 16kHz 16-bit .wav
	- Detected quiet sections in each sample, and cropped the
	  sample into multiple <=4 second clips around the loud parts.
	- Extended clips that had a duration of <4 seconds by
	  recentering the audio into a 4 second duration and adding 
	  random wind and background noise to remove abrupt changes.
	- Scanned for human speech using Google's Speech Recognition
	  API and flagged the samples for manual removal.
These scripts took a total runtime of 40 hours.

In the directory Python_Scripts are a few extra scripts for making it
easier to review the dataset quality.
