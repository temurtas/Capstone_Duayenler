from os import walk
from ImageProcess import ImageProcess as ip

f = []
base_directory = "../../resources/kkm-base-photos/"

for (dirpath, dirnames, filenames) in walk(base_directory):
    f.extend(filenames)
    break
#print filenames[1]

for photo in filenames:
	ip(base_directory, photo)
