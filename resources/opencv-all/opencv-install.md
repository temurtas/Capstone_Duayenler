## Installation

- Downloading and Installing: First instal all dependicies just in Section 01 of [this tutorial](https://linuxize.com/post/how-to-install-opencv-on-ubuntu-18e-04/). Then just follow [this tutorial](https://mindchasers.com/dev/ubuntu-opencv) word by word. This completely configures everything.
- Compiling the code: The general structure for compiling from the terminal
	- `gcc -g <code_file_name> -I <base_lib_directory> -l <include_lib1> ... -l <include_libN> -o <output_file_name>`
	- e.g. `gcc -g edge.cpp -I/usr/local/include/opencv4 -lstdc++ -lopencv_highguiThis -lopencv_imgcodecs -lopencv_core -lopencv_imgproc -o edge`

## Guides
- There are books in the folder.
- There are papers in the folder.
- [This repo](https://github.com/spmallick/learnopencv) with its website looks good.
- [This](https://www.learnopencv.com/color-spaces-in-opencv-cpp-python/) is good source for color spaces.
