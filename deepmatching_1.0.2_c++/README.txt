Implementation of the Deep Matching algorithm, published at ICCV 2013 in
"DeepFlow: Large displacement optical flow with deep matching" by Philippe 
Weinzaepfel, Jerome Revaud, Zaid Harchaoui and Cordelia Schmid.
Code and idea by Jerome Revaud, INRIA. The code is only for scientific 
or personnal use. Please contact me/INRIA for commercial use.
Email: jerome.revaud@inria.fr

Copyright (C) 2014 Jerome Revaud

Version 1.0.2

License:

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>


Installation:
  
  make clean all
  
  This program has been built on a fedora18 x64 machine. *No assistance* will be given to 
  compile the code on other OS. However, if you are able to sucessfully adapt the code
  for other platforms (Windows, Mac OS, Matlab wrapper etc.), please notify me so that I
  can release these versions on my webpage.



Example usage:

  ./deepmatching --help

  ./deepmatching liberty1.png liberty2.png -iccv_settings -v
  should produce the following output:
    layer 0, patch_size = 16x16
    remaining 49 big cells (actually, 49 are unique)
    layer 1, patch_size = 32x32
    remaining 9 big cells (actually, 9 are unique)
    layer 2, patch_size = 64x64
    remaining 1 big cells (actually, 1 are unique)
    found 1 local matches
    gathering correspondences 0%...
    8 8 0 12 1.41813 0
    8 40 4 48 1.42918 0
    8 16 0 20 1.40409 0
    32 8 28 8 1.28573 0
    32 56 32 48 1.39807 0
    32 16 28 16 1.41907 0
    40 32 40 24 1.41519 0
    56 32 56 20 1.45184 0
    56 24 56 12 1.42266 0
    24 40 24 32 1.38309 0
    24 56 28 60 1.37839 0
  
  Note that for the ICCV paper, we used the following options: "-iccv_settings -resize 1024 512"
  However, we reckon that just using "-improved_settings" generally produces (slightly) better results.
  
  To visualize the output matching:
  ./deepmatching ambush_6_0004.png ambush_6_0005.png -improved_settings -nt 4 | python viz.py ambush_6_0004.png ambush_6_0005.png
  
  Notice the importance of the "-jpg_settings" versus "-png_settings" by comparing the results of:
  ./deepmatching grimpe1.png grimpe2.png -jpg_settings -nt 8 | py viz.py grimpe1.png grimpe2.png
  ./deepmatching grimpe1.png grimpe2.png -png_settings -nt 8 | py viz.py grimpe1.png grimpe2.png
   (in this example, the images were originally MPEG-compressed video frames so the '-jpg_settings' option should be preferred.)
  
  To rescore matches prior to calling deepflow:
  ./deepmatching ambush_6_0004.png ambush_6_0005.png -improved_settings -nt 4 | python rescore.py ambush_6_0004.png ambush_6_0005.png
  

For details about the options, please refer to the help, the paper or the code.



Version history:

  version 1.0.2:
  Many thanks to Bowen Zhang from Tongji University for reporting an issue with the makefile































