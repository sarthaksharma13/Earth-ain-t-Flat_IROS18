/*
Copyright (C) 2014 Jerome Revaud

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
*/
#ifndef ___DEEP_MATCHING_H___
#define ___DEEP_MATCHING_H___
#include "array_types.h"
#include "pixel_desc.h"
#include "image.h"

#include <vector>
using namespace std;


// deep matching parameters
typedef struct {
  desc_params_t desc_params; 
  
  int prior_img_downscale;// downscale the image by 2^(this) prior to matching
  int overlap;            // pyramid level at which patches starts to overlap (999 => no overlap at all)
  bool subsample_ref;     // true if larger patches higher in the pyramid are not densely sampled
  float nlpow;            // non-linear power rectification
  int maxima_mode;        // 1: standard / 0: from all top-level patches
  int min_level;          // minimum pyramid level to retrieve maxima
  int low_mem;            // use less memory to retrieve the maxima (but approximate result)
  int scoring_mode;       // 0: like ICCV paper / 1: improved scoring mode
  int verbose;            // verbosity
  int n_thread;           // parallelization on several cores, when possible
  
} dm_params_t;

// set default parameters
void set_default_dm_params( dm_params_t* params );


// response maps at a given scale
typedef struct {
  int f;                // subsampling factor with respect to original image size
  int patch_size;       // patch size in original image coordinates in first image
  int_cube grid;        // position (center) of each patch in first image
  float_image norms;    // norm of each patch in first image
  int_array assign;     // mapping between patches and their response maps
  float_layers res_map; // response map of the patches on the second image
  float_layers max_map; // max-filtered response map
  int_cube children;    // index of children patches in the previous level
  float_layers passed;  // remember the best score so far at each response when doing argmax
  
} res_scale;  

typedef vector<res_scale> matching_pyramid_t;


// output correspondences
typedef struct {
  float x0, y0;   // position in first image (reference image)
  float x1, y1;   // position in second image (target image)
  float maxima;   // from which maxima it was generated (index)
  float score;    // matching score
} corres_t;

// main function. Returns a float_image where each row is <corres_t>
float_image* deep_matching( image_t* img0, image_t* img1, const dm_params_t* params );


#endif



































