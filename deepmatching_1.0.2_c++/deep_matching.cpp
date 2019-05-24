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
#include "deep_matching.h"
#include "std.h"
#include "conv.h"
#include "maxfilter.h"


// return size of atomic patches
int get_atomic_patch_size( const dm_params_t* params )
{
  int upsize = (1 << params->prior_img_downscale);
  return 4*upsize;
}

// extract pixel descriptor for both images
void extract_image_desc( image_t* img0, image_t* img1, const dm_params_t* params,
                         float_layers** desc0, float_layers** desc1 )
{
    // slightly reduce img0 size to fit the patch tiling
    int patch_size = get_atomic_patch_size( params );
    
    // crop the reference image to a multiple of patch size
    int width = patch_size*int(img0->width/patch_size);
    int height= patch_size*int(img0->height/patch_size);
    
    image_crop(img0, width, height);
    
    // extract gradient-based information
    *desc0 = extract_desc( img0, &params->desc_params, params->n_thread );
    *desc1 = extract_desc( img1, &params->desc_params, params->n_thread );
}


void avgpool2( float_layers* hog, const dm_params_t* params )
{
    int niter = params->prior_img_downscale;
    while(niter--) {
      float_layers res = empty_layers(float,hog->tx/2,hog->ty/2,hog->tz);
      _avgpool2(hog,&res,params->n_thread);
      
      // replace hog by res
      free(hog->pixels);
      *hog = res;
    }
}


/* compute the grid of parent cell position, and their connection to children cells
   cells can be half-overlapping if <overlap>=1
   <dense_step> forces the grid spacing if >0
*/
void prepare_big_cells( const int imshape[2], int cell_size, int overlap, int child_overlap, 
                        int_cube* child_grid, float_image* child_norms, int dense_step,
                        int_cube* grid, int_cube* children, float_image* norms )
{
    int offset, step, gtx, gty;
    if( dense_step ) {
      step = dense_step;
      offset = 0;
      // we do not care if the patches are overlapping outside the image
      #define grid_size(imsize) (1+imsize/step)
      gtx = grid_size(imshape[0]);
      gty = grid_size(imshape[1]);
      #undef grid_size
    } else {
      // we want patches fully included in the image
      offset = cell_size/2;
      step = cell_size/(overlap+1);
      #define grid_size(imsize) (1+MAX(0,imsize-2*offset)/step)
      gtx = grid_size(imshape[0]);
      gty = grid_size(imshape[1]);
      #undef grid_size
    }
    
    assert(!grid->pixels);
    *grid = empty_cube(int,gtx,gty,2);
    
    assert(0<=overlap && overlap<=1);
    int nc = pow2(2+child_overlap);  // number of children per cell
    if(child_grid) {
      assert(!norms->pixels);
      *norms = image_like(float,grid);
      assert(!children->pixels);
      *children = empty_cube(int,gtx,gty,nc);
    }
    
    _prepare_big_cells( cell_size, offset, step, child_grid, child_norms, grid, children, norms );
}

int get_patch_desc_dim( float_layers* hog, int patch_size ) 
{
    return patch_size*patch_size * hog->tz; // number of dimensions of an atomic patch descriptor
}

void sample_patches( float_layers* hog, int_cube* pos, int patch_size, int f, float norm, int n_thread, 
                     float_image* patches, float_array* norms ) 
{
    assert(norm>0);
    const int npos = pos->tx*pos->ty;
    int_image new_pos = empty_image(int,2,npos);
    for(int i=0; i<2*npos; i++)
      new_pos.pixels[i] = (pos->pixels[i]-patch_size/2)/f;
    
    patch_size /= f;
    const int nh = get_patch_desc_dim(hog,patch_size);
    
    assert(!patches->pixels);
    *patches = empty_image(float,nh,npos);
    assert(norms->tx==npos);
    
    _sample_patches( hog, NULL, &new_pos, patch_size, norm, patches, norms, n_thread );
    
    free(new_pos.pixels);
}

float_layers* prepare_dotprod_convolution( float_layers* hog, int patch_size, int extend, float norm, int nt )
{
  assert(0<=extend and extend<=1);
  const int nh = get_patch_desc_dim(hog,patch_size);
  const int etx = hog->tx+extend; // extend a bit the image
  const int ety = hog->ty+extend;
  
  float_layers* res = NEW(float_layers);
  *res = empty_layers(float,etx,ety,nh);
  
  float ninth_val = 0;
  _prepare_dotprod_convolution( hog, patch_size, ninth_val, extend, res, nt );
  
  if( norm )  norm_layers( res, norm, nt );
  return res;
}


extern "C" {
int sgemm_(char *transa, char *transb, integer *m, integer *
           n, integer *k, float *alpha, float *a, integer *lda, float *b, integer *
           ldb, float *beta, float *c, integer *ldc);
}

void fastconv( float_image* patches, float_layers* hog, int patch_size, int extend, float norm, int nt,
               float_layers* res ) 
{
  assert(0<=extend and extend<=1);
  float_layers* convolved_hog = prepare_dotprod_convolution( hog, patch_size, extend, norm, nt );
  assert( patches->tx==convolved_hog->tz);
  int nh = get_patch_desc_dim(hog,patch_size);
  
  /* matrix matrix multiplication
     res = patches * convolved_hog
     PxI   P x nh     nh x I
  */
  int P = patches->ty;
  int I = convolved_hog->tx*convolved_hog->ty;
  *res = empty_layers(float,convolved_hog->tx,convolved_hog->ty,P);
  
  // multi-threaded fast matrix product
  char T='n'; float alpha = 1, beta = 0;
  sgemm_(&T,&T,&I,&P,&nh,&alpha,convolved_hog->pixels,&I,patches->pixels,&nh,&beta,res->pixels,&I);
  free_layers(convolved_hog);
  
  rectify_conv( patch_size, hog->tz, patches, extend, res, nt ); 
}

void convolve_atomic_patches( float_layers* source, float_layers* target, int psize, int f, 
                              const dm_params_t* params, int_cube* grid, 
                              float_image* norms, int_array* assign, float_layers* res_map ) 
{
    const int extend = 1; // slightly spatially extend response maps
    const float norm = 1; // renorm patches
    
    // first, sample patches
    float_image patches = {0};
    assert(!norms->pixels);
    *norms = image_like(float, grid);
    float_array norms_arr = {norms->pixels,norms->tx*norms->ty};
    sample_patches( source, grid, psize, f, norm, params->n_thread, &patches, &norms_arr );
    //hash_image(&patches)
    
    // rectify the norm to a boolean (0 or 1) (useless ?)
    *assign = empty_array(int,norms_arr.tx);
    int n=0, tx = patches.tx;
    for(int i=0; i<norms_arr.tx; i++) {
      norms_arr.pixels[i] = norms_arr.pixels[i]>0;
      
      // eliminate zero-norm patches
      if( norms_arr.pixels[i] ) {
        if( n < i ) // copy
          memcpy( patches.pixels + n*tx, patches.pixels + i*tx, tx*sizeof(float));
        assign->pixels[i] = n++;
//        assign->pixels[i] = i;
      } else 
        assign->pixels[i] = -1;
    }
    patches.ty = n; // update new number of valid patches
    
    //hash_image(norms)
    //hash_image(&patches)
    
    // compute the first level convolutions
    fastconv( &patches, target, psize/f, extend, norm, params->n_thread, res_map );
    
    free(patches.pixels);
}

void subsample2( float_layers* hog, float_layers* res, int nt )
{
  assert(!res->pixels);
  *res = empty_layers(float,(hog->tx+1)/2,(hog->ty+1)/2,hog->tz);
  _subsample2( hog, res, nt );
}

/* aggregate response maps of children patches to form response maps of parent patches */
int sparse_conv( int_cube* children, int_array* children_assign, float_image* child_norms, 
                 int true_patch_size, float_layers* map, int nt,
                 float_image* norms, int_array* assign, float_layers* res )
{
  float_layers ext_map;
  if( MIN(map->tx,map->ty) < 5 ) {
    ext_map = zeros_layers(float,MAX(5,map->tx),MAX(5,map->ty),map->tz);
    for(int l=0; l<map->tz; l++)
      for(int j=0; j<map->ty; j++)
        for(int i=0; i<map->tx; i++)
          ext_map.pixels[(l*ext_map.ty + j)*ext_map.tx + i] = map->pixels[(l*map->ty + j)*map->tx + i];
    map = &ext_map;
  }
  
  int_image _children = reshape_z_xy(int,children);
  
  assert(!res->pixels);
  *res = empty_layers(float,map->tx,map->ty,_children.ty);
  int gap = true_patch_size / 4;
  assert(gap>0);
  float_array _norms = reshape_xy(float, norms);
  float_array _child_norms = reshape_xy(float, child_norms);
  
  // allocate useless assign
  *assign = empty_array(int,res->tz);
  for(int i=0; i<assign->tx; i++) assign->pixels[i] = i;
  
  float trans_inv = 0.9f; 
  int_array* _assign = NULL;
  int_array* _ch_assign = children_assign->pixels ? children_assign : NULL;
  int n = _sparse_conv( &_children, _ch_assign, gap, trans_inv, map, &_child_norms, &_norms, _assign, res, nt );
  
  if(map==&ext_map) free(ext_map.pixels);
  return n;
}

res_scale new_pyramid_level(int f, int psize) 
{
  res_scale res = {0};
  res.f = f;                    // subsampling factor with respect to original image size
  res.patch_size = psize;       // patch size in original image coordinates
  return res;
}

// Compute the multi-scale pyramid response
void compute_matching_pyr( float_layers* source, float_layers* target, const dm_params_t* params,
                              matching_pyramid_t& res_maps )
{
    const int src_shape[2] = {source->tx, source->ty};
    int L = 0;  // current pyramid level
    const int atomic_psize = get_atomic_patch_size( params );
    int psize = atomic_psize; // will grow by a factor 2 at each level
    int f = psize/4;  // initial scaling factor
    
    // subsample if needed
    avgpool2( source, params );
    avgpool2( target, params );
    
    //hash_layers(source)
    //hash_layers(target)
    
    res_maps.clear();
    res_maps.push_back(new_pyramid_level(f,psize));
    res_scale *child, *last = &res_maps[res_maps.size()-1];
    
    // compute the initial patches in source image
    if( params->verbose ) printf("layer %d, patch_size = %dx%d\n", L, psize, psize);
    prepare_big_cells( src_shape, psize, params->overlap<L+1, 0, NULL, NULL, 0, &last->grid, NULL, NULL );
    //hash_cube(&last->grid)
    
    //hash_layers(source)
    convolve_atomic_patches( source, target, psize, f, params, &last->grid, &last->norms, 
                             &last->assign, &last->res_map );
    //hash_layers(&last->res_map)
    if( params->verbose ) 
      printf("remaining %d big cells (actually, %d are unique)\n", IMG_SIZE(&last->grid), last->res_map.tz);
    
    // non-linear correction
    if( params->nlpow>0 ) 
      fastipow( &last->res_map, params->nlpow, params->n_thread );
    
    //hash_layers(&last->res_map)
    
    const int dense_step = params->subsample_ref ? 0 : psize/(1+(params->overlap<1));
    const int max_psize = 999;  // it's quite useless to go too deep
    
    // aggregate patches for all subsequent levels
    while( 2*psize <= MIN(max_psize, MAX(src_shape[0], src_shape[1])) )
    {
        L++;
        f *= 2;
        psize *= 2;
        res_maps.push_back(new_pyramid_level(f,psize));
        child = &res_maps[res_maps.size()-2]; // previous level
        last = &res_maps[res_maps.size()-1];  // current level
        if( params->verbose ) printf("layer %d, patch_size = %dx%d\n", L, psize, psize);
        
        // max pooling + subsampling
        float_layers maxpooled_res_map = layers_like(float,&child->res_map);
        _max_filter_3_layers( &child->res_map, &maxpooled_res_map, params->n_thread);
        float_layers subs_res_map = {0};
        subsample2( &maxpooled_res_map, &subs_res_map, params->n_thread );
        free(maxpooled_res_map.pixels);
        
        // build the set of patches at this scale
        prepare_big_cells( src_shape, psize, params->overlap<L+1, params->overlap<L, 
                           &child->grid, &child->norms, dense_step, &last->grid, &last->children, &last->norms );
        //hash_cube(&last->grid)
        //hash_image(&last->norms)
        //hash_cube(&last->children)
        
        // aggregate children response maps to form parent response maps
        sparse_conv( &last->children, &child->assign, &child->norms, psize/f, &subs_res_map, 
                     params->n_thread, &last->norms, &last->assign, &last->res_map );
        free(subs_res_map.pixels);
        if( params->verbose ) 
          printf("remaining %d big cells (actually, %d are unique)\n", IMG_SIZE(&last->grid), last->res_map.tz);
        
        // non-linear correction
        if( params->nlpow>0 ) 
          fastipow(&last->res_map, params->nlpow, params->n_thread );
        //hash_layers(&last->res_map)
    }
}


void free_matching_pyramid( matching_pyramid_t& res_maps ) {
  unsigned int i;
  for(i=0; i<res_maps.size(); i++) {
    res_scale& level = res_maps[i];
    
    free(level.grid.pixels);
    free(level.norms.pixels);
    free(level.assign.pixels);
    free(level.res_map.pixels);
    free(level.max_map.pixels);
    free(level.children.pixels);
    free(level.passed.pixels);
  }
}


#ifdef __APPLE__
static int arg_sort_maxima(void* arr, const void* a, const void* b) {
  float diff = ((float*)arr)[5*(*(int*)a)+4] - ((float*)arr)[5*(*(int*)b)+4];
  return (diff<0) - (diff>0); // descending order
}
#else
static int arg_sort_maxima(const void* a, const void* b, void* arr) {
  float diff = ((float*)arr)[5*(*(int*)a)+4] - ((float*)arr)[5*(*(int*)b)+4];
  return (diff<0) - (diff>0); // descending order
}
#endif

void reorder_rows( int_image* img, int_array* order )
{
  assert(order->tx==img->ty);
  const int tx = img->tx;
  int_image res = image_like(int, img);
  
  for(int i=0; i<order->tx; i++)
    memcpy(res.pixels + i*tx, img->pixels+order->pixels[i]*tx, tx*sizeof(int));
  
  free(img->pixels);
  *img = res;
}

// return points corresponding to patch matches 
int_image* find_optimal_matchings( matching_pyramid_t& mp, const dm_params_t* params ) 
{
  const int nobordure = 0;
  int_image* maxima = NEW(int_image);
  int_array order = {0};
  
  if( params->maxima_mode ) { // normal process: maxima detection
    
    float th=0;
    int check_parents=false, check_children=false;
    
    float_array sc_maxima = empty_array(float,int(mp.size()));
    for(unsigned int i=0; i<mp.size(); i++) sc_maxima.pixels[i]=1;  // useless but well
    
    _extract_maxima( mp.data(), mp.size(), &sc_maxima, th, params->min_level, params->nlpow, 
                     check_parents, check_children, nobordure, maxima, params->n_thread );
    free(sc_maxima.pixels);
    
    order = empty_array(int,maxima->ty); 
    for(int i=0; i<maxima->ty; i++) order.pixels[i] = maxima->ty-1-i;  // last first
    
  } else { // we just analyse all cells at the top level
    const float_layers* rmap = &mp[mp.size()-1].res_map;
    const int tx = rmap->tx, txy=tx*rmap->ty;
    *maxima = empty_image(int, 5, LAYERS_SIZE(rmap));
    
    for(int i=0; i<maxima->ty; i++) {
      int* row = maxima->pixels + 5*i;
      row[0] = mp.size()-1; // pyramid level
      row[1] = i/txy;       // layer number
      row[2] = i%tx;        // x position
      row[3] = (i%txy)/tx;  // y position
      ((float*)row)[4] = rmap->pixels[i];
    }
    //hash_image(maxima)
    
    order = empty_array(int,maxima->ty); 
    for(int i=0; i<maxima->ty; i++) order.pixels[i] = i;
    #ifdef __APPLE__
    qsort_r(order.pixels, maxima->ty, sizeof(int), maxima->pixels, arg_sort_maxima);
    #else
    qsort_r(order.pixels, maxima->ty, sizeof(int), arg_sort_maxima, maxima->pixels);
    #endif
  }
  
  if( params->verbose>0 ) 
    printf("found %d local matches\n",maxima->ty);
  
  // reorder maxima
  reorder_rows( maxima, &order );
  free(order.pixels);
  return maxima;
}



/* this function gather correspondences from each local maximum in the 
  response maps
*/
float_image* gather_correspondences( int src_shape[2], int target_shape[2], 
                                   matching_pyramid_t& scales, int_image* maxima,
                                   const dm_params_t* params )
{
    const int step = 4*scales[0].f; // bin size
    const int n_scales = (int)scales.size();
    const int tx = maxima->tx;
    const int n_maxima = maxima->ty;
    
    float_cube corres0 = zeros_cube(float, (src_shape[0]+step-1)/step, (src_shape[1]+step-1)/step,6);
    float_cube corres1 = zeros_cube(float, (target_shape[0]+step-1)/step, (target_shape[1]+step-1)/step,6);
    
    int i;
    // allocate temporary optimization maps
    for(i=0; i<n_scales; i++) {
      int size = scales[i].res_map.tx*scales[i].res_map.ty*scales[i].res_map.tz;
      if( params->low_mem && size > 1000003 ) size = 1000003; // big prime
      scales[i].passed = zeros_layers(float, size, 1, 1);
    }
    
    #if defined(USE_OPENMP)
    #pragma omp parallel for schedule(static,1) num_threads(params->n_thread)
    #endif
    for(i=0; i<n_maxima; i++) {
      if(params->verbose && i%100==0) printf("\rgathering correspondences %d%%...",100*i/n_maxima);
      int* m = maxima->pixels + tx*i;
      assert(m[0]<n_scales);
      if( params->scoring_mode )  // new mode
        _argmax_correspondences( scales.data(), m[0], m[1], m[2], m[3], ((float*)m)[4], 
                                    &corres0, step, &corres1, step, i );
      else  // old iccv mode
        _argmax_correspondences_v1( scales.data(), m[0], m[1], m[2], m[3], m[0]*((float*)m)[4], 
                                    &corres0, step, &corres1, step, i );
    }
    
    // free optimization maps
    for(i=0; i<n_scales; i++) {
      free( scales[i].passed.pixels );
      scales[i].passed.pixels = NULL;
    }
    
    if(params->verbose) printf("\n");
    
    // keep only reciprocal matches
    int nres;
    float* corres = _intersect_corres( &corres0, &corres1, &nres );
    float_image* res = NEW(float_image);
    *res = (float_image){corres, 6, nres};
    
    free(corres0.pixels);
    free(corres1.pixels);
    return res;
}


// set default parameters
void set_default_dm_params( dm_params_t* params )
{
  // pixel descriptor params
  set_default_desc_params( &params->desc_params );
  
  // general parameters
  params->prior_img_downscale = 1;
  params->overlap = 999;
  params->subsample_ref = false;
  params->nlpow = 1.6;
  params->maxima_mode = 0;
  params->min_level = 2;
  params->low_mem = true;
  params->verbose = 0;
  params->scoring_mode = 1;
  params->n_thread = 1;
}


// main function
float_image* deep_matching( image_t* img0, image_t* img1, const dm_params_t* params )
{
  // verify parameters
  assert(between(0,params->prior_img_downscale,3));
  assert(between(0,params->overlap,999));
  assert(between(0,params->subsample_ref,1));
  assert(between(1,params->nlpow,2));
  assert(between(0,params->maxima_mode,1));
  assert(between(0,params->min_level,4));
  assert(between(0,params->low_mem,1));
  assert(between(0,params->scoring_mode,1));
  assert(between(0,params->verbose,1));
  assert(between(0,params->n_thread,128));
  
  // extract pixel descriptors
  float_layers *source, *target;
  extract_image_desc( img0, img1, params, &source, &target );
  int src_shape[2] = {source->tx, source->ty};
  int target_shape[2] = {target->tx, target->ty};
  
  //hash_layers(source)
  //hash_layers(target)
  
  // compute local matchings
  matching_pyramid_t matching_pyr;
  compute_matching_pyr( source, target, params, matching_pyr );
  free_layers(source);
  free_layers(target);
  
  //hash_layers(&matching_pyr[matching_pyr.size()-1].res_map);
  
  // find optmial matchings (maxima)
  int_image* maxima = find_optimal_matchings(matching_pyr, params);
  
  //hash_image(maxima);
  
  // select the best displacements (maxpool merge)
  float_image* corres = gather_correspondences( src_shape, target_shape, matching_pyr, maxima, params );
  
  //hash_image(corres);D(9)getchar();
  
  // free everything
  free_matching_pyramid(matching_pyr);
  free_layers(maxima);
  
  return corres;
}



























