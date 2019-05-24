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
#include "conv.h"
#include "std.h"
#include "omp.h"
#include "maxfilter.h"

extern "C" {
#include <immintrin.h>
#define integer int
#define real float
extern int saxpy_(integer *n, real *sa, real *sx, integer *incx, real *sy, integer *incy);
extern int sscal_(integer *n, real *sa, real *sx, integer *incx);
}


static inline void fast_set_val( float * __restrict__ a, long d, const float val) {
  if(val) {
    int j;
    for(j=0; j<d; j++)
      a[j] = val;
  } else
    memset(a,0,d*sizeof(float));
}
static inline void fast_add_val( float * __restrict__ a, long d, const float val) {
  int j;
  for(j=0; j<d; j++)
    a[j] += val;
}
static inline void fast_set_vec( float * __restrict__ dest,
                                 const float * __restrict__ src, int d, const float mul) {
  if( mul==1) 
    memcpy(dest,src,d*sizeof(float));
  else {
    int j;
    for(j=0; j<d; j++)
      dest[j] = mul*src[j];
  }
}
static inline void fast_add_vec( float * __restrict__ dest,
                                 const float * __restrict__ add, int d, float mul) {
  if(d<=4) {
    int j;
    for(j=0; j<d; j++)
      dest[j] += mul*add[j];
  } else {
    int inc = 1;
    saxpy_( &d, &mul, (float*)add, &inc, (float*)dest, &inc );
  }
}
static inline void fast_div( float * __restrict__ a, long d, const float div) {
  const float divi = 1/div;
//  assert( ((long)a & 15) == 0 && (d & 3) == 0 );
//  const float _divi4[] = {divi,divi,divi,divi};
//  __v4sf *a4 = (__v4sf*)a;
//  __v4sf *divi4 = (__v4sf*)_divi4;
//  int e = d>>2;
//  while(e--) *a4++ *= (*divi4);
  int j;
  for(j=0; j<d; j++)
    a[j] *= divi;
}

static inline float* fast_set_trans( float * dest, float * src, const float mul,
                                     int dx, int dy, const int tx, const int ty, const int ex, const float def ) {
  if(mul==0)  {
    memset(dest,0,sizeof(float)*(tx+ex)*(ty+ex));
    return dest+(tx+ex)*(ty+ex);
  }
  if(dx>tx) dx=tx;  // after those alues, nothing happens anyway
  if(dy>ty) dy=ty;
  if(-dx>tx) dx=-tx;
  if(-dy>ty) dy=-ty;
  
  #define add_default(n)  {fast_set_val(dest,(n),mul*def); dest+=(n);}
  float* _dest = dest;
  
  // paste -v zeros rows
  if(dy<0) add_default(-dy*(tx+ex));
  
  src += MAX(0,dx);
  const int row_len = MIN(tx,tx+dx+ex) - MAX(0,dx);
  int j;
  for(j=MAX(0,dy); j<MIN(ty,ty+dy+ex); j++) {
    
    // paste -u zeros cols
    if(dx<0) add_default(-dx);
    
    // past image
    fast_set_vec(dest,src+j*tx,row_len,mul);
    dest += row_len;
    
    // paste +u zeros cols
    if(dx>=0) {add_default(dx)
    if(ex) add_default(ex)}
  }
  
  // paste +v zeros rows
  if(dy>=0){add_default(dy*(tx+ex))
  if(ex) add_default(ex*(tx+ex))}
  
  #undef add_default
  assert( dest-_dest == (tx+ex)*(ty+ex) );
  return dest;
}

static inline float* fast_add_trans( float * dest, float * src, const float mul,
                                     int dx, int dy, const int tx, const int ty, const int ex, const float def ) {
  if(mul==0)  return dest+(tx+ex)*(ty+ex);
  if(dx>tx) dx=tx;  // after those alues, nothing happens anyway
  if(dy>ty) dy=ty;
  if(-dx>tx) dx=-tx;
  if(-dy>ty) dy=-ty;
  #define add_default(n)  {fast_add_val(dest,n,def*mul); dest+=n;}
  float* _dest = dest;
  
  // paste -v zeros rows
  if(dy<0) add_default(-dy*(tx+ex));
  
  src += MAX(0,dx);
  const int row_len = MIN(tx,tx+dx+ex) - MAX(0,dx);
  int j;
  for(j=MAX(0,dy); j<MIN(ty,ty+dy+ex); j++) {
    
    // paste -u zeros cols
    if(dx<0) add_default(-dx);
    
    // past image
    fast_add_vec(dest,src+j*tx,row_len,mul);
    dest += row_len;
    
    // paste +u zeros cols
    if(dx>=0) {add_default(dx)
    if(ex) add_default(ex)}
  }
  
  // paste +v zeros rows
  if(dy>=0){add_default(dy*(tx+ex))
  if(ex) add_default(ex*(tx+ex))}
  
  #undef add_default
  assert( dest-_dest == (tx+ex)*(ty+ex) );
  return dest;
}


static inline void norm_norm( float* norms, int nb, float mode ) {
    int i;
    if( mode < 0 ) 
      assert(!"error: unknown norm mode");
    else if( mode == 0.5 ) {
      for(i=0; i<nb; i++)
        norms[i] = sqrt(sqrt(norms[i]));
    } else if( mode < 1 ) {
      mode *= 0.5;  // cumulate with initial 1/sqrt(.)
      for(i=0; i<nb; i++)
        norms[i] = pow(norms[i], mode);
    } else if( mode == 1 ) {
      for(i=0; i<nb; i++)
        norms[i] = sqrt(norms[i]);
    } else if( mode > 1 )
      assert(!"error: unknown norm mode");
}


void norm_layers( float_layers* res, float norm, int n_thread ) {
  if(norm==0) return;
  
  const int layer_size = res->tx*res->ty;
  const int n_layers = res->tz;
  float* norms = NEWAC(float,layer_size);
  int l;
  
  for(l=0; l<n_layers; l++) {
    float* r = res->pixels + l*layer_size; 
    int i;
    #if defined(USE_OPENMP)
    #pragma omp parallel for num_threads(n_thread)
    #endif
    for(i=0; i<layer_size; i++)
      norms[i] += r[i]*r[i];
  }
  norm_norm( norms, layer_size, norm );
  
  for(l=0; l<n_layers; l++) {
    float* r = res->pixels + l*layer_size; 
    int i;
    #if defined(USE_OPENMP)
    #pragma omp parallel for num_threads(n_thread)
    #endif
    for(i=0; i<layer_size; i++)
      r[i] /= norms[i]+1e-8;
  }
  
  free(norms);
}



/* Prepare image for dotprod : dot(patches, res)
   where patches is n_patches x patch_dim
   set outside of the image to be equal to (0,...,ninth_val)
*/
void _prepare_dotprod_convolution( float_layers* img, int patch_size, float ninth_val, int extend, 
                                   float_layers* res, int n_thread ) {
  assert( img->tx+extend == res->tx );
  assert( img->ty+extend == res->ty );
  const int n_layers = img->tz;
  const int tx = img->tx;
  const int ty = img->ty;
  const int npix = tx*ty;
  const int npixex = (tx+extend)*(ty+extend);
  assert( res->tz==patch_size*patch_size*img->tz );
  
  int l;
  const int hs = patch_size/2;  // half-size
  const int layer_size = patch_size*patch_size*npixex;
  
  #if defined(USE_OPENMP)
  #pragma omp parallel for num_threads(n_thread)
  #endif
  for(l=0; l<n_layers; l++) {
    float* img_pix = img->pixels + l*npix;
    float* r = res->pixels + l*layer_size; 
    int u,v;
    // copy translated version of the image into res
    for(v=-hs; v<hs; v++)
      for(u=-hs; u<hs; u++)
        r = fast_set_trans( r, img_pix, 1, u, v, tx, ty, extend, l+1<n_layers? 0 : ninth_val );
  }
}




/* Prepare image for dotprod with a subsampled2 patch: dot(patches, res)
   where patches is n_patches x patch_dim
   set outside of the image to be equal to (0,...,ninth_val)
*/
void _prepare_dotprod_convolution2( float_layers* img, int patch_size, float ninth_val, int extend, 
                                    float_layers* res, int n_thread ) {
  assert( img->tx+extend == res->tx );
  assert( img->ty+extend == res->ty );
  const int n_layers = img->tz;
  const int tx = img->tx;
  const int ty = img->ty;
  const int npix = tx*ty;
  const int npixex = (tx+extend)*(ty+extend);
  assert( res->tz==patch_size*patch_size*img->tz );
  
  int l;
  const int hs = patch_size/2;  // half-size
  const int layer_size = patch_size*patch_size*npixex;
  
  #if defined(USE_OPENMP)
  #pragma omp parallel for num_threads(n_thread)
  #endif
  for(l=0; l<n_layers; l++) {
    float* img_pix = img->pixels + l*npix;
    float* r = res->pixels + l*layer_size; 
    int u,v;
    // copy translated version of the image into res
    for(v=-hs; v<hs; v++)
      for(u=-hs; u<hs; u++) 
        r = fast_set_trans( r, img_pix, 1, 2*u, 2*v, tx, ty, extend, l+1<n_layers? 0 : ninth_val );
  }
}



/* Sample a set of patches from a HOG image.
   grid : array of (x,y) position of the patches
   size: size of the patches, ie. [x,x+size[ x [y,y+size[
   res: result array, n_patches x desc_dim
        desc_dim = n_layers * size**2
   norms: result, n_patches x 1, norm of each patch
*/
void _sample_patches( float_layers* hog, float_layers* color, int_image* grid, int size, float norm, 
                      float_image* res, float_array* norms, int n_thread ) {
  const int tx = hog->tx;
  const int npix = tx*hog->ty;
  assert( grid->tx == 2 );
  const int n_patches = grid->ty;
  assert( res->ty == n_patches );
  const int n_layers = hog->tz;
  const int n_colors = (color? color->tz: 0);
  const int color_npix = (color? color->tx*color->ty: 0);
  const int desc_size = size*size*n_layers + (color? color->tz: 0);
  assert(res->tx == desc_size );
  
  int n;
  
  #if defined(USE_OPENMP)
  #pragma omp parallel for num_threads(n_thread)
  #endif
  for(n=0; n<n_patches; n++) {
    float *r = res->pixels + desc_size*n;
    int *p = grid->pixels + 2*n;
    // copy hog
    int x=p[0],y=p[1];
    assert(0<=x && x+size<=tx);
    assert(0<=y && y+size<=hog->ty);
    int l,j;
    for(l=0; l<n_layers; l++) {
      float* h = hog->pixels + l*npix + y*tx + x;
      for(j=0; j<size; j++) {
        memcpy(r, h, size*sizeof(float));
        h += tx;
        r += size;
      }
    }
    if(!color)  continue;
    // copy color
    float* c = color->pixels + (y+size/2)*color->ty + (x+size/2);
    for(l=0; l<n_colors; l++) 
      *r++ = c[l*color_npix];
  }
  
  if(norm) {
    float* normp = norms ? norms->pixels : NEWAC(float, n_patches);
    if(norms) {
      assert(norms->tx==n_patches);
      memset(normp,0,n_patches*sizeof(float));
    }
    
    #if defined(USE_OPENMP)
    #pragma omp parallel for num_threads(n_thread)
    #endif
    for(n=0; n<n_patches; n++) {
      float *r = res->pixels + desc_size*n;
      int l;
      for(l=0; l<desc_size; l++)
        normp[n] += r[l]*r[l];
    }
    norm_norm( normp, n_patches, norm );
    
    #if defined(USE_OPENMP)
    #pragma omp parallel for num_threads(n_thread)
    #endif
    for(n=0; n<n_patches; n++) {
      float *r = res->pixels + desc_size*n;
      int l;
      float nn = normp[n]+1e-8;
      for(l=0; l<desc_size; l++)
        r[l] /= nn;
    }
    
    if(!norms)  free(normp);
  }
}


inline float sum_array_f(const float* a, int n) {
  int i=n;
  double res = 0;
  while(i--)  res+=a[i];
  return (float)res;
}


/* correct the convolution on the boundaries of the image
*/
void rectify_conv( int patch_size, int nori, float_image* patches, int extend, float_layers* res, int n_thread ) {
  const int n_patches = patches->ty;
  assert( n_patches == res->tz );
  //const int nori = patches->tx/pow2(patch_size);
  assert( patches->tx >= nori*pow2(patch_size) );
  const int tx = res->tx - extend;  // because it has been extended
  const int ty = res->ty - extend;  // because it has been extended
  const int etx = res->tx;
  const int ety = res->ty;
  
  int l;
  #if defined(USE_OPENMP)
  #pragma omp parallel for num_threads(n_thread)
  #endif
  for(l=0; l<n_patches; l++) {
    
    float sums[8]; // temporary norm of columns or rows
    assert( patch_size <= (int)(sizeof(sums)/sizeof(sums[0])) );
    int o,i,j;
    
    // horizontal boundaries
    memset(sums,0,sizeof(sums));
    float* p = patches->pixels + l*patches->tx;
    for(o=0; o<nori; o++)
      for(j=0; j<patch_size; j++)
        for(i=0; i<patch_size; i++)
          sums[j] += pow2(*p++);
    
    float old_norm = sqrt(sum_array_f(sums,patch_size));
    if( old_norm==0 ) continue;
    
    // upper boundary
    for(j=0; j<patch_size/2; j++) {
      float new_norm = sqrt(sum_array_f(sums+(patch_size/2-j),patch_size/2+j));
      float mul = old_norm / (new_norm + 1e-8);
      float* r = res->pixels + l*etx*ety + j*etx;
      for(i=0; i<etx; i++) 
        r[i] *= mul;
    }
    // lower boundary
    for(j=ty+1-patch_size/2; j<ety; j++) {
      float new_norm = sqrt(sum_array_f(sums,patch_size/2+ty-j));
      float mul = old_norm / (new_norm + 1e-8);
      float* r = res->pixels + l*etx*ety + j*etx;
      for(i=0; i<etx; i++) 
        r[i] *= mul;
    }
    
    // vertical boundaries
    memset(sums,0,sizeof(sums));
    p = patches->pixels + l*patches->tx;
    for(o=0; o<nori; o++)
      for(j=0; j<patch_size; j++)
        for(i=0; i<patch_size; i++)
          sums[i] += pow2(*p++);
    
    // left boundary
    for(i=0; i<patch_size/2; i++) {
      float new_norm = sqrt(sum_array_f(sums+(patch_size/2-i),patch_size/2+i));
      float mul = old_norm / (new_norm + 1e-8);
      float* r = res->pixels + l*etx*ety + i;
      for(j=0; j<ety; j++) 
        r[j*etx] *= mul;
    }
    // right boundary
    for(i=tx+1-patch_size/2; i<etx; i++) {
      float new_norm = sqrt(sum_array_f(sums,patch_size/2+tx-i));
      float mul = old_norm / (new_norm + 1e-8);
      float* r = res->pixels + l*etx*ety + i;
      for(j=0; j<ety; j++) 
        r[j*etx] *= mul;
    }
    
    // because we over-estimated the rectification for the corners, check that they do not overpass old_norm
    float* r = res->pixels + l*etx*ety;
    for(j=0; j<patch_size/2; j++) {
      for(i=0; i<patch_size/2; i++)
        r[j*etx+i] = MIN(r[j*etx+i], old_norm);
      for(i=tx+1-patch_size/2; i<etx; i++)
        r[j*etx+i] = MIN(r[j*etx+i], old_norm);
    }
    for(j=ty+1-patch_size/2; j<ety; j++) {
      for(i=0; i<patch_size/2; i++)
        r[j*etx+i] = MIN(r[j*etx+i], old_norm);
      for(i=tx+1-patch_size/2; i<etx; i++)
        r[j*etx+i] = MIN(r[j*etx+i], old_norm);
    }
  }
}



static inline int retrieve_children( int x, int y, int_cube* child_grid ) {
  const int size0_div2 = child_grid->pixels[0];
  const int step0 = child_grid->tx==1 && child_grid->ty==1 ? 1 : 
                                        MAX( child_grid->pixels[2]-child_grid->pixels[0], 
                                             child_grid->pixels[1+2*child_grid->tx]-child_grid->pixels[1] );
  int i = (x-size0_div2)/step0;
  int j = (y-size0_div2)/step0;
  assert( x==(i*step0+size0_div2) || !"error: child_grid does not match current grid" );
  assert( y==(j*step0+size0_div2) || !"error: child_grid does not match current grid" );
  if( i<0 || i>=child_grid->tx )  return -1;
  if( j<0 || j>=child_grid->ty )  return -1;
  return i+j*child_grid->tx;
}

/* Prepare a grid of cell positions in the first image for a given scale. Big cells inherit the cell at the previous scale.
    size = size of cells at current scale
    offset, step = grid generator: (offset + i*step, offset + j*step)
    child_grid = grid of the previous layer (or None if first layer)
    child_norms = image containing the norms of the patch at the previous level
    grid = result center positions of cells in current scale
    children = index of cells in previous scale used to construct big cells
    norms = norms of the cells of this level
*/
void _prepare_big_cells( int size, int offset, int step, 
                         int_cube* child_grid, float_image* child_norms,
                         int_cube* grid, int_cube* children, float_image* norms ) {
  assert(grid->tz==2);
  const int ntx = grid->tx; // should be == 1+(tx-size)/step so that patches do not pass the border
  const int nty = grid->ty; // should be == 1+(ty-size)/step so that patches do not pass the border
  
  /* grid[i,j] = ( offset + i*step, offset + j*step )
    
    connection between two scales:
    x cell position in lower scale == x position of children in upper scale
    child_offset + child_i*child_step = offset + i*step + (2*u/(nc-1)-1)*size/4
  */
  
  int i,j,u,v;
  int* r = grid->pixels;
  
  if( !child_grid ) {
    // this is the first scale: 
    // we just return a grid of step size*(1-overlap/2) in [0, tx[ x [0, ty[
    
    for(j=0; j<nty; j++)
      for(i=0; i<ntx; i++) {
        *r++ = offset + i*step;
        *r++ = offset + j*step;
      }
  } else {
    assert(child_grid->tz==2);
    ASSERT_SAME_SIZE( child_grid, child_norms );
    assert( children );
    const int nc = sqrt(children->tz); // number of children per row or col
    assert( children->tz==pow2(nc) );
    ASSERT_SAME_SIZE( grid, children );
    ASSERT_SAME_SIZE( grid, norms );
    // this is at least second scale
    // we return a grid of step size*(1-overlap/2) in [0, tx[ x [0, ty[
    
    const int quarter = size/4;
    assert(4*quarter==size);
    int* c = children->pixels; 
    float *n = norms->pixels;
    memset(n,0,ntx*nty*sizeof(float));
    for(j=0; j<nty; j++)
      for(i=0; i<ntx; i++) {
        int x = offset + i*step;
        int y = offset + j*step;
        *r++ = x;
        *r++ = y;
        
        // accumulate norms from 2x2 or 3x3 neighbors        
        for(v=0; v<nc; v++)
          for(u=0; u<nc; u++,c++) {
            // we want to index the children at position:
            // ( center_x + (2*u/(nc-1)-1)*size/4, center_y + (2*v/(nc-1)-1)*size/4 )
            *c = retrieve_children( x+(2*u/(nc-1)-1)*quarter, y+(2*v/(nc-1)-1)*quarter, child_grid );
            if(*c>=0) *n += child_norms->pixels[*c];
          }
        n++;
      }
  }
}


#include <map>

typedef struct {
  int data[9];
} sig;

struct cmp_sig {
  bool operator () ( const sig& a, const sig& b ) {
    return memcmp( a.data, b.data, sizeof(a.data) )<0;
  }
};

static inline sig get_sig_1( int i, const int* ch, const int* cass ) {
  sig res;
  memset(res.data,0xFF,sizeof(res.data));
  res.data[i] = cass[ch[i]];
  return res;
}

static inline sig get_sig_nc( int i, const int* ch, const int* cass, int nc ) {
  sig res;
  memset(res.data,0xFF,sizeof(res.data));
  int j;
  for(j=0; j<nc; j++)
    res.data[i+j*nc] = cass[ch[i+j*nc]];
  return res;
}

static inline sig get_sig_nc2( const int* ch, const int* cass, int nc2 ) {
  sig res;
  memset(res.data,0xFF,sizeof(res.data));
  int j;
  for(j=0; j<nc2; j++)
    res.data[j] = cass[ch[j]];
  return res;
}







/* Compute: arr **= p
*/
void fastipow( float_layers* arr, const float p, int n_thread ) {
  const int n_layers = arr->tz;
  const int npix = arr->tx*arr->ty;
  int l;
  
  // optimization: precompute some values of pow(x,p)
  const int npc = 64;
  float precom[npc+1];
  for(l=0; l<=npc; l++) precom[l]= pow(l/(float)npc,p);
  const float maxindex = npc - 0.001;
  
  #if defined(USE_OPENMP)
  #pragma omp parallel for num_threads(n_thread)
  #endif
  for(l=0; l<n_layers; l++) {
    float* a = arr->pixels + l*npix;
    int i;
    for(i=0; i<npix; i++) {
//      arr[i] = pow(arr[i],p);
      float v = a[i]*npc;
      assert( v>=0 && v<npc+1 );
      if(v>maxindex)  v=maxindex;
      int n = int(v);
      float w = v-n;
      a[i] = (1-w)*precom[n] + w*precom[n+1];
    }
  }
}

/* Compute: arr = max(0,(arr-p)/(1-p))
*/
void fasthinge( float_layers* arr, const float p, int n_thread ) {
  const int n_layers = arr->tz;
  const int npix = arr->tx*arr->ty;
  int l;
  const float f = 1/(1-p);
  
  #if defined(USE_OPENMP)
  #pragma omp parallel for num_threads(n_thread)
  #endif
  for(l=0; l<n_layers; l++) {
    float* a = arr->pixels + l*npix;
    int i;
    for(i=0; i<npix; i++) {
      float v = a[i];
      a[i] = MAX(0,f*(v-p));
    }
  }
}

inline int max_array_i(const int* a, int n)  {
  int i=n;
  int res = INT_MIN;
  while(i--)  if(a[i]>res)  res=a[i];
  return res;
}

/* Compute the (sparse) convolutions specified by <children> on <map> and put the result in <res>.
   A standard order is assumed on the children: 
    a response map #p is built from the children[p] at positions 
      [(gap*dx,gap*dy) for dy in dys for dx in dxs]
      where dxs = [-1,1] or [-1,0,1]
            dys = [-1,1] or [-1,0,1]
   children_assign denote assignement of the children level, while assign is for the next level
   child_norms contain the norms of small patches and norms for big new cells
*/
int _sparse_conv( int_image* children, int_array* children_assign, int gap, float trans_inv,
                   float_layers* map, float_array* child_norms, float_array* norms, 
                   int_array* assign, float_layers* res, int n_thread ) {
  const int nconv = children->ty; // number of convolutions to perform
  const int nc2 = children->tx;
  const int nc = sqrt(nc2);
  assert( nc*nc == nc2 );
  assert( res->tz == nconv );
  const int tx = map->tx;
  const int ty = map->ty;
  const int npix = tx*ty;
  ASSERT_SAME_SIZE( map, res );
  const int n_lower_conv = max_array_i(children->pixels,nconv*nc2)+1;
  int* cass = children_assign ? children_assign->pixels : NEWA(int,n_lower_conv);
  if(!children_assign) {for(int i=0; i<n_lower_conv; i++) cass[i]=i;}
  
  if(assign) {
    assert(0); // not supposed to happen
  } else {
    // normal case: no redundancy to exploit in response maps
    
    int l;
    #if defined(USE_OPENMP)
    #pragma omp parallel for num_threads(n_thread)
    #endif
    for(l=0; l<nconv; l++) {
      float *rmap = res->pixels + l*npix;
      if(norms->pixels[l]==0) continue; // this cell is outside of the image
      
      int u,v,ncall=0;  // children number
      int* child = children->pixels + l*nc2;
      
      float sum_divf[9];
      memset(sum_divf,0,sizeof(sum_divf));
      int i,j;
      
      for(v=0; v<nc; v++) {
        int dy = (2*v/(nc-1)-1);
        for(u=0; u<nc; u++,child++) {
          int dx = (2*u/(nc-1)-1);
          
          if(*child<0 || child_norms->pixels[*child]==0) continue;
          float divf = child_norms->pixels[*child]/norms->pixels[l];
          for(i=-1; i<=1; i++)for(j=-1; j<=1; j++)
            if(i*dx<=0 && j*dy<=0)
              sum_divf[4+j*3+i] += divf;  // count the sum of weights in every image area
          
          // add a translated version of map[children[c]] by (-dx,-dy)
          if(ncall++==0)  // first call
            fast_set_trans( rmap, map->pixels + cass[*child]*npix, divf, dx*gap,dy*gap, tx,ty, 0, 0 );
          else
            fast_add_trans( rmap, map->pixels + cass[*child]*npix, divf, dx*gap,dy*gap, tx,ty, 0, 0 );
        }
      }
      
      // now we are supposed to rectify the boundaries (to perfect convolution)
      if( trans_inv == 0 ) continue;
      for(i=0; i<9; i++) {
        if( sum_divf[i]>0 )
          sum_divf[i] = 1/pow(sum_divf[i], trans_inv);  // if trans_inv==1, no effect
      }
      for(j=0; j<gap; j++) {
        if(sum_divf[0])
          for(i=0; i<gap; i++)
            rmap[j*tx+i] *= sum_divf[0];
        if(sum_divf[1])
          for(i=gap; i<tx-gap; i++)
            rmap[j*tx+i] *= sum_divf[1];
        if(sum_divf[2])
          for(i=tx-gap; i<tx; i++)
            rmap[j*tx+i] *= sum_divf[2];
      }
      for(; j<ty-gap; j++) {
        if(sum_divf[3])
          for(i=0; i<gap; i++)
            rmap[j*tx+i] *= sum_divf[3];
        if(sum_divf[5])
          for(i=tx-gap; i<tx; i++)
            rmap[j*tx+i] *= sum_divf[5];
      }
      for(; j<ty; j++) {
        if(sum_divf[6])
          for(i=0; i<gap; i++)
            rmap[j*tx+i] *= sum_divf[6];
        if(sum_divf[7])
          for(i=gap; i<tx-gap; i++)
            rmap[j*tx+i] *= sum_divf[7];
        if(sum_divf[8])
          for(i=tx-gap; i<tx; i++)
            rmap[j*tx+i] *= sum_divf[8];
      }
    }
  }
  if(!children_assign) free(cass);
  
  return nconv;
}



/* Compute: arr = exp(-arr)
*/
void fastnegexp( float_image* arr ) {
  const int npix = arr->tx*arr->ty;
  float* a = arr->pixels;
  int l;
  
  // optimization: precompute some values of exp(-x)
  // x=linspace(0,8,64); plot(x,exp(-x))
  const int npc = 64;
  float precom[npc+1];
  for(l=0; l<=npc; l++) precom[l]= exp(-l/8.f);
  const float maxindex = npc - 0.001;
  
  int i;
  for(i=0; i<npix; i++) {
    // doing arr[i] = exp(-arr[i])
    float v = 8*a[i];
    if(v>maxindex)  v=maxindex;
    int n = int(v);
    float w = v-n;
    a[i] = (1-w)*precom[n] + w*precom[n+1];
  }
}



/* incorporate the color difference between patches into existing patch similarity
    
    formula: new_response = ( color_sim*addw + old_response*(1-addw) ) * ( mulw*color_sim + 1-mulw )
    
    if mulw=1, adddw=0, then:    new_response = old_response * color_sim
    if mulw=0, adddw=0.5,then:   new_response = (old_response + color_sim )/2
*/
void incorporate_color( int_cube* grid, int_array* assign, float_layers* lab0, float_layers* var0, 
                        float_layers* lab1, float_layers* var1, 
                        float_layers* res_maps, float L_std, float ab_std, int sym_dist, int n_opening, 
                        const float addw, const float mulw, int n_thread ) {
  assert( grid && res_maps && grid->tz==2 );
  const int ncells = grid->tx*grid->ty;
  int* ass = assign ? assign->pixels : NEWA(int,ncells);
  if(!assign) {for(int i=0; i<ncells; i++) ass[i]=i;}
  else  assert( grid->tx*grid->ty == assign->tx );
  const int n_layers = res_maps->tz;
  const int npix = res_maps->tx * res_maps->ty;
  ASSERT_SAME_SIZE( lab0, var0 );
  const int npix0 = lab0->tx * lab0->ty;
  assert( lab0->tz==3 && var0->tz==3 );
  ASSERT_SAME_SIZE( lab1, var1 );
  assert( lab1->tz==3 && var1->tz==3 );
  ASSERT_SAME_SIZE( res_maps, lab1 );
  assert( 0<=addw && addw<=1 );
  assert( 0<=mulw && mulw<=1 );
  
  const int* grid_pix = grid->pixels;
  
  // pre-allocate memory
  float** diffs = NEWA(float*, n_thread);
  int l;
  for(l=0; l<n_thread; l++)
    diffs[l] = NEWA(float, npix);
  
  #if defined(USE_OPENMP)
  omp_set_nested(0);
  omp_set_dynamic(0);
  #pragma omp parallel for num_threads(n_thread)
  #endif
  for(l=0; l<n_layers; l++) {
    if(ass[l]<0)  continue; // unallocated cell
    
    // load patch center, mean color and variance
    int x = grid_pix[2*l+0];
    int y = grid_pix[2*l+1];
    float* patch_colors = lab0->pixels + y*lab0->tx + x;
    float* patch_vars   = var0->pixels + y*var0->tx + x;
    
    #if defined(USE_OPENMP)
    const int numT = omp_get_thread_num();
    #else
    const int numT = 0;
    #endif
    float* diff = diffs[numT];
    int i;
    
    // take care of luminance
    float* lab1_pix = lab1->pixels;
    {const float L = patch_colors[0*npix0];
    const float L_var = L_std*L_std; 
    for(i=0; i<npix; i++) {
      float d = lab1_pix[i] - L;
      diff[i] = d*d/L_var;
      //assert( diff[i]>=0 );
    }}
    
    if( !sym_dist ) {
      // non-symmetric distance: we just use the gaussian Lab model of reference image
      {const float a = patch_colors[1*npix0];
      const float a_var = ab_std*ab_std * patch_vars[1*npix0];
      lab1_pix += npix;
      for(i=0; i<npix; i++) {
        float d = lab1_pix[i] - a;
        diff[i] += d*d/a_var;
        //assert( diff[i]>=0 );
      }}
      {const float b = patch_colors[2*npix0];
      const float b_var = ab_std*ab_std * patch_vars[2*npix0];
      lab1_pix += npix;
      for(i=0; i<npix; i++) {
        float d = lab1_pix[i] - b;
        diff[i] += d*d/b_var;
        //assert( diff[i]>=0 );
      }}
    } else {
      // symmetric distance: we use the gaussian Lab models of both images
      const float ab_var = ab_std*ab_std/2;
      {const float a = patch_colors[1*npix0];
      const float a_var = patch_vars[1*npix0];
      const float* a_var1 = var1->pixels + npix;
      lab1_pix += npix;
      for(i=0; i<npix; i++) {
        float d = lab1_pix[i] - a;
        diff[i] += d*d/(ab_var * (a_var+a_var1[i]));
      }}
      {const float b = patch_colors[2*npix0];
      const float b_var = patch_vars[2*npix0];
      const float* b_var1 = var1->pixels + 2*npix;
      lab1_pix += npix;
      for(i=0; i<npix; i++) {
        float d = lab1_pix[i] - b;
        diff[i] += d*d/(ab_var * (b_var+b_var1[i]));
      }}
    }
    
    float_image diff_img = {diff,lab1->tx,lab1->ty};
    
    // compute col_map = exp(-diff.sum(0))
    fastnegexp( &diff_img );
    
    // enlarge the maxima (apply several openings)
    for(i=0; i<n_opening; i++)
      _max_filter_3( &diff_img, &diff_img, n_thread );
    
    // combine with response maps
    assert( 0<=ass[l] && ass[l]<ncells );
    float* res_map = res_maps->pixels + ass[l]*npix;
    ass[l] += ncells; // this way, we ensure that we won't redo one layer
    const float comp_addw = 1-addw;
    const float comp_mulw = 1-mulw;
    for(i=0; i<npix; i++) {
      float r = res_map[i];
      float c = diff[i];
      r = addw*c + comp_addw*r;
      r *= mulw*c + comp_mulw;
      res_map[i] = r;
    }
  }
  for(l=0; l<n_layers; l++)
    if(ass[l]>=ncells)
      ass[l] -= ncells; // rectify as before
  
  // free memory
  for(l=0; l<n_thread; l++)
    free( diffs[l] );
  free( diffs );
  if(!assign) free(ass);
}
























