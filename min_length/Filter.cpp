#include "Filter.h"
void Filter::begin(float freq_cutoff, float freq_sampling){
  fc = freq_cutoff;
  fs = freq_sampling;
  tau = 1/(2.0*PI*fc);
  alpha = (1.0/fs) / (tau + (1.0/fs));
  prev = {0, 0, 0};
}

vec3_t Filter::step(vec3_t vec){
  prev = prev + alpha*(vec - prev);
  return prev;
}

void Filter::reset(){
  prev = {0, 0, 0};
}

void ZVU::begin(float ta, float tb, int ts){
  thresh_a = ta;
  thresh_b = tb;
  sample_thresh = ts;
}

bool ZVU::check(vec3_t a, vec3_t b){
  if(a.x < thresh_a && a.y < thresh_a && a.z < thresh_a 
//    && b.x < thresh_b && b.y < thresh_b && b.z < thresh_b
    ){
    samples++;
  } else {
    samples = 0;
  }
  return samples > sample_thresh;
}
