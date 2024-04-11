#include "Integral.h"

void Integral::begin(vec3_t vec){
  reset(vec);
}

vec3_t Integral::step(vec3_t v, float dt){
  cum += (v + prev) * 0.5 * dt;
  prev = v;
  return cum;
}

void Integral::reset(vec3_t v){
  resetPrev(v);
  cum = {0,0,0};
}

void Integral::resetPrev(vec3_t v){
  prev = v;
}

float DeltaTime::step(uint32_t ts){
  float dt = (float)(ts - prev_ts) * 0.000001f; // Convert us to seconds
  set(ts);
  return dt;
}
void DeltaTime::set(uint32_t ts){
  prev_ts = ts;
}

float DeltaTime::cumDiff(uint32_t ts){
  return (float)(ts - first) * MICRO_TO_SEC;
}
