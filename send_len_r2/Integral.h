#include <vector_type.h>
#define MICRO_TO_SEC 0.000001f
class Integral{
  public:
    vec3_t cum;
    vec3_t prev;
    float timescale;
    void begin(vec3_t);
    vec3_t step(vec3_t, float);
    void reset(vec3_t);
    void resetPrev(vec3_t);
};

class DeltaTime {
  public:
    uint32_t prev_ts;
    uint32_t first;
    float step(uint32_t);
    float cumDiff(uint32_t);
    void set(uint32_t);
};
