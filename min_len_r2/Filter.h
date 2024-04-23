#include <vector_type.h>
class Filter {
  public:
    float fc;
    float fs;
    float alpha;
    float tau;
    vec3_t prev;
    void begin(float, float);
    vec3_t step(vec3_t);
    void reset();
};

class ZVU {
  public:
    int samples;
    int sample_thresh;
    float thresh_a;
    float thresh_b;
    void begin(float, float, int);
    bool check(vec3_t, vec3_t);
};
