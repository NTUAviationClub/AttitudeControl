#ifndef MIXER__H_
#define MIXER__H_

#include "filter_base.h"

#define MX_SOURCE_NUM_ 8

#define MX_SOURCE_FX_ 1
#define MX_SOURCE_FY_ 2
#define MX_SOURCE_FZ_ 3
#define MX_SOURCE_RR_ 4
#define MX_SOURCE_PR_ 5
#define MX_SOURCE_YR_ 6
#define MX_SOURCE_FL_ 7

DTYPE source_[MX_SOURCE_NUM_];

typedef struct MixerSource {
  char source;
  DTYPE gain;
} MS;

void mx_update_source(int source, DTYPE value);

DTYPE mx_get_value(MS *actuator);

#endif