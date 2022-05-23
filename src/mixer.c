#include "mixer.h"

void mx_update_source(int source, DTYPE value) { source_[source] = value; }

DTYPE mx_get_value(MS *actuator) {
  DTYPE res = 0;
  char sn = (char)sizeof(actuator) / sizeof(actuator[0]);
  for (int i = 0; i < sn; i++) {
    res += actuator[i].gain * source_[actuator[i].source];
  }
  return res;
}