
#include <simpletools.h>

long ping_hcsr04(int echopin, int triggerpin)
{
  low(triggerpin);
  pulse_out(triggerpin, 10);
  return pulse_in(echopin, 1);
}

int ping_cm_hcsr04(int echopin, int triggerpin)
{
  long tEcho = ping_hcsr04(echopin, triggerpin);
  int cmDist = tEcho / 58;
  return cmDist;
}