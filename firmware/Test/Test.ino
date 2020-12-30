#include <JNTUB.h>

using namespace JNTUB;

void setup() {
  Serial.begin(9600);
}

void loop() {
  uint16_t param1 = Io::readParam1();
  uint16_t param2 = Io::readParam2();
  uint16_t param3 = Io::readParam3();

  bool gate = Io::readGate();

  uint16_t output_val;
  if (gate) {
    // Output the min of the three inputs.
    output_val = param1;
    if (param2 < output_val)
      output_val = param2;
    if (param3 < output_val)
      output_val = param3;
  } else {
    // Output the max of the three inputs
    output_val = param1;
    if (param2 > output_val)
      output_val = param2;
    if (param3 > output_val)
      output_val = param3;
  }

  Io::analogWriteOut(map(output_val, 0, 1023, 0, 255));

  char buf[256];
  int len = sprintf(buf, "Param1: %i, Param2: %i, Param3: %i, Gate: %i\n", param1, param2, param3, gate);
  Serial.write(buf, len);
  delay(50);
}
