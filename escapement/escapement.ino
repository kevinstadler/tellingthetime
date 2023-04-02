#include <Wire.h>
#include <VL53L1X.h>

VL53L1X sensor;

#include <MD_MAXPanel.h>
#include <SPI.h>

MD_MAXPanel display = MD_MAXPanel(MD_MAX72XX::FC16_HW, 10, 4, 1);

#define BUFFERSIZE 2
uint32_t buffer[BUFFERSIZE];
uint8_t index;

void setup() {
  while (!Serial) {}
  Serial.begin(115200);
  Wire.begin();
//  Wire.setClock(400000); // use 400 kHz I2C

  sensor.setTimeout(50);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }

  // Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
  // You can change these settings to adjust the performance of the sensor, but
  // the minimum timing budget is 20 ms for short distance mode and 33 ms for
  // medium and long distance modes. See the VL53L1X datasheet for more
  // information on range and timing limits.
  sensor.setDistanceMode(VL53L1X::Short);
  sensor.setMeasurementTimingBudget(20000);

  // Start continuous readings at a rate (the inter-measurement period). This period should be at least as long as the
  // timing budget.
  sensor.startContinuous(20);

  display.begin();
  display.update(true);
}

void loop() {
  uint32_t t = millis();
  sensor.read();
  buffer[index] = sensor.ranging_data.range_mm;
  uint32_t s = 0;
  for (uint8_t i = 0; i < BUFFERSIZE; i++) {
    s += buffer[i];
  }
  Serial.print(millis() - t);
  Serial.print("\t");
  Serial.print(buffer[index]);
  Serial.print("\t");
  Serial.println(s / BUFFERSIZE);
  index = (index+1) % BUFFERSIZE;
  return;

 // dist = min(63, 10000 / dist);
  Serial.print("\tstatus: ");
  Serial.print(VL53L1X::rangeStatusToString(sensor.ranging_data.range_status));
  Serial.print("\tpeak signal: ");
  Serial.print(sensor.ranging_data.peak_signal_count_rate_MCPS);
  Serial.print("\tambient: ");
  Serial.print(sensor.ranging_data.ambient_count_rate_MCPS);

  Serial.println();
//  display.clear();
//  display.drawFillRectangle(0, 0, dist, 7);
}