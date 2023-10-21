#include <ArduinoSTL.h>
#include <arduino-timer.h>
#include <array>

#define SAMPLE_RATE 50   // ms
#define RESULT_RATE 1000 // ms
#define BUFF_SIZE (RESULT_RATE / SAMPLE_RATE)

#define TDS_PIN A0
#define VOLT_REF 5.0           // volts
#define VOLT_CONST 9.765625e-4 // 1/1024

#define TEMP_PIN A1
#define TEMP_R1 1000 // ohm
#define TEMP_C1 1.009249522e-03
#define TEMP_C2 2.378405444e-04
#define TEMP_C3 2.019202697e-07

static Timer<2, millis> timer;
static std::vector<int> voltBuffer(BUFF_SIZE);
static std::vector<int> tempBuffer(BUFF_SIZE);

void setup() {
  Serial.begin(115200);

  timer.every(SAMPLE_RATE, sample);
  timer.every(RESULT_RATE, print_results);
}

void loop() {
  timer.tick();
}

bool sample(void*) {
  int voltSample, tempSample;

  voltSample = analogRead(TDS_PIN);
  voltBuffer.push_back(voltSample);

  tempSample = analogRead(TEMP_PIN);
  tempBuffer.push_back(tempSample);

  #ifdef DEBUG
  printf("Sample values: %d %d\n", voltSample, tempSample);
  #endif
}

bool print_results(void*) {
  double medianTemp, logR2, temperature;
  double medianVolt, coeff, EC, EC25, ppm;
  char buff[4][10];
  unsigned long currentTick;

  // ========= Temperatur Calculation ==========
  medianTemp = getMedian(tempBuffer);
  logR2 = log(TEMP_R1 * ((1023.0 / medianTemp) - 1.0));
  temperature = 1.0 / (TEMP_C1 + TEMP_C2 * logR2 + TEMP_C3 * logR2 * logR2 * logR2);
  temperature -= 273.15; // final value in celcius
  // ======= End Temperatur Calculation ========

  // ============= TDS Calculation =============
  medianVolt = getMedian(voltBuffer);
  EC = medianVolt * VOLT_REF * VOLT_CONST;

  // Temperature Compensation of EC:
  // EC  =  [ 1 + 0.02 (t – 25) ] * EC25
  coeff = 1.0 + 0.02 * (temperature - 25.0);
  EC25 = EC / coeff;

  // not sure what this is, looks like a sensor calibrated formula
  // or some kind of polynomial interpolation for a more complicated math function...
  ppm = (133.42 * EC25 * EC25 * EC25 - 255.86 * EC25 * EC25 + 857.39 * EC25) * 0.5;
  // =========== End TDS Calculation ===========

  // double to string conversion
  dtostrf(temperature, 6, 2, buff[0]);
  dtostrf(EC, 6, 2, buff[1]);
  dtostrf(EC25, 6, 2, buff[2]);
  dtostrf(ppm, 6, 2, buff[3]);

  currentTick = millis() / 1000UL;
  printf("Tick: %4lu, Temp: %s °C, EC: %s, EC25: %s, ppm: %s\n", currentTick, buff[0], buff[1], buff[2], buff[3]);

  // clear buffers for next sample
  voltBuffer.clear();
  voltBuffer.reserve(BUFF_SIZE);

  tempBuffer.clear();
  tempBuffer.reserve(BUFF_SIZE);
}

// algorithm for noise filtering, it implements a median filtering
// another option is to use a  morphological filter, or contraharmonic mean filter
double getMedian(std::vector<int>& data) {
  size_t size = data.size();
  const auto med1 = data.begin() + (size / 2);

  // partial sort data
  std::nth_element(data.begin(), med1, data.end());

  // if odd, median is the middle value
  if ((size & 1) > 0) return 1.0 * (*med1);

  // otherwise it is the average of two middle value
  const auto med2 = data.begin() + (size / 2) - 1;
  std::nth_element(data.begin(), med2, data.end());

  return 0.5 * (*med1 + *med2);
}
