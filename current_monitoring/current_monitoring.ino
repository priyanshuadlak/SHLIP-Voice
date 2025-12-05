#define SENSOR_PIN 34
#define ADC_MAX 4095.0
#define VREF 3.3
#define SENSITIVITY 0.066

float ZERO_OFFSET = 1.65;  // default, will be corrected in setup

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  // AUTO CALIBRATION - read average of 1000 samples
  long sum = 0;
  for (int i = 0; i < 1000; i++) {
    int adc = analogRead(SENSOR_PIN);
    sum += adc;
    delayMicroseconds(500);
  }

  float avgAdc = sum / 1000.0;
  ZERO_OFFSET = (avgAdc / ADC_MAX) * VREF;

  Serial.print("Calibrated ZERO_OFFSET = ");
  Serial.println(ZERO_OFFSET);
}

void loop() {
  const int samples = 2000;
  double sumSq = 0;

  for (int i = 0; i < samples; i++) {
    int adc = analogRead(SENSOR_PIN);
    float voltage = (adc / ADC_MAX) * VREF;
    float centered = voltage - ZERO_OFFSET;
    sumSq += centered * centered;
  }

  float rmsVoltage = sqrt(sumSq / samples);
  float current = rmsVoltage / SENSITIVITY;

  Serial.print("Current = ");
  Serial.print(current - 0.13);
  Serial.println(" A");

  delay(300);
}
