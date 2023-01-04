#include <Adafruit_MPU6050.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include "arduinoFFT.h"
#include <OLED_SSD1306_Chart.h>
#include <Adafruit_I2CDevice.h>

#define SDA_PIN 4 // D1
#define SCL_PIN 5 // D2

// Define push button input
#define INPUT_BUTTON_PIN 14 // D5

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET -1    // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

Adafruit_MPU6050 _mpu;
Adafruit_SSD1306 _display = Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);

/*
These values can be changed in order to evaluate the functions
*/
const uint16_t _samplesCount = 512;      // This value MUST ALWAYS be a power of 2
const float _samplingFrequency = 300.0f; // Hz, must be less than 10000 due to ADC

unsigned int sampling_period_us;
unsigned long microseconds;

/*
These are the input and output vectors
Input vectors receive computed results from fft
*/
float _samples[_samplesCount];
float _vectorReal[_samplesCount];
float _vectorImag[_samplesCount];

bool startSampling = true;
bool skipView = false;

std::vector<std::pair<float, int>> topPeakFrequencies;

// Checks if motion was detected, sets LED HIGH and starts a timer
ICACHE_RAM_ATTR void startSamplingInterruption() {
  Serial.println("Interruption");
  
  skipView = true;

  auto interruptionTime = millis();
  while(!startSampling) {
    if (digitalRead(INPUT_BUTTON_PIN) == HIGH) {
      break;
    }

    if (millis() - interruptionTime >= 2000) {
      // switch was LOW throughout the period
      startSampling = true;
      Serial.println("Start sampling");
    } 
  }
}

void setup()
{
// put your setup code here, to run once:
#if defined ESP8266
  Wire.begin(SDA_PIN, SCL_PIN);
#else
  Wire.begin();
#endif

  Serial.begin(115200);
  while (!Serial);
  Serial.println("Init");

  // PIR input button mode INPUT_PULLUP
  pinMode(INPUT_BUTTON_PIN, INPUT_PULLUP);

  // Set INPUT_BUTTON_PIN pin as interrupt, assign interrupt function and set FALLING mode
  attachInterrupt(digitalPinToInterrupt(INPUT_BUTTON_PIN), startSamplingInterruption, FALLING);


  // Serial.println("Found a MPU-6050 sensor");

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!_display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  { // Address 0x3C for 128x32
    // Serial.println(F("SSD1306 allocation failed"));
    Serial.println("!s");
    for (;;)
      ; // Don't proceed, loop forever
  }

  if (!_mpu.begin())
  {
    Serial.println("!m");
    _display.setTextSize(1);
    _display.setTextColor(WHITE);
    _display.setRotation(0);
    _display.clearDisplay();
    _display.setCursor(0, 0);
    _display.println("Accelerometer failure");
    _display.display();

    while (1)
      yield();
  }

  _display.display();
  delay(2000); // Pause for 2 seconds
  _display.setTextSize(1);
  _display.setTextColor(WHITE);
  _display.setRotation(0);

  sampling_period_us = round(1000000 * (1.0 / _samplingFrequency));
  Serial.println("Inited");

  _display.clearDisplay();
  _display.setCursor(0, 0);

  _display.println("Mechanical vibrations\n");

  _display.println("Elaborated by:");
  _display.println("Martha Linares\nBaldomino");

  _display.println("Assessed by:");
  _display.println("Jose Alberto Padilla Madrid");

  _display.display();
  delay(3000); // Pause for 3 seconds
}

void loop()
{
  AdquireSamples();

  ComputePeakFrequencies();

  do
  {
    PrintTop3PeakFrequencies();

    PlotSamplesGraph();

    PlotFrequenciesVectorGraph();
  } while (!startSampling);

  PrintVector(_vectorReal, (_samplesCount >> 1), SCL_FREQUENCY);
  Serial.println(topPeakFrequencies[0].first, 6); // Print out what frequency is the most dominant.

  Serial.println(topPeakFrequencies.size()); // Print out what frequency is the most dominant.
  for (uint16_t i = 0; i < min(topPeakFrequencies.size(), 3U); i++)
  {
    Serial.print(topPeakFrequencies[i].second);     // Print out what frequency is the most dominant.
    Serial.print("= ");                             // Print out what frequency is the most dominant.
    Serial.println(topPeakFrequencies[i].first, 6); // Print out what frequency is the most dominant.
  }
}

float ReadAcceleration()
{
  sensors_event_t a, g, temp;
  _mpu.getEvent(&a, &g, &temp);

  float x2 = sq(a.acceleration.x);
  float y2 = sq(a.acceleration.y);
  float z2 = sq(a.acceleration.z);

  return sqrt(x2 + y2 + z2);
}

void AdquireSamples()
{
  // Sampling
  Serial.println("Smp");

  _display.clearDisplay();
  _display.setCursor(0, 0);

  _display.println("Sampling...");
  _display.display();

  delay(2000);
  microseconds = micros();
  for (int i = 0; i < _samplesCount; i++)
  {
    float acceleration = ReadAcceleration();
    _samples[i] = acceleration;
    _vectorReal[i] = acceleration;
    _vectorImag[i] = 0;
    while (micros() - microseconds < sampling_period_us)
    {
      yield();
    }
    microseconds += sampling_period_us;
  }

  startSampling = false;
  yield();
}

void ComputePeakFrequencies()
{
  // Create fft object
  Serial.println("Create fft");
  _display.println("Create fft...");
  _display.display();

  arduinoFFT fft = arduinoFFT(_vectorReal, _vectorImag, _samplesCount, _samplingFrequency);

  // Weigh data
  yield();
  Serial.println("Weigh data");
  _display.println("Weigh data...");
  _display.display();
  fft.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);

  // Compute fft
  yield();
  Serial.println("Compute fft");
  _display.println("Compute fft...");
  _display.display();
  fft.Compute(FFT_FORWARD);

  // Compute magnitudes
  yield();
  Serial.println("Compute magnitudes");
  _display.println("Compute magnitudes...");
  _display.display();
  fft.ComplexToMagnitude();
  topPeakFrequencies = fft.TopPeakFrequencies();

  yield();
}

void PlotSamplesGraph()
{
  const int &minY = *std::min_element(_samples, _samples + _samplesCount);
  const int &maxY = *std::max_element(_samples, _samples + _samplesCount);

  int minX = 60;
  int increment = 5;
  int width = 123;
  int height = 55;

  auto display = OLED_SSD1306_Chart(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3c);

  display.clearDisplay();
  display.setChartCoordinates(0, minX);          // Chart lower left coordinates (X, Y)
  display.setChartWidthAndHeight(width, height); // Chart width = 123 and height = 60
  display.setXIncrement(increment);              // Distance between Y points will be 5px
  display.setYLimits(minY, maxY);                // Ymin = 0 and Ymax = 100
  display.setYLimitLabels("0", "100");           // Setting Y axis labels
  display.setYLabelsVisible(true);
  display.setAxisDivisionsInc(12, 6);    // Each 12 px a division will be painted in X axis and each 6px in Y axis
  display.setPlotMode(SINGLE_PLOT_MODE); // Set single plot mode

  display.setLineThickness(LIGHT_LINE);
  display.drawChart(); // Update the buffer to draw the cartesian chart
  display.display();

  auto result = false;

  for (uint16_t i = 0; i < _samplesCount; i++)
  {
    if (startSampling || skipView) {
      break;
    }

    auto value = _samples[i];
    result = display.updateChart(value);

    delay(1);

    if (!result)
    {
      display.clearDisplay(); // If chart is full, it is drawn again
      display.drawChart();
      // break;
    }
  }

  if (!skipView) {
    Wait(3000);
  }

  skipView = false;
}

void PlotFrequenciesVectorGraph()
{
  auto skipElements = 3;
  const int &minY = *std::min_element(_vectorReal + skipElements, _vectorReal + _samplesCount - skipElements);
  const int &maxY = *std::max_element(_vectorReal + skipElements, _vectorReal + _samplesCount - skipElements);

  int minX = 60;
  int increment = 1;
  int width = 123;
  int height = 55;

  auto display = OLED_SSD1306_Chart(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3c);

  display.clearDisplay();
  display.setChartCoordinates(0, minX);          // Chart lower left coordinates (X, Y)
  display.setChartWidthAndHeight(width, height); // Chart width = 123 and height = 60
  display.setXIncrement(increment);              // Distance between Y points will be 5px
  display.setYLimits(minY, maxY);                // Ymin = 0 and Ymax = 100
  display.setYLimitLabels("0", "100");           // Setting Y axis labels
  display.setYLabelsVisible(true);
  display.setAxisDivisionsInc(12, 6);    // Each 12 px a division will be painted in X axis and each 6px in Y axis
  display.setPlotMode(SINGLE_PLOT_MODE); // Set single plot mode

  display.setLineThickness(LIGHT_LINE);
  display.drawChart(); // Update the buffer to draw the cartesian chart
  display.display();

  auto result = false;

  for (uint16_t i = skipElements; i < _samplesCount - skipElements; i++)
  {
    if (startSampling || skipView) {
      break;
    }

    auto value = _vectorReal[i];
    result = display.updateChart(value);
    delay(1);

    if (!result)
    {
      display.clearDisplay(); // If chart is full, it is drawn again
      display.drawChart();
    }
  }

    if (!skipView) {
      Wait(3000);
  }

  skipView = false;
}

void PrintTop3PeakFrequencies()
{
  if (topPeakFrequencies.size() < 1)
  {
    return;
  }

  float topFrequency = topPeakFrequencies[0].first;
  uint16_t size = min(topPeakFrequencies.size(), 3U);

  _display.clearDisplay();
  _display.setCursor(0, 0);

  _display.println("Top peak frequencies:\n");

  for (uint16_t i = 0; i < size; i++)
  {
    if (startSampling || skipView) {
      break;
    }

    float frequency = topPeakFrequencies[i].first;
    float percentage = frequency / topFrequency * 100.0;

    _display.print(percentage, 1);
    _display.print(" - ");
    _display.print(frequency, 1);
    _display.println(" Hz");
  }

  _display.display();

  if (!skipView) {
    Wait(5000);
  }

  skipView = false;
}

void Wait(int delay_time_in_ms)
{
  auto startTime = millis();
  while((millis() - startTime < delay_time_in_ms)
  && !skipView && !startSampling)
  {
    delay (100);
  }

}

void PrintVector(float *vData, uint16_t bufferSize, uint8_t scaleType)
{
  for (uint16_t i = 0; i < bufferSize; i++)
  {
    float abscissa;
    /* Print abscissa value */
    switch (scaleType)
    {
    case SCL_INDEX:
      abscissa = (i * 1.0);
      break;
    case SCL_TIME:
      abscissa = ((i * 1.0) / _samplingFrequency);
      break;
    case SCL_FREQUENCY:
      abscissa = ((i * 1.0 * _samplingFrequency) / _samplesCount);
      break;
    }
    Serial.print(abscissa, 6);
    if (scaleType == SCL_FREQUENCY)
      Serial.print("Hz");
    Serial.print(" ");
    Serial.println(vData[i], 4);
  }
  Serial.println();
}
