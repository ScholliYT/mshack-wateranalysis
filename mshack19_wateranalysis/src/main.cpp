#include <Arduino.h>
#include <pinout.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define SAMPLE_SIZE 3
#define MEASUREMENT_DELAY 20
#define SAMPLE_DELAY 500

OneWire oneWire(TemperatureSensorPin);
DallasTemperature sensors(&oneWire);

float phArray[SAMPLE_SIZE];
float ConductivityArray[SAMPLE_SIZE];

float phValue, conductivityValue, temperatureValue;

float getPHValue()
{
  int sensorValue = analogRead(PHSensorPin);
  float voltage = sensorValue * (5.0 / 1024.0);
  float pHValue = 3.5 * voltage + Offset;

  Serial.print("PH Voltage: ");
  Serial.print(voltage);
  Serial.print(" PH value: ");
  Serial.println(pHValue);
  return phValue;
}

float getConductivityValue()
{
  int sensorValue = analogRead(ConductivitySensorPin);
  float voltage = sensorValue * (5.0 / 1024.0);
  float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);                                                                                                                              //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
  float compensationVolatge = voltage / compensationCoefficient;                                                                                                                                  //temperature compensation
  float conductivityValue = (133.42 * compensationVolatge * compensationVolatge * compensationVolatge - 255.86 * compensationVolatge * compensationVolatge + 857.39 * compensationVolatge) * 0.5; //convert voltage value to tds value

  Serial.print("Conductivity Voltage: ");
  Serial.print(voltage);
  Serial.print(" Conductivity value: ");
  Serial.print(conductivityValue);
  Serial.println(" ppm");

  return conductivityValue;
}

float getTemperature()
{
  sensors.requestTemperatures();
  float temperature = sensors.getTempCByIndex(0);

  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" C");

  return temperature;
}

float avgArray(float values[], int size) {
  float sum = 0;
  for(int i = 0; i < size; i++) {
    sum += values[i];
  }
  return (sum/size);
}

void takeMeasurements() {
  for (int i = 0; i < SAMPLE_SIZE; i++) {
    phArray[i] = getPHValue();
    ConductivityArray[i] = getConductivityValue();

    delay(MEASUREMENT_DELAY);
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println("MSHack19 Wateranalysis");
  Serial.print("Taking samples of ");Serial.print(SAMPLE_SIZE);Serial.print(" measurements with a delay of ");
  Serial.print(MEASUREMENT_DELAY);Serial.print(" ms every ");Serial.print(SAMPLE_DELAY);Serial.println(" ms.");

  pinMode(PHSensorPin, INPUT);
  pinMode(ConductivitySensorPin, INPUT);
  pinMode(TemperatureSensorPin, INPUT);

  sensors.begin(); // Start up the library for temperature reading
}

void loop()
{
  takeMeasurements();

  phValue = avgArray(phArray, SAMPLE_SIZE);
  conductivityValue = avgArray(ConductivityArray, SAMPLE_SIZE);
  temperatureValue = getTemperature();

  Serial.println();

  Serial.println("===== SAMPLE =====");
  Serial.print("PH Value: ");Serial.println(phValue);
  Serial.print("ConductivityValue: ");Serial.print(conductivityValue);Serial.println(" ppm");
  Serial.print("TemperatureValue: ");Serial.print(temperatureValue);Serial.println(" C");

  Serial.println();
  Serial.println();

  delay(SAMPLE_DELAY);
}

