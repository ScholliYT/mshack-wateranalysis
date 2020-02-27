#include <Arduino.h>
#include <pinout.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <lora.h>

#define SAMPLE_SIZE 3
#define MEASUREMENT_DELAY 20
#define SAMPLE_DELAY 20000

OneWire oneWire(TemperatureSensorPin);
DallasTemperature sensors(&oneWire);

float phValue, conductivityValue, temperatureValue;
static unsigned long lastSampleTime = 0;

float getPHValue()
{
  int sensorValue = analogRead(PHSensorPin);
  float voltage = sensorValue * (5.0 / 1024.0);
  float pH = 3.5 * voltage + Offset;

  Serial.print("PH Voltage: ");
  Serial.print(voltage);
  Serial.print(" PH value: ");
  Serial.println(pH);
  return pH;
}

float getConductivityValue()
{
  int sensorValue = analogRead(ConductivitySensorPin);
  float voltage = sensorValue * (5.0 / 1024.0);
  float compensationCoefficient = 1.0 + 0.02 * (temperatureValue - 25.0);                                                                                                                              //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
  float compensationVolatge = voltage / compensationCoefficient;                                                                                                                                  //temperature compensation
  float conductivity = (133.42 * compensationVolatge * compensationVolatge * compensationVolatge - 255.86 * compensationVolatge * compensationVolatge + 857.39 * compensationVolatge) * 0.5; //convert voltage value to tds value

  Serial.print("Conductivity Voltage: ");
  Serial.print(voltage);
  Serial.print(" Conductivity value: ");
  Serial.print(conductivity);
  Serial.println(" ppm");

  return conductivity;
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

float avgArray(float values[], int size)
{
  float sum = 0;
  for (int i = 0; i < size; i++)
  {
    sum += values[i];
  }
  return (sum / (float)size);
}

void takeMeasurements() { 
  temperatureValue = getTemperature(); 
 
  float sumph = 0; 
  float sumConductivity = 0; 
  for (int i = 0; i < SAMPLE_SIZE; i++) { 
    sumph += getPHValue(); 
    sumConductivity += getConductivityValue(); 
 
    delay(MEASUREMENT_DELAY); 
  } 
 
  phValue = sumph / (float)SAMPLE_SIZE; 
  conductivityValue = sumConductivity / (float)SAMPLE_SIZE; 
} 

void setup()
{
  Serial.begin(115200);
  Serial.println("MSHack19 Wateranalysis");
  Serial.print("Taking samples of ");
  Serial.print(SAMPLE_SIZE);
  Serial.print(" measurements with a delay of ");
  Serial.print(MEASUREMENT_DELAY);
  Serial.print(" ms every ");
  Serial.print(SAMPLE_DELAY);
  Serial.println(" ms.");

  pinMode(PHSensorPin, INPUT);
  pinMode(ConductivitySensorPin, INPUT);
  pinMode(TemperatureSensorPin, INPUT);

  sensors.begin(); // Start up the library for temperature reading

  setup_lora(SAMPLE_DELAY);
}

void loop()
{
  if (millis() - lastSampleTime > SAMPLE_DELAY)
  {
    takeMeasurements();

    Serial.println();

    Serial.println("===== Sending this Measurment =====");
    Serial.print("PH Value: ");
    Serial.println(phValue);
    Serial.print("ConductivityValue: ");
    Serial.print(conductivityValue);
    Serial.println(" ppm");
    Serial.print("TemperatureValue: ");
    Serial.print(temperatureValue);
    Serial.println(" C");

    Serial.println();
    Serial.println();

    queue_package(phValue, conductivityValue, temperatureValue);

    lastSampleTime = millis();
  }

  loop_lora();
}
