#include <Arduino.h>
#include <pinout.h>
#include <OneWire.h>
#include <DallasTemperature.h>

OneWire oneWire(TemperatureSensorPin);
DallasTemperature sensors(&oneWire);

void readTurbidity()
{
  int sensorValue = analogRead(TurbiditySensorPin);                               // read the input on analog pin 0:
  float voltage = sensorValue * (5.0 / 1024.0);                                   // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  float turbidityValue = -1120.4 * voltage * voltage + 5742.3 * voltage - 4352.9; // -1120.4*x^2 + 5742.3*x - 4352.9

  Serial.print("Turbidity voltage: ");
  Serial.print(voltage);
  Serial.print(" Turbidity value: ");
  Serial.print(turbidityValue);
  Serial.println(" NTU");
}

void readPH()
{
  int sensorValue = analogRead(PHSensorPin);
  float voltage = sensorValue * (5.0 / 1024.0);
  float pHValue = 3.5 * voltage + Offset;

  Serial.print("PH Voltage: ");
  Serial.print(voltage);
  Serial.print(" PH value: ");
  Serial.println(pHValue);
}

void readConductivity()
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
}

void readTemperature()
{
  sensors.requestTemperatures();
  float temperature = sensors.getTempCByIndex(0);

  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" C");
}

void setup()
{
  Serial.begin(115200);
  Serial.println("MSHack19 Wateranalysis");

  pinMode(PHSensorPin, INPUT);
  pinMode(TurbiditySensorPin, INPUT);
  pinMode(ConductivitySensorPin, INPUT);
  pinMode(TemperatureSensorPin, INPUT);

  sensors.begin(); // Start up the library for temperature reading
}

void loop()
{
  readTurbidity();
  readPH();
  readConductivity();
  readTemperature();

  Serial.println();
  Serial.println();

  delay(2000);
}