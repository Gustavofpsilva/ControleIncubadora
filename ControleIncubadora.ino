#include <DHT.h>
#include <PID_v1.h>
#include <Adafruit_ADS1015.h>

#define DHT_PIN 2
#define HEATER_PIN 5
#define FAN_PIN 6
#define TDS_PIN A0
#define pH_PIN A1
#define WATER_SPRINKLER_PIN 7

DHT dht(DHT_PIN, DHT22);
Adafruit_ADS1015 ads;

double setpoint_temperature = 37.5; // Temperatura desejada em Celsius
double setpoint_humidity = 60.0;    // Umidade desejada em porcentagem
double setpoint_TDS = 1000.0;       // Valor de TDS desejado em ppm
double setpoint_pH = 7.0;           // Valor de pH desejado
double setpoint_air_humidity = 50.0; // Umidade do ar desejada em porcentagem

double input_temperature, output_temperature;
double input_humidity, output_humidity;
double input_TDS, output_TDS;
double input_pH, output_pH;
double input_air_humidity, output_air_humidity;

double Kp = 5; // Ganho proporcional
double Ki = 0.1; // Ganho integral
double Kd = 1; // Ganho derivativo

double elapsedTime, previousTime;
double error_temperature, lastError_temperature;
double cumError_temperature, rateError_temperature;

double error_humidity, lastError_humidity;
double cumError_humidity, rateError_humidity;

double error_TDS, lastError_TDS;
double cumError_TDS, rateError_TDS;

double error_pH, lastError_pH;
double cumError_pH, rateError_pH;

double error_air_humidity, lastError_air_humidity;
double cumError_air_humidity, rateError_air_humidity;

double outMin = 0;
double outMax = 255;

void setup() {
  Serial.begin(9600);
  dht.begin();
  ads.begin();

  pinMode(HEATER_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);
  pinMode(WATER_SPRINKLER_PIN, OUTPUT);

  digitalWrite(HEATER_PIN, LOW);
  digitalWrite(FAN_PIN, LOW);
  digitalWrite(WATER_SPRINKLER_PIN, LOW);

  previousTime = millis();
}

void loop() {
  double current_temperature = dht.readTemperature();
  double current_humidity = dht.readHumidity();
  double current_TDS = readTDS();
  double current_pH = readpH();

  PID_Controller_Temperature(current_temperature);
  PID_Controller_Humidity(current_humidity);
  PID_Controller_TDS(current_TDS);
  PID_Controller_pH(current_pH);
  PID_Controller_AirHumidity(current_humidity);

  Serial.print("Temperatura: ");
  Serial.print(current_temperature);
  Serial.print(" °C | Umidade: ");
  Serial.print(current_humidity);
  Serial.print(" % | TDS: ");
  Serial.print(current_TDS);
  Serial.print(" ppm | pH: ");
  Serial.print(current_pH);
  Serial.print(" | Umidade do Ar: ");
  Serial.println(current_humidity);

  delay(1000); // Aguarda um segundo antes de realizar a próxima leitura
}

double readTDS() {
  int rawValue = ads.readADC_SingleEnded(TDS_PIN);
  double TDSValue = map(rawValue, 0, 1023, 0, 5000); // Mapeia a leitura para o intervalo de TDS (0 a 5000 ppm)
  return TDSValue;
}

double readpH() {
  // Lógica para ler o valor de pH do módulo de pH
  // Implemente esta parte de acordo com a documentação do seu módulo específico
  // e o método de calibração que você está utilizando
  return 6.8 + random(-10, 10) / 10.0;
}

void PID_Controller_Temperature(double current_temperature) {
  // Implemente o controle PID para temperatura
  // ...

  // Exemplo simples para ligar/desligar o aquecedor
  if (current_temperature < setpoint_temperature - 0.5) {
    digitalWrite(HEATER_PIN, HIGH);
  } else if (current_temperature > setpoint_temperature + 0.5) {
    digitalWrite(HEATER_PIN, LOW);
  }
}

void PID_Controller_Humidity(double current_humidity) {
  // Implemente o controle PID para umidade
  // ...

  // Exemplo simples para ligar/desligar o umidificador
  if (current_humidity < setpoint_humidity - 5) {
    digitalWrite(FAN_PIN, HIGH);
  } else if (current_humidity > setpoint_humidity + 5) {
    digitalWrite(FAN_PIN, LOW);
  }
}

void PID_Controller_TDS(double current_TDS) {
  // Implemente o controle PID para TDS
  // ...

  // Exemplo simples para ligar/desligar dispositivos relacionados ao TDS
  // ...
}

void PID_Controller_pH(double current_pH) {
  // Implemente o controle PID para pH
  // ...

  // Exemplo simples para ligar/desligar dispositivos relacionados ao pH
  // ...
}

void PID_Controller_AirHumidity(double current_air_humidity) {
  error_air_humidity = setpoint_air_humidity - current_air_humidity;
  double currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000.0;

  double P = Kp * error_air_humidity;
  cumError_air_humidity += error_air_humidity * elapsedTime;
  double I = Ki * cumError_air_humidity;
  rateError_air_humidity = (error_air_humidity - lastError_air_humidity) / elapsedTime;
  double D = Kd * rateError_air_humidity;

  output_air_humidity = P + I + D;
  output_air_humidity = constrain(output_air_humidity, outMin, outMax);

  digitalWrite(WATER_SPRINKLER_PIN, output_air_humidity > 50);
  lastError_air_humidity = error_air_humidity;
  previousTime = currentTime;
}
