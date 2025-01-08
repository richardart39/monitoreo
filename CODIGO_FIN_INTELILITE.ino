#include <DHT.h>
#include <STM32FreeRTOS.h>
#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Configuración de LCD, LEDs, Ethernet y sensores
LiquidCrystal_I2C lcd(0x27, 16, 2);
const int ledConectado = PC3;
const int ledDesconectado = PC2;

// Configuración de la dirección MAC e IP fija
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(172, 30, 33, 20);      // Dirección IP fija
IPAddress gateway(172, 30, 33, 255);  // Puerta de enlace
IPAddress subnet(255, 255, 255, 0); // Máscara de red
IPAddress dns(8, 8, 8, 8);          // Servidor DNS (puedes usar otro)

// Configuración del servidor MQTT
const char* mqttServer = "192.168.150.167";
const int mqttPort = 1883;

// Configuración del DHT22
#define DHTPIN PB7
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// Configuración del MQ135
const int mq135Pin = PA1;

// Variables para el nivel de combustible
int percentage = 0;

EthernetClient ethClient;
PubSubClient client(ethClient);

// Callback para gestionar mensajes MQTT
void callback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  if (String(topic) == "sensor/combustible") {
    percentage = message.toInt();
    Serial.println("Nivel de combustible recibido: " + String(percentage) + "%");
  }
}

void setup() {
  Serial.begin(9600);
  lcd.init();
  lcd.backlight();
  pinMode(ledConectado, OUTPUT);
  pinMode(ledDesconectado, OUTPUT);

  // Inicializar DHT
  dht.begin();

  // Configuración manual de IP
  Ethernet.begin(mac, ip, dns, gateway, subnet);

  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("No se encontró hardware Ethernet");
    lcd.print("No hay HW Ethernet");
    while (true);
  }

  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Cable Ethernet desconectado");
    lcd.print("Sin conexión cable");
    while (true);
  }

  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);

  // Creación de tareas FreeRTOS
  xTaskCreate(tareaConexionMQTT, "Conexion MQTT", 128, NULL, 1, NULL);
  xTaskCreate(tareaActualizarLCD, "Actualizar LCD", 128, NULL, 1, NULL);
  xTaskCreate(tareaLeerDHT22, "Leer DHT22", 128, NULL, 1, NULL);
  xTaskCreate(tareaLeerMQ135, "Leer MQ135", 128, NULL, 1, NULL);

  vTaskStartScheduler();
}

void loop() {
  // No es necesario código aquí, ya que FreeRTOS maneja las tareas.
}

// Tarea 1: Conexión y gestión de LEDs
void tareaConexionMQTT(void *pvParameters) {
  while (true) {
    if (!client.connected()) {
      digitalWrite(ledDesconectado, HIGH);
      digitalWrite(ledConectado, LOW);
      while (!client.connected()) {
        if (client.connect("STM32F446RE")) {
          digitalWrite(ledConectado, HIGH);
          digitalWrite(ledDesconectado, LOW);
          client.subscribe("sensor/combustible");
        } else {
          vTaskDelay(pdMS_TO_TICKS(5000));
        }
      }
    }
    client.loop();
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// Tarea 2: Actualizar la pantalla LCD con la IP del dispositivo
void tareaActualizarLCD(void *pvParameters) {
  while (true) {
    IPAddress ip = Ethernet.localIP();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("IP:");
    lcd.setCursor(3, 0);
    lcd.print(ipToString(ip));
    lcd.setCursor(0, 1);
    lcd.print(client.connected() ? "MQTT: Conectado " : "MQTT: Desconect");
    vTaskDelay(pdMS_TO_TICKS(5000));

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Combustible:");
    int barLength = map(percentage, 0, 100, 0, 10);
    lcd.setCursor(0, 1);
    for (int i = 0; i < barLength; i++) {
      lcd.print((char)255);
    }
    lcd.setCursor(11, 1);
    lcd.print(percentage);
    lcd.print("%");
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

// Tarea 3: Leer datos del DHT22 y publicar en MQTT
void tareaLeerDHT22(void *pvParameters) {
  while (true) {
    float temperatura = dht.readTemperature();
    float humedad = dht.readHumidity();

    if (!isnan(temperatura) && !isnan(humedad)) {
      client.publish("sensor/temperatura", String(temperatura).c_str());
      client.publish("sensor/humedad", String(humedad).c_str());
      Serial.println("Temperatura: " + String(temperatura) + " °C");
      Serial.println("Humedad: " + String(humedad) + " %");
    } else {
      Serial.println("Error al leer DHT22");
    }

    vTaskDelay(pdMS_TO_TICKS(10000));
  }
}

// Tarea 4: Leer datos del MQ135 y publicar en MQTT
void tareaLeerMQ135(void *pvParameters) {
  while (true) {
    int calidadAire = analogRead(mq135Pin);
    client.publish("sensor/calidad_aire", String(calidadAire).c_str());
    Serial.println("Calidad de aire (MQ135): " + String(calidadAire));

    vTaskDelay(pdMS_TO_TICKS(10000));
  }
}

// Funciones auxiliares
String ipToString(IPAddress ip) {
  return String(ip[0]) + "." + String(ip[1]) + "." + String(ip[2]) + "." + String(ip[3]);
}
