# Practica ESP32 con Node-Red.
Este repositorio muestra como podemos programar una ESP32 con el sensor ultrasonico, mostrar los resultados de distancia en node-red atraves de indicadores y graficos.

## Introducción

### Descripción

La Esp32 la utilizamos en un entorno de adquision de datos, lo cual en esta practica ocuparemos un sensor ultrasonico para adquirir la distancia del sensor con el muro que se vera reflejado en Node-Red como grafica y como indicador; Cabe aclarar que esta practica se usara un simulador llamado [WOKWI](https://https://wokwi.com/).


## Material Necesario

Para realizar esta practica necesitas lo siguiente

- [WOKWI](https://https://wokwi.com/)
- Tarjeta ESP 32
- Sensor ultrasonico
- Programa Node-Red (previamente instalado en https://github.com/DiegoJm10/Node-red-instalacion)



## Instrucciones

### Requisitos previos

Para poder usar este repositorio necesitas entrar a la plataforma [WOKWI](https://https://wokwi.com/).


### Instrucciones de preparación de entorno 

1. Abrir la terminal de programación y colocar la siguente programación:

```
#include <ArduinoJson.h>
#include <WiFi.h>
#include <PubSubClient.h>
#define BUILTIN_LED 2
#include "DHTesp.h"
const int DHT_PIN = 15;
DHTesp dhtSensor;
// Update these with values suitable for your network.

const char* ssid = "Wokwi-GUEST";
const char* password = "";
const char* mqtt_server = "44.195.202.69"; 
String username_mqtt="jorgeroldan";
String password_mqtt="1234";

WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE  (50)
char msg[MSG_BUFFER_SIZE];
int value = 0;

const int Trigger = 4;   //Pin digital 2 para el Trigger del sensor
const int Echo = 27;   //Pin digital 3 para el Echo del sensor

void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
    digitalWrite(BUILTIN_LED, LOW);   
    // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is active low on the ESP-01)
  } else {
    digitalWrite(BUILTIN_LED, HIGH);  
    // Turn the LED off by making the voltage HIGH
  }

}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(), username_mqtt.c_str() , password_mqtt.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic", "hello world");
      // ... and resubscribe
      client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {
  pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  dhtSensor.setup(DHT_PIN, DHTesp::DHT22);
  pinMode(Trigger, OUTPUT); //pin como salida
  pinMode(Echo, INPUT);  //pin como entrada
  digitalWrite(Trigger, LOW);//Inicializamos el pin con 0
}

void loop() {


delay(1000);

long t; //timepo que demora en llegar el eco
  long d; //distancia en centimetros

  digitalWrite(Trigger, HIGH);
  delayMicroseconds(10);          //Enviamos un pulso de 10us
  digitalWrite(Trigger, LOW);
  
  t = pulseIn(Echo, HIGH); //obtenemos el ancho del pulso
  d = t/59;             //escalamos el tiempo a una distancia en cm

TempAndHumidity  data = dhtSensor.getTempAndHumidity();
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  unsigned long now = millis();
  if (now - lastMsg > 2000) {
    lastMsg = now;
    //++value;
    //snprintf (msg, MSG_BUFFER_SIZE, "hello world #%ld", value);

    StaticJsonDocument<128> doc;

    doc["DEVICE"] = "ESP32";
    //doc["Anho"] = 2022;
    //doc["Empresa"] = "Educatronicos";
    //doc["TEMPERATURA"] = String(data.temperature, 1);
    //doc["HUMEDAD"] = String(data.humidity, 1);
    doc["DISTANCIA"] = d;
   

    String output;
    
    serializeJson(doc, output);

    Serial.print("Publish message: ");
    Serial.println(output);
    Serial.println(output.c_str());
    client.publish("jorge/diplomado", output.c_str());
  }

}
```

2. Instalar las librerias de *Arduino.Json*, *PubSubClient*, *DHT sensor library for ESPx* como se muestra en la siguente imagen.

![](https://github.com/jroldanap/esp32connered/blob/main/librerias.png?raw=true)

3. Hacer la conexion de *seonsor ultrasonico*, con la *ESP32* como se muestra en la siguente imagen.

![](https://github.com/jroldanap/esp32noderedultrasonic/blob/main/ultra.png?raw=true)

4. Poner la funcion mqtt in en programa Node-Red y cambiar el topic a *jorge/diplomado* y el id del servidor a 44.195.202.69, como se muestar en la imagen.

![](https://github.com/jroldanap/esp32connered/blob/main/mqqt.png?raw=true)

![](https://github.com/jroldanap/esp32connered/blob/main/id.png?raw=true)

5. Añadir el bloque json y cambiar la Action a *Always convert to script object*.

![](https://github.com/jroldanap/esp32connered/blob/main/json.png?raw=true)

6. Añadir el bloque function y cambiar el Name a *temperatura* y pegar el siguiente codigo:

msg.payload = msg.payload.DISTANCIA;

msg.topic = "DISTANCIA";

return msg;

 en la barra como se muestar en al imagen.

![](https://github.com/jroldanap/esp32noderedultrasonic/blob/main/DIST.png?raw=true)

7. Añadir el bloque gauge (1) y poner la configuracion como se muestra en la imagen.

![](https://github.com/jroldanap/esp32noderedultrasonic/blob/main/ga.png?raw=true)


8. Añadir el bloque chart y poner la configuracion como se muestra en la imagen.

![](https://github.com/jroldanap/esp32noderedultrasonic/blob/main/graf.png?raw=true)


### Instrucciónes de operación

1. Iniciar simulador dando click en el boton verde de play.
2. Visualizar los datos en el monitor serial.
3. Colocar la distancia dando doble click al sensor *ultasonico* 
4. Visualizar los datos en el monitor serial y en las graficas de Node-Red.

## Resultados

Cuando haya funcionado, verás los valores dentro del monitor serial y en las graficas de Node-Red como se muestra en la siguente imagen.

![](https://github.com/jroldanap/esp32noderedultrasonic/blob/main/ul.png?raw=true)

![](https://github.com/jroldanap/esp32noderedultrasonic/blob/main/indica.png?raw=true)




## Evidencias de programa corriendo

![](https://github.com/jroldanap/esp32noderedultrasonic/blob/main/ul.png?raw=true)

![](https://github.com/jroldanap/esp32noderedultrasonic/blob/main/indica.png?raw=true)


# Créditos

Desarrollado por Jorge Alberto Roldan Aponte

- [GitHub](https://github.com/jroldanap)