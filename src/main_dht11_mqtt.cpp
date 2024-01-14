#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <WiFiManager.h>  // https://github.com/tzapu/WiFiManager WiFi Configuration Magic
#include <ArduinoJson.h>  // https://github.com/bblanchon/ArduinoJson
#include <ArduinoOTA.h>
#include <Esp.h>
#include <string>
#include <DHT_U.h>
#include "config.h"

extern "C" {
#include "user_interface.h"
#include "wpa2_enterprise.h"
#include "c_types.h"
}

// #define DEVICE_NAME "mqtt_OTA_A"
#define DHT11_PIN 2
#define ERROR_SENSOR_STR "Can't read DHT11. Please check the hardware or use \"help\" & \"info\" under \"/cmd\" to debug."
#define ERROR_SENSOR_JSON "{\"sensor\": false, \"log\": \"Please check the hardware or use \"help\" & \"info\" under \"/cmd\" to debug.\"}"
#define COMPILE_TIME __DATE__ " " __TIME__

// Wi-Fi config
#ifdef CONFIG_USING_ENTERPRISE_WIFI
const char *enterprise_wifi_ssid = __ENTERPRISE_WIFI_SSID;
const char *enterprise_wifi_identity = __ENTERPRISE_WIFI_IDENTITY;
const char *enterprise_wifi_username = __ENTERPRISE_WIFI_USERNAME;
const char *enterprise_wifi_password = __ENTERPRISE_WIFI_PASSWORD;
#elif CONFIG_USING_REGULAR_WIFI
const char *wifi_ssid = __WIFI_SSID;          // Enter your WiFi name
const char *wifi_password = __WIFI_PASSWORD;  // Enter WiFi password
#endif
// Fallback hotspot AP
String fallback_ap_ssid = "esp-fallback-";
const char *fallback_ap_password = "12345678";
// Wi-Fi Manager and configs
WiFiManager wm;
// MQTT config
const char *mqtt_broker = __MQTT_BROKER;
const char *mqtt_topic = __MQTT_TOPIC;
const char *mqtt_username = __MQTT_USERNAME;
const char *mqtt_password = __MQTT_PASSWORD;
const int mqtt_port = __MQTT_PORT;
// Misc. config
int mqtt_watchdog = __MQTT_WATCHDOG;
unsigned long mqtt_sensor_update_ms = __UPDATE_INTERVAL;
char macStr[13] = { 0 };
String client_id = "esp8266-sensor-";
DHT dht11(DHT11_PIN, DHT11, 11);
WiFiClient espClient;
PubSubClient mqtt_client(espClient);

#ifdef CONFIG_USING_ENTERPRISE_WIFI
static void wifi_enterprise_connect_init(void);
  #define wifi_connect_init() wifi_enterprise_connect_init();
#elif CONFIG_USING_REGULAR_WIFI
static void wifi_regular_connect_init(void);
  #define wifi_connect_init() wifi_regular_connect_init();
#endif
void mqtt_connect(void);
void callback(char *topic, byte *payload, unsigned int length);
double round2(double value);
// void blink_led(uint8 ledPin, long interval);

// Global variable
bool sensor_state = false;

// JSON config
JsonDocument pub_doc, help_doc, log_doc, sub_doc, state_doc;  // Allocate memory for the sub_doc array.

String sys_info( ) {
  pub_doc["node"] = client_id;                              // node
  pub_doc["ssid"] = WiFi.SSID();                            // SSID
  pub_doc["ip"] = WiFi.localIP().toString();                // ip
  pub_doc["gw"] = WiFi.gatewayIP().toString();              // gateway
  pub_doc["free_heap"] = ESP.getFreeHeap();                 // free_heap
  pub_doc["cpu_mhz"] = ESP.getCpuFreqMHz();                 // cpu_freq_mhz
  pub_doc["pub_ms"] = mqtt_sensor_update_ms;                // update_ms (publish)
//pub_doc["sdk"] = system_get_sdk_version();                // sdk
  pub_doc["compile"] = COMPILE_TIME;                        // COMPILE_TIME
  pub_doc["boot_ms"] = system_get_time() / 1000;            // boot_time_ms
  size_t jsonLength = measureJson(pub_doc) + 1; // Account for null-terminator, else trailing garbage is returned
	char json_output[jsonLength];
	serializeJson(pub_doc, json_output, sizeof(json_output));
  return json_output;
}
String help_msg( ) {
  help_doc["help"] = "Show this help.";
  help_doc["reboot"] = "Reboot.";
  help_doc["info"] = "Show the system info.";
  help_doc["reset-wifi"] = "Clear and reset the Wi-Fi.";
  help_doc["JSONconfig"] = "Modify options from <info> in JSON form.";
  help_doc["ping"] = "For testing.";
  help_doc["boot_ms"] = system_get_time() / 1000;
	char json_output[measureJson(help_doc) + 1];   // Account for null-terminator, else trailing garbage is returned
	serializeJson(help_doc, json_output, sizeof(json_output));
  return json_output;
}
String sensor_logger(bool state, float temperature, float humidity) {
  if (state) {
    log_doc["t"] = round2(temperature);
    log_doc["rh"] = round2(humidity);
  } else {
    log_doc["err"] = "Please check the hardware or use <help> & <info> under </cmd> to debug.";
  }
	char json_output[measureJson(log_doc) + 1];   // Account for null-terminator, else trailing garbage is returned
	serializeJson(log_doc, json_output, sizeof(json_output));
  return json_output;
}
String state_logger(bool state) {
  state_doc["t_ms"] = system_get_time() / 1000;
  state_doc["sensor"] = state;
  state_doc["pub_ms"] = mqtt_sensor_update_ms;
	char json_output[measureJson(state_doc) + 1];   // Account for null-terminator, else trailing garbage is returned
	serializeJson(state_doc, json_output, sizeof(json_output));
  return json_output;
}
  uint8_t mac[6];
  WiFi.macAddress(mac);
  // char macStr[13] = { 0 };
  sprintf(macStr, "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  client_id += String(macStr);

  WiFi.mode(WIFI_STA);
  pinMode(DHT11_PIN, INPUT);
  dht11.begin();
  Serial.begin(115200);
  delay(2000);
  Serial.setDebugOutput(true);
  Serial.printf("\nSDK version: %s\n", system_get_sdk_version());
  Serial.printf("Free Heap: %4d\n", ESP.getFreeHeap());
  // Setting ESP into STATION mode only (no AP mode or dual mode)
  wifi_set_opmode(STATION_MODE);
  wifi_connect_init();

  ArduinoOTA.setHostname(client_id.c_str());
  ArduinoOTA.setPassword(__OTA_PASSWORD);
  ArduinoOTA.onStart([]() {
  String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
  // NOTE: if updating FS this would be the place to unmount FS using FS.end()
  Serial.println("Start updating " + type);});
  ArduinoOTA.onEnd([]() {Serial.println("\nEnd");});
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
  Serial.printf("Progress: %u%%\r", (progress / (total / 100)));});
  ArduinoOTA.onError([](ota_error_t error) {
  Serial.printf("Error[%u]: ", error);
  if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
  else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
  else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
  else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
  else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();

  //connecting to a mqtt broker
  mqtt_client.setServer(mqtt_broker, mqtt_port);
  mqtt_client.setCallback(callback);
  mqtt_connect();
}

unsigned long last_update = 0;

void loop()
{
  if (WiFi.status() != WL_CONNECTED)
    wifi_connect_init();
  if (!mqtt_client.connected())
    mqtt_connect();
  ArduinoOTA.handle();
  mqtt_client.loop();

  if ( (millis() - last_update) > mqtt_sensor_update_ms ) {
    float temperature, humidity;
    if(dht11.read()) {
      humidity = dht11.readHumidity();
      temperature = dht11.readTemperature();
      mqtt_client.publish(String(mqtt_topic + String("/") + String(macStr) + String("/temperature")).c_str(), String(temperature).c_str()); 
      mqtt_client.publish(String(mqtt_topic + String("/") + String(macStr) + String("/humidity")).c_str(), String(humidity).c_str()); 
    } else 
      mqtt_client.publish(String(mqtt_topic + String("/") + String(macStr) + String("/error")).c_str(), 
        "Can't read DHT11. Please check the hardware or use \"reboot\" and \"info\" under \"/cmd\" for further information.");
    last_update = millis();
  }
}

void callback(char *topic, byte *payload, unsigned int length) {
  // Serial.printf("[%s]: %hhn", topic, payload);
  // Serial.printf("\n-----------------------%ld(s)\n", millis());
  std::string buffer = "";
  for (unsigned int i = 0; i < length; i++)
    buffer += (char)payload[i];
  if (buffer == "reboot")
    ESP.restart();
  else if (buffer == "info") {
    bool mqtt_res = mqtt_client.publish(String(mqtt_topic + String("/") + String(macStr)).c_str(), sys_info().c_str());
    Serial.printf("%s\n", sys_info().c_str());
    Serial.printf("Published? %d (with state %d)\n", mqtt_res, mqtt_client.state());
    }
  else if (buffer == "test") {
    bool mqtt_res = mqtt_client.publish(
      String(mqtt_topic + String("/") + String(macStr)).c_str(),
      String("\"hello world\" from [" + client_id + "] boot_time_ms: " + system_get_time() / 1000).c_str());
    Serial.printf("[test] Published? %d (with state %d)\n", mqtt_res, mqtt_client.state());
    }
  else if (buffer == "config") {
    mqtt_sensor_update_ms = 6;
  }
}

#ifdef CONFIG_USING_ENTERPRISE_WIFI
static void wifi_enterprise_connect_init(void) {
  /// thanks to https://gist.github.com/Matheus-Garbelini/2cd780aed2eddbe17eb4adb5eca42bd6
  struct station_config wifi_config;
  memset(&wifi_config, 0, sizeof(wifi_config));
  strcpy((char*)wifi_config.ssid, enterprise_wifi_ssid);
  strcpy((char*)wifi_config.password, enterprise_wifi_password);

  // wifi_set_phy_mode(PHY_MODE_11N);
  wifi_station_set_config(&wifi_config);

  // Clean up to be sure no old data is still inside
  wifi_station_clear_cert_key();
  wifi_station_clear_enterprise_ca_cert();
  // wifi_station_clear_enterprise_identity();
  // wifi_station_clear_enterprise_username();
  // wifi_station_clear_enterprise_password();
  // wifi_station_clear_enterprise_new_password();

  wifi_station_set_wpa2_enterprise_auth(1);
  
  wifi_station_set_enterprise_identity((uint8*)enterprise_wifi_identity, strlen(enterprise_wifi_identity));
  wifi_station_set_enterprise_username((uint8*)enterprise_wifi_username, strlen(enterprise_wifi_username));
  wifi_station_set_enterprise_password((uint8*)enterprise_wifi_password, strlen(enterprise_wifi_password));

  wifi_station_connect();
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.printf("Wifi is ready. (IP: %s)\n", WiFi.localIP().toString().c_str());
}
#elif CONFIG_USING_REGULAR_WIFI
static void wifi_regular_connect_init() {
  String split_macStr = String(macStr);
  split_macStr.remove(0, 6);
  fallback_ap_ssid += split_macStr;
  // wm.autoConnect();
  // wm.autoConnect(fallback_ap_ssid.c_str(), fallback_ap_password);
  wm.autoConnect(fallback_ap_ssid.c_str());
  // WiFi.begin(wifi_ssid, wifi_password);
  // while (WiFi.status() != WL_CONNECTED)
  // {
  //   delay(500);
  //   Serial.print(".");
  // }
  Serial.printf("Wifi is ready (IP: %s)\n", WiFi.localIP().toString().c_str());
}
#endif

void mqtt_connect() {
  while (!mqtt_client.connected())
  {
    Serial.printf("[%s] Connecting to MQTT broker \"%s:%s\"\n", client_id.c_str(), mqtt_broker, mqtt_port_str);
    if (mqtt_client.connect(client_id.c_str(), mqtt_username, mqtt_password))
      Serial.println("MQTT Connected!");
    else
    {
      Serial.printf("No luck, retrying... <%d/%d> (with state %d)", mqtt_watchdog--, __MQTT_WATCHDOG, mqtt_client.state());
      delay(2000);
    }
    if (mqtt_watchdog == 0)
      ESP.restart();
  }
  bool mqtt_res = mqtt_client.publish(mqtt_topic, sys_info().c_str());
  Serial.printf("%s\n", sys_info().c_str());
  Serial.printf("Published? %d (with state %d)\n", mqtt_res, mqtt_client.state());
  mqtt_client.subscribe((String(mqtt_topic) + String(__MQTT_TOPIC_CMD)).c_str());                         // For broadcast
  mqtt_client.subscribe((String(mqtt_topic) + "/" + String(macStr) + String(__MQTT_TOPIC_CMD)).c_str());  // For specifiy by MAC
}



// void blink_led(uint8 ledPin, long interval){
//   digitalWrite(ledPin, HIGH);
//   delay(long(interval/2));
//   digitalWrite(ledPin, LOW);
//   delay(long(interval/2));


  // int ledState = LOW;             // ledState used to set the LED
  // // Generally, you should use "unsigned long" for variables that hold time
  // // The value will quickly become too large for an int to store
  // unsigned long previousMillis = 0;        // will store last time LED was updated
  // unsigned long currentMillis = millis();

  // if (currentMillis - previousMillis >= interval) {
  //   // save the last time you blinked the LED
  //   previousMillis = currentMillis;

  //   // if the LED is off turn it on and vice-versa:
  //   if (ledState == LOW) {
  //     ledState = HIGH;
  //   } else {
  //     ledState = LOW;
  //   }

  //   // set the LED with the ledState of the variable:
  //   digitalWrite(ledPin, ledState);
  // }

// }

// rounds a number to 2 decimal places. Example: round(3.14159) -> 3.14
double round2(double value) {
  return (int)(value * 100 + 0.5) / 100.0;
}
