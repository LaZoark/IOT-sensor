#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <WiFiManager.h>  // https://github.com/tzapu/WiFiManager WiFi Configuration Magic
#include <ArduinoJson.h>  // https://github.com/bblanchon/ArduinoJson
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
#include <WiFiClientSecure.h>
#include <CertStoreBearSSL.h>
#include <TZ.h>
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

const char* fw_version = FW_VERSION;
const char* host = "raw.githubusercontent.com";
const int httpsPort = 443;

// DigiCert High Assurance EV Root CA
const char trustRoot[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDrzCCApegAwIBAgIQCDvgVpBCRrGhdWrJWZHHSjANBgkqhkiG9w0BAQUFADBh
MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3
d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBD
QTAeFw0wNjExMTAwMDAwMDBaFw0zMTExMTAwMDAwMDBaMGExCzAJBgNVBAYTAlVT
MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j
b20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IENBMIIBIjANBgkqhkiG
9w0BAQEFAAOCAQ8AMIIBCgKCAQEA4jvhEXLeqKTTo1eqUKKPC3eQyaKl7hLOllsB
CSDMAZOnTjC3U/dDxGkAV53ijSLdhwZAAIEJzs4bg7/fzTtxRuLWZscFs3YnFo97
nh6Vfe63SKMI2tavegw5BmV/Sl0fvBf4q77uKNd0f3p4mVmFaG5cIzJLv07A6Fpt
43C/dxC//AH2hdmoRBBYMql1GNXRor5H4idq9Joz+EkIYIvUX7Q6hL+hqkpMfT7P
T19sdl6gSzeRntwi5m3OFBqOasv+zbMUZBfHWymeMr/y7vrTC0LUq7dBMtoM1O/4
gdW7jVg/tRvoSSiicNoxBN33shbyTApOB6jtSj1etX+jkMOvJwIDAQABo2MwYTAO
BgNVHQ8BAf8EBAMCAYYwDwYDVR0TAQH/BAUwAwEB/zAdBgNVHQ4EFgQUA95QNVbR
TLtm8KPiGxvDl7I90VUwHwYDVR0jBBgwFoAUA95QNVbRTLtm8KPiGxvDl7I90VUw
DQYJKoZIhvcNAQEFBQADggEBAMucN6pIExIK+t1EnE9SsPTfrgT1eXkIoyQY/Esr
hMAtudXH/vTBH1jLuG2cenTnmCmrEbXjcKChzUyImZOMkXDiqw8cvpOp/2PV5Adg
06O/nVsJ8dWO41P0jmP6P6fbtGbfYmbW0W5BjfIttep3Sp+dWOIrWcBAI+0tKIJF
PnlUkiaY4IBIqDfv8NZ5YBberOgOzW6sRBc4L0na4UU+Krk2U886UAb3LujEV0ls
YSEY1QSteDwsOoBrp+uvFRTp2InBuThs4pFsiv9kuXclVzDAGySj4dzp30d8tbQk
CAUw7C29C79Fv1C5qfPrmAESrciIxpg0X40KPMbp1ZWVbd4=
-----END CERTIFICATE-----
)EOF";
X509List cert(trustRoot);

extern const unsigned char caCert[] PROGMEM;
extern const unsigned int caCertLen;

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
String fallback_ap_ssid = AP_PREFIX;
const char *fallback_ap_password = "12345678";
// Wi-Fi Manager and configs
WiFiManager wm;
// WiFiManagerParameter: id/name, placeholder/prompt, default, length
WiFiManagerParameter custom_mqtt_broker("mqtt_broker", "mqtt_broker", __MQTT_BROKER, 40);
WiFiManagerParameter custom_mqtt_topic("mqtt_topic", "mqtt_topic", __MQTT_TOPIC, 32);
WiFiManagerParameter custom_mqtt_username("mqtt_username", "mqtt_username", __MQTT_USERNAME, 32);
WiFiManagerParameter custom_mqtt_password("mqtt_password", "mqtt_password", __MQTT_PASSWORD, 32);
WiFiManagerParameter custom_mqtt_port("mqtt_port", "mqtt_port", __MQTT_PORT, 6);
// MQTT config
// const char *mqtt_broker = __MQTT_BROKER;
// const char *mqtt_topic = __MQTT_TOPIC;
// const char *mqtt_username = __MQTT_USERNAME;
// const char *mqtt_password = __MQTT_PASSWORD;
// const char *mqtt_port_str = __MQTT_PORT;
char mqtt_broker[40];
char mqtt_topic[32];
char mqtt_username[32];
char mqtt_password[32];
char mqtt_port_str[6];
int mqtt_port;
// Misc. config
int mqtt_watchdog = __MQTT_WATCHDOG;
unsigned long mqtt_sensor_update_ms = __UPDATE_INTERVAL_MS;
unsigned long mqtt_sys_update_ms = __UPDATE_SYS_INTERVAL_MS;
// Only when the "mqtt_sensor_update_ms" was updated the value will become non-zero (default 2000).
// This value also enables the `force trigger` when the "mqtt_sensor_update_ms" was updated
unsigned long force_update_delay_ms = 0;
unsigned long last_update_sensor = 0;
unsigned long last_update_sys = 0;
char macStr[13] = { 0 };
String client_id = "esp8266-sensor-";
#ifdef DHT11_PIN
#define SENSOR_PIN DHT11_PIN
#define SENSOR_TYPE DHT11
#define ERROR_SENSOR_STR "Can't read DHT11. Please check the hardware or use \"help\" & \"info\" under \"/cmd\" to debug."
#elif DHT22_PIN
#define SENSOR_PIN DHT22_PIN
#define SENSOR_TYPE DHT22
#define ERROR_SENSOR_STR "Can't read DHT22. Please check the hardware or use \"help\" & \"info\" under \"/cmd\" to debug."
#endif
DHT dht_sensor(SENSOR_PIN, SENSOR_TYPE, 11);
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
String sys_info(void);
String help_msg(void);
String sensor_logger(bool state, float temperature, float humidity);
String state_logger(bool state);
void FirmwareUpdate(void);
void setClock(void);
// void blink_led(uint8 ledPin, long interval);

// Global variable
bool sensor_state = false;
String mqtt_topic_mac;
String mqtt_topic_log;
String mqtt_topic_data;
String mqtt_topic_state;
String mqtt_topic_log_ntp;

// JSON config
JsonDocument pub_doc, help_doc, log_doc, sub_doc, state_doc;  // Allocate memory for the sub_doc array.

void setup() {
  wm.setBreakAfterConfig(true);     // Save parameters even on wifi fail or empty (will call `saveConfigCallback`).
  wm.setConfigPortalTimeout(120);   // Set a timeout for captive portal
  wm.addParameter(&custom_mqtt_broker);
  wm.addParameter(&custom_mqtt_topic);
  wm.addParameter(&custom_mqtt_username);
  wm.addParameter(&custom_mqtt_password);
  wm.addParameter(&custom_mqtt_port);
  strcpy(mqtt_broker, custom_mqtt_broker.getValue());
  strcpy(mqtt_topic, custom_mqtt_topic.getValue());
  strcpy(mqtt_username, custom_mqtt_username.getValue());
  strcpy(mqtt_password, custom_mqtt_password.getValue());
  strcpy(mqtt_port_str, custom_mqtt_port.getValue());
  mqtt_port = atoi(mqtt_port_str);      // Turn mqtt port to int

  uint8_t mac[6];
  WiFi.macAddress(mac);
  sprintf(macStr, "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  client_id += String(macStr);

  WiFi.mode(WIFI_STA);
  pinMode(SENSOR_PIN, INPUT);
  dht_sensor.begin();
  Serial.begin(115200);
  // Serial.setDebugOutput(true);
  Serial.printf("\nSDK version: %s\n", system_get_sdk_version());
  Serial.printf("Free Heap: %4d\n", ESP.getFreeHeap());
  // Setting ESP into STATION mode only (no AP mode or dual mode)
  wifi_set_opmode(STATION_MODE);
  wifi_connect_init();

  ArduinoOTA.setHostname(client_id.c_str());
  ArduinoOTA.setPassword(__OTA_LOCAL_PASSWORD);
  ArduinoOTA.onStart([]() {
  String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
  // NOTE: if updating FS this would be the place to unmount FS using FS.end()
  Serial.println("Start updating " + type); mqtt_sensor_update_ms=100000; mqtt_sys_update_ms=100000; force_update_delay_ms=100000;});
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
  mqtt_client.setBufferSize(__MQTT_MAX_PACKET_SIZE);
  // mqtt_connect();
  mqtt_topic_mac = String(mqtt_topic) + "/" + macStr;
  mqtt_topic_log = String(mqtt_topic) + "/" + macStr + "/log";
  mqtt_topic_data = String(mqtt_topic) + "/" + macStr + "/data";
  mqtt_topic_state = String(mqtt_topic) + "/" + macStr + "/state";
  mqtt_topic_log_ntp = String(mqtt_topic) + "/" + macStr + "/log/ntp";
  ESPhttpUpdate.rebootOnUpdate(false); // remove automatic reboot on update
}


void loop()
{
  if (WiFi.status() != WL_CONNECTED)
    wifi_connect_init();
  if (!mqtt_client.connected())
    mqtt_connect();
  ArduinoOTA.handle();
  mqtt_client.loop();
// Sense and publish the data
  if ( (millis() - last_update_sensor) >= mqtt_sensor_update_ms ) {
    float temperature=0, humidity=0;
    sensor_state = dht_sensor.read();
    if (sensor_state) {
      humidity = dht_sensor.readHumidity();
      temperature = dht_sensor.readTemperature();
    } else {
      Serial.printf("%s %lu(ms)\n", ERROR_SENSOR_STR, millis());
    }
    mqtt_client.publish(mqtt_topic_data.c_str(), sensor_logger(sensor_state, temperature, humidity).c_str());
    last_update_sensor = millis();
  }
// Publish the system info
  if ( (millis() - last_update_sys) >= ((force_update_delay_ms==0) ? mqtt_sys_update_ms : force_update_delay_ms)) {
    mqtt_client.publish(mqtt_topic_state.c_str(), state_logger(sensor_state).c_str());
    last_update_sys = millis();
    force_update_delay_ms = 0;      // Set to 0 again since the `force update` was done.
  }
}

void callback(char *topic, byte *payload, unsigned int length) {
  char res_buffer[96];
  String buffer;
  for (unsigned int i = 0; i < length; i++)
    buffer += (char)payload[i];
  if (buffer == "reboot") {
    ESP.restart();
  } else if (buffer == "help") {
    String temp_payload = help_msg();
    mqtt_client.publish(mqtt_topic_log.c_str(), temp_payload.c_str());
    Serial.println(temp_payload);
  } else if (buffer == "info") {
    String temp_payload = sys_info();
    mqtt_client.publish(mqtt_topic_log.c_str(), temp_payload.c_str());
    Serial.println(temp_payload);
  } else if (buffer == "reset-wifi") {
    mqtt_client.publish(mqtt_topic_mac.c_str(), "[INFO] Resetting the Wi-Fi (resetSettings)");
    wm.resetSettings();
  } else if (buffer == "config") {  /// # BUG: It appears to OOM when checking ota (check-ota) after connected to another Wi-Fi. Workaround: use "reset-wifi"
    mqtt_client.publish(mqtt_topic_mac.c_str(), "[INFO] Entering config mode (startConfigPortal)");
    wm.startConfigPortal(fallback_ap_ssid.c_str());
  } else if (buffer == "ping") {
    mqtt_client.publish(mqtt_topic_log.c_str(), String("{\"ping\":\"" + client_id + "\",\"t\":" + system_get_time() + "}").c_str());  // nano second
    Serial.println(String("{\"ping\":\"" + client_id + "\",\"t\":" + system_get_time() + "}"));
  } else if (buffer == "check-ota") {
    mqtt_client.publish(mqtt_topic_log.c_str(), String("{\"ota\":\"Checking latest firmware... (current=" + String(fw_version) + ")\"}").c_str());
    Serial.println(String("{\"ota\":\"Checking latest firmware... (current=" + String(fw_version) + ")\"}"));
    FirmwareUpdate();
  } else if (buffer == "time") {
    mqtt_client.publish(mqtt_topic_log_ntp.c_str(), "{\"ntp\":\"Checking time from NTP...\"}");
    setClock();
  } else if (String(buffer[0]) == String('{')) {  /*JSONconfig*/
    deserializeJson(sub_doc, payload, length);  // Convert JSON string to something useable.
    if (!sub_doc["pub_ms"].isNull()) {
      int _pub_ms = sub_doc["pub_ms"];
      if (_pub_ms >= __MINIMAL_UPDATE_INTERVAL_MS) {
        snprintf(res_buffer, sizeof(res_buffer), "{\"config\":\"MQTT publish interval from %lu set to %d\"}", mqtt_sensor_update_ms, _pub_ms);
        mqtt_sensor_update_ms = _pub_ms;
        // Active trigger: Force update the "/state" after overwriting the "mqtt update interval"
        force_update_delay_ms = __FORCE_UPDATE_DELAY_MS;
      } else {
        snprintf(res_buffer, sizeof(res_buffer), "{\"error\":\"%d ms is too short. The value must larger than %d.\"}", _pub_ms, __MINIMAL_UPDATE_INTERVAL_MS);
      } 
    } else if (!sub_doc["cpu_mhz"].isNull()) {
      int _cpu_mhz = sub_doc["cpu_mhz"];
      if (_cpu_mhz == SYS_CPU_80MHZ || _cpu_mhz == SYS_CPU_160MHZ) {
        snprintf(res_buffer, sizeof(res_buffer), "{\"config\":\"CPU frequency from %d set to %d\"}", ESP.getCpuFreqMHz(), _cpu_mhz);
        if (!system_update_cpu_freq(_cpu_mhz)) {
          snprintf(res_buffer, sizeof(res_buffer), "{\"error\":\"CPU frequency failed from setting %d to %d\"}", ESP.getCpuFreqMHz(), _cpu_mhz);
          mqtt_client.publish(mqtt_topic_log.c_str(), res_buffer);
          Serial.println(res_buffer);
          }
      } else {
        snprintf(res_buffer, sizeof(res_buffer), "{\"error\":\"%d mhz is not allowed. Only 80 & 160 are supported.\"}", _cpu_mhz);
      } 
    } else {
      snprintf(res_buffer, sizeof(res_buffer), "{\"error\":\"Unsupported JSON config: %s\"}", buffer.c_str());
    }
    mqtt_client.publish(mqtt_topic_log.c_str(), res_buffer);
    Serial.println(res_buffer);
  } else {
    Serial.printf("[%s]: Unknown command: \"%s\"", topic, buffer.c_str());
    Serial.printf("\n----------------------- %lu(ms) -----------------------\n", millis());
    snprintf(res_buffer, sizeof(res_buffer), "{\"error\":\"Unknown command: %s\"}", buffer.c_str());
    mqtt_client.publish(mqtt_topic_log.c_str(), res_buffer);
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
  fallback_ap_ssid = AP_PREFIX + split_macStr;
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
      Serial.printf("No luck, retrying... <%d/%d> (with state %d)\n", mqtt_watchdog--, __MQTT_WATCHDOG, mqtt_client.state());
      delay(1000);
    }
    if (mqtt_watchdog == 0)
      ESP.restart();
  }
  String temp_payload = sys_info();
  Serial.println(temp_payload);
  mqtt_client.publish(mqtt_topic_log.c_str(), temp_payload.c_str());
  mqtt_client.subscribe((String(mqtt_topic) + __MQTT_TOPIC_CMD).c_str());                         // For broadcast
  mqtt_client.subscribe((String(mqtt_topic) + "/" + macStr + __MQTT_TOPIC_CMD).c_str());          // For specifiy by MAC
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

void FirmwareUpdate(void) {
  setClock();
  BearSSL::WiFiClientSecure https_client;
  char res_buffer[80];
  https_client.setTrustAnchors(&cert);
  https_client.setTimeout(10000);
  int res_code = https_client.connect(host, httpsPort);
  if (!res_code) {
    // Serial.printf("Connection failed: res=%d, (status=%d)\n", res_code, https_client.status());
    Serial.printf("Connection failed: res=%d\n", res_code);
    Serial.println(https_client.status());
    Serial.printf("host: %s\n", host);
    Serial.printf("port: %d\n", httpsPort);
    return;
  }
  https_client.print(
    String("GET ") + URL_FW_VERSION + " HTTP/1.1\r\n" +
    "Host: " + host + "\r\n" +
    "User-Agent: BuildFailureDetectorESP8266\r\n" +
    "Connection: close\r\n\r\n");
  while (https_client.connected()) {
    String line = https_client.readStringUntil('\n');
    if (line == "\r") {
      // Serial.println("Headers received");
      break;
    }
  }
  // String payload = https_client.readStringUntil('\n');
  String payload;
  while(https_client.available()){        
    payload = https_client.readStringUntil('\n');  //Read Line by Line
    Serial.println(payload); //Print response
  }
  Serial.println("==========");

  payload.trim();
  /// debug
  // Serial.print("***** payload = ");
  // Serial.println(payload);
  // Serial.print("***** fw_version = ");
  // Serial.println(fw_version);
  if (payload.equals(fw_version)) {
    snprintf(res_buffer, sizeof(res_buffer), "{\"ota\":\"Device firmware is up-to-date.\",\"ver\":\"%s\"}", payload.c_str());
    Serial.println(res_buffer);
    mqtt_client.publish(mqtt_topic_log.c_str(), res_buffer);
  } else {
    snprintf(res_buffer, sizeof(res_buffer), "{\"ota\":\"New firmware detected (%s). Downloading...\"}", payload.c_str());
    Serial.println(res_buffer);
    mqtt_client.publish(mqtt_topic_log.c_str(), res_buffer);
    // ESPhttpUpdate.setLedPin(LED_BUILTIN, LOW); 
    t_httpUpdate_return ret = ESPhttpUpdate.update(https_client, URL_FW_BIN);
    // Based on the HTTP update result, display the corresponding message
    switch (ret) {
      case HTTP_UPDATE_FAILED:
        snprintf(res_buffer, sizeof(res_buffer), "{\"ota\":\"HTTP_UPDATE_FAILD Error (%d): %s\"}", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
        break;
      case HTTP_UPDATE_NO_UPDATES:
        snprintf(res_buffer, sizeof(res_buffer), "{\"ota\":\"HTTP_UPDATE_NO_UPDATES\"}");
        break;
      case HTTP_UPDATE_OK:
        snprintf(res_buffer, sizeof(res_buffer), "{\"ota\":\"HTTP_UPDATE_OK\"}");
        break;
      default:
        snprintf(res_buffer, sizeof(res_buffer), "{\"ota\":\"Done. ret=%d\"}", ret);
        break;
    }
    Serial.println(res_buffer);
    mqtt_client.publish(mqtt_topic_log.c_str(), res_buffer);
    if (ret == HTTP_UPDATE_OK) ESP.restart();
  }
}  



// Set time via NTP, as required for x.509 validation
void setClock() {
  configTime(TZ_Asia_Taipei, "tock.stdtime.gov.tw", "time.stdtime.gov.tw", "asia.pool.ntp.org");
  // Serial.print("{\"ntp\":\"Waiting for NTP time sync...\"}");
  Serial.print(F("[NTP] Waiting for NTP time sync..."));
  mqtt_client.publish(mqtt_topic_log_ntp.c_str(), "{\"ntp\":\"Waiting for NTP time sync...\"}");
  time_t now = time(nullptr);
  unsigned long last_check_ntp = 0;
  while (now < 8 * 3600 * 2) {
    if ((millis() - last_check_ntp) >= 100) {
      if (last_check_ntp % 500)
        Serial.print(".");
      now = time(nullptr);
      last_check_ntp = millis();
    }
  }
  Serial.println();
  struct tm timeinfo;
  // gmtime_r(&now, &timeinfo);
  localtime_r(&now, &timeinfo);

  char res_buffer[64];
  char time_buffer[20];
  strftime(time_buffer, sizeof(time_buffer), "%Y-%m-%d,%H:%M:%S", &timeinfo);
  snprintf(res_buffer, sizeof(res_buffer), "{\"ntp\": {\"localtime\":\"%s\"}}", time_buffer);
  // snprintf(res_buffer, sizeof(res_buffer), "{\"ntp\": {\"localtime\":\"%s\"}}", asctime(&timeinfo));
  mqtt_client.publish(mqtt_topic_log_ntp.c_str(), res_buffer);
  Serial.println(res_buffer);
  // return now;
  return;
}



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
  pub_doc["model"] = PROJECT;                               // PROJECT
  pub_doc["ver"] = fw_version;                              // fw_version
  pub_doc["boot_ms"] = system_get_time() / 1000;            // boot_time_ms
	char json_output[measureJson(pub_doc) + 1]; // Account for null-terminator, else trailing garbage is returned
	serializeJson(pub_doc, json_output, sizeof(json_output));
  return json_output;
}
String help_msg( ) {
  help_doc["help"]       =  "Show this help";
  help_doc["reboot"]     =  "Reboot";
  help_doc["info"]       =  "Show the system info";
  help_doc["reset-wifi"] =  "Clear and reset the Wi-Fi";
  help_doc["config"]     =  "Start the config portal";
  help_doc["JSONconfig"] =  "Modify options from <info> in JSON form";
  help_doc["ping"]       =  "Testing";
  help_doc["time"]       =  "Time calibration";
  help_doc["check-ota"]  =  "Perform the OTA";
  help_doc["boot_ms"]    =  system_get_time() / 1000;
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
  state_doc["heap"] = ESP.getFreeHeap();
	char json_output[measureJson(state_doc) + 1];   // Account for null-terminator, else trailing garbage is returned
	serializeJson(state_doc, json_output, sizeof(json_output));
  return json_output;
}
