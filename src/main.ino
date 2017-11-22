// vim:softtabstop=4:shiftwidth=4:tabstop=64:filetype=cpp
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>
#include <FS.h>
#include <Ticker.h>
#include <Wire.h> 
#include <SPI.h>
#include <time.h>
#include <list>

#include "MD_MAX72xx.h"
#include <ArduinoJson.h>
#include <WiFiManager.h> 
#include <PubSubClient.h>
#include <Bounce2.h>
#include <apds9301.h>
#include <Adafruit_BME680.h>
#include <BH1750.h>
#include <BME280I2C.h>
#include <EnvironmentCalculations.h>

#define HAS_APDS9301  // Lux meter
#define HAS_BH1750    // Lux meter
#define HAS_BME280    // Env meter
#define HAS_BME680    // Env meter + gas

#define APP_NAME    "LedNTP"
#define APP_VERSION "0.3.5"

/* constants */
constexpr auto _SDA = 5;
constexpr auto _SCL = 4;

constexpr char tz[] = "TZ=CET-1CEST-2,M3.5.0/02:00:00,M10.5.0/03:00:00";

constexpr auto BUTTON = 12;

/* forward reference */
void mqtt_subscribe(char*, unsigned char*, unsigned int);

/* global variables */
MD_MAX72XX leds(SS, 5);
Ticker ticker1;
Bounce bouncy;
int display_mode;
int display_time;
bool show_time;

//define your default values here, if there are different values in config.json, they are overwritten.
String mqtt_server("rp1.ch.polyware.nl");
String mqtt_port("1883");
bool shouldSaveConfig;

uint32_t press_time;
uint32_t delay_next;
uint32_t delay_next_sensors;
#if defined(HAS_BME280) || defined(HAS_BME680)
enum {
    ENV_UNKNOWN,
    ENV_BME280,
    ENV_BME680,
} bme;
#endif
#if defined(HAS_APDS9301) || defined(HAS_BH1750)
enum {
    LUX_UNKNOWN,
    LUX_APDS9301,
    LUX_BH1750,
} lux;
#endif
#ifdef HAS_APDS9301
LuxI2C_APDS9301 apds(0x39);
#endif
#ifdef HAS_BH1750
BH1750 bh;
#endif
#ifdef HAS_BME280
BME280I2C bme280(BME280I2C::Settings{
    /*tosr=*/BME280::OSR_X1,
    /*hosr=*/BME280::OSR_X1,
    /*posr=*/BME280::OSR_X1,
    /*mode=*/BME280::Mode_Forced,
    /*st=*/BME280::StandbyTime_1000ms,
    /*filter=*/BME280::Filter_Off,
    /*spiEnable=*/BME280::SpiEnable_False,
    /*bme_280_addr=*/0x76});
#endif
#ifdef HAS_BME680
Adafruit_BME680 bme680;
#endif
String mqtt_prefix;
WiFiClient wificlient;
PubSubClient mqtt("", 0, mqtt_subscribe, wificlient);

// Setup ESP to measure VCC
ADC_MODE(ADC_VCC);

extern "C" {
extern const unsigned char data_index_html_gz[];
extern const unsigned int data_index_html_gz_len;
}

void mqtt_subscribe(char* topic, unsigned char* payload, unsigned int len) {
    Serial.print("MQTT Subscribe(");
    Serial.print(topic);
    Serial.print(",");
    Serial.write(payload, len);
    Serial.println(")");
}

void setString(const char* msg) {
    int offset = leds.getColumnCount()-1;
    leds.control(MD_MAX72XX::UPDATE, MD_MAX72XX::OFF);
    leds.clear();
    while (char ch = *msg++) {
        offset -= leds.setChar(offset, ch) + 1;
    }
    leds.control(MD_MAX72XX::UPDATE, MD_MAX72XX::ON);
}

void setString(float val, const char suffix[]) {
    String s(val, 0);
    s += suffix;

    int offset = -2;
    uint8_t buf[8];
    for (const auto &ch : s) {
        offset += leds.getChar(ch, sizeof(buf), buf) + 1;
    }
    for (const auto &ch : s) {
        offset -= leds.setChar(offset, ch) + 1;
    }
}

struct {
    float volt;
    float lux;
    float temp;
    float hum;
    float dew;
    float pres;
    float gas_r;
} sensors;

void showTime(void) {
    time_t now = time(nullptr);
    if (now == 0)
        return;
    struct tm *tm = localtime(&now);

    show_time = true;

    leds.control(MD_MAX72XX::UPDATE, MD_MAX72XX::OFF);
    leds.clear();
    switch (display_mode) {
        case 4: // Temp
            setString(sensors.temp, "\775C");
            if (--display_time == 0) {
                display_mode--;
                display_time = 5;
            }
            break;
        case 3: // RH
            setString(sensors.hum, " RH");
            if (--display_time == 0) {
                display_mode--;
                display_time = 5;
            }
            break;
        case 2: // hPA
            setString(sensors.pres, " hPa");
            if (--display_time == 0) {
                display_mode--;
                display_time = 5;
            }
            break;
        case 1: // Lux
            setString(sensors.lux, " lx");
            if (--display_time == 0) {
                display_mode--;
                display_time = 5;
            }
            break;
        case 0:
            leds.setChar(38, '0'+(tm->tm_hour/10)%10);
            leds.setChar(32, '0'+(tm->tm_hour/ 1)%10);
            leds.setChar(26, ':');
            leds.setChar(24, '0'+(tm->tm_min/10)%10);
            leds.setChar(18, '0'+(tm->tm_min/ 1)%10);
            leds.setChar(12, ':');
            leds.setChar(10, '0'+(tm->tm_sec/10)%10);
            leds.setChar( 4, '0'+(tm->tm_sec/ 1)%10);
            break;
    }
    leds.control(MD_MAX72XX::UPDATE, MD_MAX72XX::ON);
}

void Sensors() {
#if MQTT_MAX_PACKET_SIZE < 256
#error "Please define MQTT_MAX_PACKET_SIZE larger or equal to 256."
#endif
    StaticJsonBuffer<200> jsonBuffer;
    JsonObject& root = jsonBuffer.createObject();
    root["id"] = String(ESP.getChipId(), HEX);

    sensors.volt = ESP.getVcc()/1024.0;

#if defined(HAS_APDS9301) || defined(HAS_BH1750)
    switch (lux) {
        case LUX_UNKNOWN:
            sensors.lux = NAN;
            break;
#ifdef HAS_APDS9301
        case LUX_APDS9301:
            sensors.lux = apds.getLux();
            break;
#endif
#ifdef HAS_BH1750
        case LUX_BH1750:
            sensors.lux = bh.readLightLevel();
            break;
#endif
    }
#endif

#if defined(HAS_BME280) || defined(HAS_BME680)
    switch (bme) {
#ifdef HAS_BME280
        case ENV_BME280: {
                    bme280.read(sensors.pres, sensors.temp, sensors.hum, BME280::TempUnit_Celcius, BME280::PresUnit_hPa);
                    sensors.gas_r = NAN;
                }
                break;
#endif
#ifdef HAS_BME680
        case ENV_BME680: {
                    bme680.performReading();
                    sensors.temp = bme680.temperature;
                    sensors.hum = bme680.humidity;
                    sensors.pres = bme680.pressure / 100.0;
                    sensors.gas_r = bme680.gas_resistance;
                }
#endif
    }
    sensors.dew = EnvironmentCalculations::DewPoint(sensors.temp, sensors.hum, /*metric=*/true);
#endif

#if defined(HAS_APDS9301) || defined(HAS_BH1750)
    if (!isnan(sensors.lux)) {
        root["lux"] = sensors.lux;
        if (!mqtt.publish((mqtt_prefix+"/lux").c_str(), String(sensors.lux).c_str())) {
            Serial.println("Publish lux failed");
        }
    }
#endif
#if defined(HAS_BME280) || defined(HAS_BME680)
    root["temperature"] = sensors.temp;
    if (!mqtt.publish((mqtt_prefix+"/temperature").c_str(), String(sensors.temp).c_str())) {
        Serial.println("Publish temperature failed");
    }
    root["humidity"] = sensors.hum;
    if (!mqtt.publish((mqtt_prefix+"/humidity").c_str(), String(sensors.hum).c_str())) {
        Serial.println("Publish humidity failed");
    }
    root["dewpoint"] = sensors.dew;
    if (!mqtt.publish((mqtt_prefix+"/dewpoint").c_str(), String(sensors.dew).c_str())) {
        Serial.println("Publish dewpoint failed");
    }
    root["pressure"] = sensors.pres;
    if (!mqtt.publish((mqtt_prefix+"/pressure").c_str(), String(sensors.pres).c_str())) {
        Serial.println("Publish pressure failed");
    }
#endif
#ifdef HAS_BME680
    if (!isnan(sensors.gas_r) && sensors.gas_r > 1.0) {
        root["gas_resistance"] = sensors.gas_r;
        if (!mqtt.publish((mqtt_prefix+"/gas_resistance").c_str(), String(sensors.gas_r).c_str())) {
            Serial.println("Publish gas_resistance failed");
        }
    }
#endif
    root["voltage"] = sensors.volt;
    if (!mqtt.publish((mqtt_prefix+"/voltage").c_str(), String(sensors.volt).c_str())) {
        Serial.println("Publish voltage failed");
    }
    root["reset"] = ESP.getResetReason();
    if (!mqtt.publish((mqtt_prefix+"/reset_reason").c_str(), ESP.getResetReason().c_str())) {
        Serial.println("Publish reset failed");
    }
    String json;
    root.printTo(json);
    if (!mqtt.publish((mqtt_prefix+"/json").c_str(), json.c_str())) {
        Serial.println("Publish json failed");
    }
    Serial.print("json="); Serial.println(json);
}

bool mqtt_setup() {
    Serial.println("Attempting MQTT connection...");
    // Attempt to connect
    setString("Mqtt...");
    if (mqtt.connect(WiFi.hostname().c_str(), (mqtt_prefix+"/status").c_str(), MQTTQOS0, true, "died")) {
        Serial.println("Connected to MQTT broker");
        // Once connected, publish an announcement...
        if (!mqtt.publish((mqtt_prefix+"/status").c_str(), "online", true)) {
            Serial.println("Publish-hello failed");
        }
        setString("Mqtt OK");
        return true;
    }
    Serial.print("failed, rc=");
    Serial.println(mqtt.state());
    return false;
}

bool wifi_setup() {
    //read configuration from FS json
    Serial.println("mounting FS...");
    if (SPIFFS.begin()) {
        Serial.println("mounted file system");
        if (SPIFFS.exists("/config.json")) {
            //file exists, reading and loading
            Serial.println("reading config file");
            File configFile = SPIFFS.open("/config.json", "r");
            if (configFile) {
                Serial.println("opened config file");
                DynamicJsonBuffer jsonBuffer;
                JsonObject& json = jsonBuffer.parseObject(configFile);
                json.printTo(Serial);
                if (json.success()) {
                    Serial.println("\nparsed json");

                    mqtt_server = json["mqtt_server"].as<String>();
                    mqtt_port = json["mqtt_port"].as<String>();
                } else {
                    Serial.println("failed to load json config");
                }
            }
        }
    } else {
        Serial.println("failed to mount FS");
    }

    // The extra parameters to be configured (can be either global or just in the setup)
    // After connecting, parameter.getValue() will get you the configured value
    // id/name placeholder/prompt default length
    WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server.c_str(), 40);
    WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port.c_str(), 5);

    setString("Wifi...");
    WiFiManager wifiManager;
    wifiManager.setAPCallback([](WiFiManager *wifi) {
        setString("Wifi AP");
    });
    // Set config save notify callback
    shouldSaveConfig = false;
    wifiManager.setSaveConfigCallback([]() {
            shouldSaveConfig = true;
    });

    // Add all your parameters here
    wifiManager.addParameter(&custom_mqtt_server);
    wifiManager.addParameter(&custom_mqtt_port);

    ticker1.detach();
    auto res = wifiManager.autoConnect();
    ticker1.attach_ms(1000, showTime);
    if (res) {
        setString("Wifi OK");
    } else {
        setString("Wifi BAD");
        Serial.println("failed to connect or hit timeout");
        delay(3000);
        // Reset and try again, or maybe put it to deep sleep
        ESP.reset();
        delay(5000);
    }

    // Read updated parameters
    mqtt_server = custom_mqtt_server.getValue();
    mqtt_port = custom_mqtt_port.getValue();
    mqtt.setServer(mqtt_server.c_str(), mqtt_port.toInt());

    // Save the custom parameters to FS
    if (shouldSaveConfig) {
        Serial.println("saving config");
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.createObject();
        json["mqtt_server"] = mqtt_server;
        json["mqtt_port"] = mqtt_port;

        File configFile = SPIFFS.open("/config.json", "w");
        if (!configFile) {
            Serial.println("failed to open config file for writing");
        }

        json.printTo(Serial);
        json.printTo(configFile);
        configFile.close();
    }
    SPIFFS.end();

    return res;
}

void setup() {
    mqtt_prefix = "esp8266/" + String(ESP.getChipId(), HEX);

    Serial.begin(76800);

    leds.begin();
    leds.control(MD_MAX72XX::INTENSITY, MAX_INTENSITY/6);

    putenv(const_cast<char *>(tz));
    tzset();
    configTime(0, 0, "0.centos.pool.ntp.org", "1.centos.pool.ntp.org", "2.centos.pool.ntp.org");

    // Start I2C bus on our own pins.
    Wire.begin(_SDA, _SCL);
    //Wire.setClock(100000);

//===========
    Serial.print("chip_id="); Serial.println(ESP.getChipId(), HEX);
    Serial.print("flash_id="); Serial.println(ESP.getFlashChipId(), HEX);
    Serial.print("flash_size="); Serial.println(ESP.getFlashChipRealSize());
    Serial.print("mac="); Serial.println(WiFi.macAddress());
//===========

    Serial.println("Scanning i2c bus...");
    uint8_t data = 0;
    for (uint8_t addr = 1; addr < 128; addr++) {
        auto rc = twi_writeTo(addr, &data, /*len=*/0, /*send_stop=*/true);
        if (rc == 0 ) {
            switch (addr) {
#ifdef HAS_APDS9301
                case 0x29:
                case 0x39:
                case 0x49:
                    lux = LUX_APDS9301;
                    apds.powerOn();
                    apds.setADCGain(LuxI2C_APDS9301::low_gain);
                    // apds.setIntegrationTime(LuxI2C_APDS9301::low_time);
                    // apds.setIntegrationTime(LuxI2C_APDS9301::medium_time);
                    apds.setIntegrationTime(LuxI2C_APDS9301::high_time);
                    Serial.printf("APDS-9301: part=%02x, rev=%02x\n", apds.getPartNum(), apds.getRevNum());
                    break;
#endif
#ifdef HAS_BH1750
                case 0x23:
                case 0x5C:
                    lux = LUX_BH1750;
                    bh.begin();
                    Serial.println("BH1750FVI");
                    break;
#endif
#if defined(HAS_BME280) || defined(HAS_BME680)
                case 0x76:
                case 0x77:
                    if (!bme280.begin()) {
                        if (!bme680.begin(addr)) {
                            delay(5);
                            if (!bme280.begin()) {
                                if (!bme680.begin(addr)) {
                                    Serial.println("Unable to locate either BME280 or BME680");
                                }
                            }
                        } else {
                            Serial.println("BME680");
                            bme = ENV_BME680;
                        }
                    } else {
                        Serial.printf("BME280: chipid=%02x\n", bme280.chipID());
                        bme = ENV_BME280;
                    }
                    break;
#endif
            }
        }
        delay(1);
    }
//===========

    ArduinoOTA.onStart([]() {
        ticker1.detach();
        Serial.println("OTA Start");
        setString("OTA");
    });
    ArduinoOTA.onEnd([]() {
        Serial.println("\nOTA End");
        setString("OTA OK");
        // Reboot will restart the timer for us.
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
        setString("OTA...");
    });
    ArduinoOTA.onError([](ota_error_t error) {
        ticker1.attach_ms(1000, showTime);
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
        setString("OTA error");
    });
    ArduinoOTA.setPassword("NoWay");
    //ArduinoOTA.setHostname(WiFi.hostname().c_str());
    ArduinoOTA.begin();

    ticker1.attach_ms(1000, showTime);

    // Explicitly set button to input even though it already should be.
    pinMode(BUTTON, INPUT);
    // After setting up the button, setup the Bounce instance
    bouncy.attach(BUTTON);
    bouncy.interval(5);
}

void loop() {
    ArduinoOTA.handle();
    bouncy.update();

    if (show_time) {
        show_time = false;

        time_t now = time(nullptr);
        Serial.print(ctime(&now));
    }

    auto now = millis();
    // Try to connect to network every 30 seconds.
    if (WiFi.status() != WL_CONNECTED) {
        if (delay_next < now) {
            if (!wifi_setup()) {
                // no network? retry in 30s.
                delay_next = now + 30000;
                return;
            }
            delay_next = now;
        }
        return;
    }

    // Now we have network, try mqtt every 10 seconds.
    if (mqtt_server.length()) {
        if (!mqtt.loop()) {
            if (delay_next < now) {
                if (!mqtt_setup()) {
                     // no mqtt? retry in 10s.
                     delay_next = now + 10000;
                     return;
                }
                delay_next = now;
            }
            return;
        }

        // Tick-tock on the state machine
        if (delay_next_sensors < now) {
            delay_next_sensors = now + 5*60*1000;

            Sensors();
        }
    }

    if (bouncy.rose()) {
        press_time = now;
    } else if (bouncy.fell()) {
        press_time = now - press_time;
        Serial.printf("Button pressed %dms ago\n", press_time);
        if (press_time > 2000) {
            Serial.println("Disconnecting WiFi");
            WiFi.disconnect();
            return;
        }
        display_mode = 4;
        display_time = 5;
    }

    // Process serial port bytes
    if (Serial.available()) {
        switch (Serial.read()) {
         case ' ':
            {
                Serial.println("Scanning...");
                uint8_t data = 0;
                for (uint8_t addr = 1; addr < 128; addr++) {
                    auto rc = twi_writeTo(addr, &data, /*len=*/0, /*send_stop=*/true);
                    if (rc == 0 ) {
                        Serial.print("addr: ");
                        Serial.println(addr, HEX);
                    }
                    delay(1);
                }
                Serial.println();
            }
            break;
        }
    }
}
