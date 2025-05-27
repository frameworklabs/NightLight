// Project: NightLight
// Copyright: (c) 2025 Framework Labs

#include "WeatherSymbols.h"

#include <proto_activities.h>
#include <pa_ard_utils.h>

#include <M5Unified.h>
#include <ArduinoJson.hpp>

#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Preferences.h>

#include <esp_pthread.h>
#include <pthread.h>

using namespace proto_activities::ard_utils;

// Helpers

pa_activity (DelayS, pa_ctx_tm(), int s) {
    pa_delay_s (s);
} pa_end

pa_activity (RaisingEdgeDetector, pa_ctx(bool prevVal), bool val, bool& edge) {
    pa_self.prevVal = val;
    edge = val;
    pa_pause;

    pa_always {
        if (val != pa_self.prevVal) {
            pa_self.prevVal = val;
            edge = val;
        } else {
            edge = false;
        }
    } pa_always_end
} pa_end

// Screen

class Dpy {
public:
    void init(uint8_t brightness = 10) {
        M5.Lcd.setRotation(2);
        dpy_.createSprite(128, 128);
    }

    M5Canvas& operator()() {
        return dpy_;
    }

    void setNeedsDisplay() {
        needsDisplay_ = true;
    }

    void displayIfNeeded() {
        if (needsDisplay_) {
            dpy_.pushSprite(0, 0);
            needsDisplay_ = false;
        }
    }

private:
    M5Canvas dpy_{&M5.Lcd};
    bool needsDisplay_ = false;
};

static Dpy dpy;

pa_activity (DisplayUpdater, pa_ctx()) {
    pa_always {
        dpy.displayIfNeeded();
    } pa_always_end
} pa_end

pa_activity (ScreenWakeup, pa_ctx()) {
    M5.Display.wakeup();
    pa_pause; // This is not really needed.
} pa_end

// Input Receiver

static void emitPressIfSet(PressSignal& press, uint8_t val) {
    switch (val) {
        case 1: pa_emit_val(press, Press::short_press); break;
        case 2: pa_emit_val(press, Press::double_press); break;
        case 3: pa_emit_val(press, Press::long_press); break;
        default: break;
    }
}

pa_activity (InputReceiver, pa_ctx(), PressSignal& press, PressSignal& up, PressSignal& down) {
    Serial1.begin(9600, SERIAL_8N1, 2, 1);

    pa_every (Serial1.available() > 0) {
        uint8_t val = Serial1.read();

        // Skip old data
        while (Serial1.available() > 0) {
            val = Serial1.read();
        }

        if (val != 0) {
            Serial.printf("Received: %d\n", val);
        }
        
        // Emit presses
        emitPressIfSet(press, val & 0b00000011);
        emitPressIfSet(up, (val & 0b0001100) >> 2);
        emitPressIfSet(down, (val & 0b0110000) >> 4);
    } pa_every_end
} pa_end

// WIFI

static void disconnectWifi() {
    WiFi.disconnect(true);  // Disconnect wifi
    WiFi.mode(WIFI_OFF);  // Set the wifi mode to off
}

pa_activity (WiFiConnectorAux, pa_ctx(), bool& connected) {
    //Serial.println("WiFiConnector start...");
    WiFi.setHostname("hut");
    WiFi.begin("", "");
    connected = false;
    pa_await (WiFi.isConnected());
    connected = true;
    //Serial.println("WiFiConnector start...done");
} pa_end

pa_activity (WiFiConnector, pa_ctx_tm(pa_use(WiFiConnectorAux); bool connected)) {
    pa_repeat {
        pa_after_s_abort (5, WiFiConnectorAux, pa_self.connected);

        if (pa_self.connected) {
            break;
        }

        disconnectWifi();
        pa_delay_s (5);
    }
} pa_end

pa_activity (WiFiConnectionMaintainer, pa_ctx_tm(pa_use(WiFiConnector))) {
    pa_every (!WiFi.isConnected()) {
        pa_delay_s (1); // Wait 1 second before reconnecting
        pa_run (WiFiConnector);
    } pa_every_end
} pa_end

// NTP

pa_activity (NTPEstablisher, pa_ctx_tm()) {
    //Serial.println("Config Time...");

    // TODO: use https://timeapi.io/swagger/index.html instead
    //configTime(3600, 0, "time.ovgu.de"); // winter time
    configTime(3600, 3600, "time.ovgu.de"); // summer time

    //Serial.println("Getting local time..."); //Serial.flush();
    struct tm timeinfo;
    while (!getLocalTime(&timeinfo, 100)) {
        pa_delay_s (1);
    }
    //Serial.println("Getting local time...done");  
} pa_end

pa_activity (WiFiAndNTPConnector, pa_ctx(pa_use(WiFiConnector); pa_use(NTPEstablisher))) {
    pa_run (WiFiConnector);
    pa_run (NTPEstablisher);
} pa_end

// Alarm

struct BaseAlarm {
    uint8_t enabled = false;
    uint8_t hour = 7;
    uint8_t minute = 0;
};

struct Alarm : BaseAlarm {
    bool dirty = false;
};

class Prefs {
public:
    void init() {
        preferences_.begin("Hut");
    }

    Alarm readAlarm() {
        if (!hasLoadedAlarm_) {
            preferences_.getBytes("Alarm", &loadedAlarm_, sizeof(BaseAlarm));
            hasLoadedAlarm_ = true;
        }
        return loadedAlarm_;    
    }  

    void writeAlarm(const Alarm& alarm) {
        loadedAlarm_ = alarm;
        hasLoadedAlarm_ = true;
        isAlarmDirty_ = true;
    }

    void writeAlarmIfNeeded() {
        if (isAlarmDirty_) {
            preferences_.putBytes("Alarm", &loadedAlarm_, sizeof(BaseAlarm));
            isAlarmDirty_ = false;
        }
    }

    bool readIsAnalogClock() {
        if (!hasLoadedIsAnalogClock_) {
            loadedIsAnalogClock_ = preferences_.getBool("ClockType");
            hasLoadedIsAnalogClock_ = true;
        }
        return loadedIsAnalogClock_;
    }

    void writeIsAnalogClock(bool isAnalog) {
        if (!hasLoadedIsAnalogClock_ || isAnalog != loadedIsAnalogClock_) {
            loadedIsAnalogClock_ = isAnalog;
            hasLoadedIsAnalogClock_ = true;
            preferences_.putBool("ClockType", isAnalog);
        }
    }
  
private:
    Preferences preferences_;
    Alarm loadedAlarm_;
    bool hasLoadedAlarm_ = false;
    bool loadedIsAnalogClock_ = false;
    bool hasLoadedIsAnalogClock_ = false;
    bool isAlarmDirty_ = false;
};

static Prefs prefs;

// Wait screen

static constexpr auto ARC_LEN = 180.0 / 4.0;
static constexpr auto ARC_INC = 180.0 / 10.0;

pa_activity (WaitScreen, pa_ctx(float angle; bool color)) {
    pa_always {
        dpy().clear();

        dpy().fillArc(64, 64, 20, 30, pa_self.angle, pa_self.angle + ARC_LEN, pa_self.color ? TFT_ORANGE : TFT_GREEN);

        pa_self.angle += ARC_INC;
        if (pa_self.angle >= 360.0) {
            pa_self.angle -= 360.0;
            pa_self.color = !pa_self.color;
        }

        dpy.setNeedsDisplay();
    } pa_always_end
} pa_end

// Clock Screen

static void renderDigitalClock(struct tm& timeinfo) {
    dpy().clear();

    dpy().setCursor(0, 20);
    dpy().setFont(&fonts::Roboto_Thin_24);
    dpy().setTextColor(TFT_PURPLE);
    dpy().println(&timeinfo, "%F");  

    dpy().setCursor(15, 70);
    dpy().setFont(&fonts::Roboto_Thin_24);
    dpy().setTextColor(TFT_GOLD);
    dpy().println(&timeinfo, "%T");

    dpy.setNeedsDisplay();
}

static float stophi(int s) {
    return PI * (15 - s) / 30;
}

static void drawArmLine(int offx, int offy, int len, float co, float si) {
    dpy().drawLine(64 + offx, 64 - offy, 64 + offx + len * co, 64 - offy - len * si);
}

static void drawArm(int s, int len, int color, int thick) {
    const auto phi = stophi(s);
    const auto co = cosf(phi);
    const auto si = sinf(phi);

    dpy().setColor(color);
    drawArmLine(0, 0, len, co, si);
}

static void drawTick(int s, int len, int color, int thick) {
    const auto phi = stophi(s);
    const auto co = cosf(phi);
    const auto si = sinf(phi);
    dpy().drawLine(64 + len * co, 64 - len * si, 64 + 63 * co, 64 - 63 * si, color);
}

static void renderAnalogClock(struct tm& timeinfo) {
    dpy().clear();

    dpy().setCursor(90, 54);
    dpy().setFont(&fonts::Roboto_Thin_24);
    dpy().setTextColor(TFT_DARKGRAY);
    dpy().println(&timeinfo, "%d");
    
    dpy().drawCircle(64, 64, 63, TFT_WHITE);
    
    for (int s = 0; s < 60; s += 5) {
        drawTick(s, 59, TFT_WHITE, 1);
    }

    drawArm(timeinfo.tm_hour * 5 + timeinfo.tm_min / 12, 40, TFT_WHITE, 3);
    drawArm(timeinfo.tm_min, 58, TFT_BLUE, 2);
    drawArm(timeinfo.tm_sec, 62, TFT_RED, 1);

    dpy.setNeedsDisplay();
}

pa_activity (ClockScreen, pa_ctx_tm(pa_use(ScreenWakeup)), bool analog) {
    pa_run (ScreenWakeup);

    pa_every_s (1) {
        struct tm timeinfo;
        if (!getLocalTime(&timeinfo, 50)) {
            dpy().clear();
            dpy().println("Failed to obtain time");
            dpy.setNeedsDisplay();
        } 
        else {
            if (analog) {
                renderAnalogClock(timeinfo);
            } else {
                renderDigitalClock(timeinfo);
            }
        }
    } pa_every_end
} pa_end

pa_activity (ClockScreenController, pa_ctx(pa_use(ClockScreen)), const PressSignal& press) {
    pa_repeat {
        if (prefs.readIsAnalogClock()) {
            pa_when_abort (press && press.val() == Press::double_press, ClockScreen, true);
            prefs.writeIsAnalogClock(false);
        }
        pa_when_abort (press && press.val() == Press::double_press, ClockScreen, false);
        prefs.writeIsAnalogClock(true);
    }
} pa_end

// Settings Screen

static uint8_t caw(uint8_t val, int delta, uint8_t limit) {
    int newVal = val + delta;
    if (newVal < 0) {
        return limit + newVal;
    }
    if (newVal >= limit) {
        return newVal - limit;
    }
    return newVal;
}

pa_activity (SettingsController, pa_ctx(), const PressSignal& up, const PressSignal& down, Alarm& alarm) {
    pa_every (up || down) {
        if (up && down) {
            alarm.enabled = !alarm.enabled;
        } else {
            const PressSignal& press = up ? up : down;
            const int inc = up ? +1 : -1;

            if (press.val() == Press::short_press || press.val() == Press::long_press) {
                alarm.minute = caw(alarm.minute, inc, 60);
            } else if (press.val() == Press::double_press) {
                alarm.hour = caw(alarm.hour, inc, 24);
            }
        }

        alarm.dirty = true;
    } pa_every_end
} pa_end

pa_activity (SettingsPresenter, pa_ctx(), Alarm alarm) {
    pa_repeat {
        dpy().clear();

        dpy().setCursor(30, 50);
        dpy().setFont(&fonts::Roboto_Thin_24);

        if (alarm.enabled) {
            dpy().setTextColor(TFT_GREEN);
        } else {
            dpy().setTextColor(TFT_RED);
        }

        dpy().printf("%2d:%02d", alarm.hour, alarm.minute);

        dpy.setNeedsDisplay();

        pa_await (alarm.dirty);
    }
} pa_end

pa_activity (SettingsPersister, pa_ctx(), Alarm& alarm) {
    pa_every (alarm.dirty) {
        prefs.writeAlarm(alarm);
        alarm.dirty = false;
    } pa_every_end
} pa_end

pa_activity (SettingsTimeout, pa_ctx(pa_use(DelayS)), const PressSignal& up, const PressSignal& down) {
    pa_when_reset (up || down, DelayS, 4);
} pa_end

pa_activity (SettingsScreen, pa_ctx(pa_co_res(4); pa_defer_res;
                                    pa_use(ScreenWakeup); 
                                    pa_use(SettingsController); pa_use(SettingsTimeout); pa_use(SettingsPresenter);
                                    pa_use(SettingsPersister); Alarm alarm), 
                             const PressSignal& up, const PressSignal& down) {
    pa_defer {
        prefs.writeAlarmIfNeeded();
    };

    pa_run (ScreenWakeup);

    pa_self.alarm = prefs.readAlarm();

    pa_co(4) {
        pa_with_weak (SettingsController, up, down, pa_self.alarm);
        pa_with_weak (SettingsPresenter, pa_self.alarm);
        pa_with_weak (SettingsPersister, pa_self.alarm);
        pa_with (SettingsTimeout, up, down);
    } pa_co_end
} pa_end

// Weather Screen

struct WeatherData {
    bool isValid{};
    float curTemp{};
    float minTemp{};
    float maxTemp{};
    int weatherCode{};
    int maxPercipitationProb{};

    bool operator==(const WeatherData& other) const {
        return isValid == other.isValid 
            && curTemp == other.curTemp
            && minTemp == other.minTemp
            && maxTemp == other.maxTemp
            && weatherCode == other.weatherCode
            && maxPercipitationProb == other.maxPercipitationProb;      
    }

    bool operator!=(const WeatherData& other) const { return !(*this == other); }
};

class WeatherAccessor {
public:
    WeatherAccessor() {
    }

    ~WeatherAccessor() {
        stop();
    }

    void init() {
        if (didInit_) {
            return;
        }
        esp_pthread_init();

        esp_pthread_cfg_t cfg = esp_pthread_get_default_config();
        cfg.stack_size = 1024 * 8;
        cfg.prio = 1;
        cfg.pin_to_core = 1;
        esp_pthread_set_cfg(&cfg);
        didInit_ = true;
    }

    void start() {
        if (isRunning_) {
            return;
        }

        isRunning_ = true;
        isDone_ = false;
        weather_.isValid = false;

        //Serial.println("starting thread..."); //Serial.flush();
        init();
        pthread_create(&thread_, nullptr, staticRunner, this);
        //Serial.println("starting thread...done");
    }

    void stop() {
        if (thread_) {
            pthread_join(thread_, nullptr);
            thread_ = {};
        }
    }

    bool isDone() const {
        return isDone_;
    }

    const WeatherData& getWeather() const {
        assert (isDone_);
        return weather_;
    }

private:
    static void* staticRunner(void* self) {
        reinterpret_cast<WeatherAccessor*>(self)->runner();
        return nullptr;
    }

    void runner() {
        HTTPClient http;

        do {
            //Serial.println("http begin...");
            if (!http.begin("https://api.open-meteo.com/v1/forecast?latitude=48.1374&longitude=11.5755&current=temperature_2m&daily=temperature_2m_max,temperature_2m_min,weather_code,precipitation_probability_max&timezone=Europe%2FBerlin&forecast_days=1")) {
                //Serial.println("http begin failed");
                break;
            }
            //Serial.println("http get...");
            const auto code = http.GET();
            if (code != HTTP_CODE_OK) {
                //Serial.printf("http GET failed with: %d\n", code);
                break;
            }
            //Serial.println("printing payload...");
            const auto payload = http.getString();
            //Serial.println(payload);

            ArduinoJson::DynamicJsonDocument doc(1024);

            //Serial.println("json deserialize...");
            const auto res = ArduinoJson::deserializeJson(doc, payload);
            if (res.code() != ArduinoJson::DeserializationError::Ok) {
                //Serial.printf("json deserialization failed: %s\n", res.c_str());
                break;
            }

            weather_.maxTemp = doc["daily"]["temperature_2m_max"][0].as<float>();
            weather_.curTemp = doc["current"]["temperature_2m"].as<float>();
            weather_.minTemp = doc["daily"]["temperature_2m_min"][0].as<float>();
            weather_.weatherCode = doc["daily"]["weather_code"][0].as<int>();
            weather_.maxPercipitationProb = doc["daily"]["precipitation_probability_max"][0].as<int>();
            weather_.isValid = true;

        } while (false);
                
        //Serial.println("http end...");
        http.end();

        //Serial.println("http end...done");

        isDone_ = true;
        isRunning_ = false;
    }

private:
    bool didInit_{};
    std::atomic_bool isRunning_{};
    std::atomic_bool isDone_{};
    pthread_t thread_{};
    WeatherData weather_{};
};

static WeatherAccessor weatherAccessor;
static WeatherData cachedWeather;

pa_activity (WeatherProvider, pa_ctx_tm(int tries), WeatherData& weather) {
    pa_repeat {
        weather.isValid = false;
        pa_self.tries = 0;

        pa_repeat {
            ++pa_self.tries;

            weatherAccessor.start();
            pa_await (weatherAccessor.isDone());

            if (weatherAccessor.getWeather().isValid) {
                break;
            }
            if (pa_self.tries == 5) {
                break;
            }
            //Serial.println("Failed loading weather - retrying in 2 seconds");
            pa_delay_s (2); // retry every 2 seconds
        }

        if (weatherAccessor.getWeather().isValid) {
            cachedWeather = weatherAccessor.getWeather();
            weather = weatherAccessor.getWeather();

            //Serial.println("Succeeded retrieving weather - refreshing in 15 min");
            pa_delay_m (15); // update every 15 minutes in case of success
        } 
        else {
            weather = cachedWeather; // Better show old data than a spinner for 1 min.

            //Serial.println("Failed retrieving weather - retrying in 1 min");
            pa_delay_m (1); // retry every minute in case of error
        }
    }
} pa_end

pa_activity (TemperatureScreen, pa_ctx(WeatherData prevWeather), bool sigActivation, const WeatherData& weather) {
    pa_repeat {
        dpy().fillRect(0, 0, 128, 128, TFT_WHITE);

        dpy().setCursor(10, 15);
        dpy().setFont(&fonts::FreeSans12pt7b);
        dpy().setTextColor(TFT_RED);
        dpy().printf("max: % 2.1f", weather.maxTemp);

        dpy().setCursor(30, 50);
        dpy().setFont(&fonts::FreeSans18pt7b);
        dpy().setTextColor(TFT_BLACK);
        dpy().printf("% 2.1f", weather.curTemp);

        dpy().setCursor(10, 95);
        dpy().setFont(&fonts::FreeSans12pt7b);
        dpy().setTextColor(TFT_BLUE);
        dpy().printf("min: % 2.1f", weather.minTemp);

        dpy.setNeedsDisplay();

        pa_self.prevWeather = weather;
        pa_await (weather != pa_self.prevWeather || sigActivation);
    }
} pa_end

static void drawWeatherCode(int weatherCode) {
    size_t weatherSymbolSize;
    const uint8_t* weatherSymbol = getWeatherSymbol(weatherCode, weatherSymbolSize);

    dpy().drawPng(weatherSymbol, weatherSymbolSize, 36, 10);
}

pa_activity (PercipitationScreen, pa_ctx(WeatherData prevWeather), bool sigActivation, const WeatherData& weather) {
    pa_repeat {
        dpy().fillRect(0, 0, 128, 128, TFT_WHITE);

        drawWeatherCode(weather.weatherCode);

        dpy().setCursor(20, 70);
        dpy().setFont(&fonts::FreeSans9pt7b);
        dpy().setTextColor(TFT_BLACK);
        dpy().printf("Wetter: %d", weather.weatherCode);

        dpy().setCursor(3, 95);
        dpy().setFont(&fonts::FreeSans12pt7b);
        dpy().setTextColor(TFT_BLUE);
        dpy().printf("Rain: %3d%%", weather.maxPercipitationProb);

        dpy.setNeedsDisplay();

        pa_self.prevWeather = weather;
        pa_await (weather != pa_self.prevWeather || sigActivation);
    }
} pa_end

pa_activity (WeatherOrWaitScreenController, pa_ctx(pa_use(TemperatureScreen); pa_use(PercipitationScreen); pa_use(WaitScreen)), 
                                            bool sigActivation, const WeatherData& weather, const PressSignal& press) {
    pa_repeat {
        if (!weather.isValid) {
            pa_when_abort (weather.isValid, WaitScreen);
        }
        pa_when_abort (!weather.isValid || (press && press.val() == Press::double_press), TemperatureScreen, sigActivation, weather);
        if (weather.isValid) {
            pa_when_abort (!weather.isValid || (press && press.val() == Press::double_press), PercipitationScreen, sigActivation, weather);
        }
    }
} pa_end

pa_activity (WeatherScreenController, pa_ctx(pa_co_res(2); WeatherData weather;
                                             pa_use(WeatherProvider); pa_use(WeatherOrWaitScreenController)), 
                                      bool sigActivation, const PressSignal& press) {
    pa_co(2) {
        pa_with (WeatherProvider, pa_self.weather)
        pa_with (WeatherOrWaitScreenController, sigActivation, pa_self.weather, press);
    } pa_co_end
} pa_end

// Off Screen

pa_activity (OffScreenController, pa_ctx()) {
    dpy().clear();
    dpy.setNeedsDisplay();

    M5.Display.sleep();
    pa_halt;
} pa_end

// UI

pa_activity (GadgetScreenController, pa_ctx(pa_use(ClockScreenController); pa_use(WeatherScreenController)), bool sigActivation, bool isBuzzing, const PressSignal& press) {
    pa_repeat {
        pa_when_abort (press && press.val() == Press::long_press, ClockScreenController, press);
        pa_await_immediate (!press);
        pa_when_abort ((press && press.val() == Press::long_press) || isBuzzing, WeatherScreenController, sigActivation, press);
        pa_await_immediate (!press);
    }
} pa_end

pa_activity (SuspendingGadgetScreenController, pa_ctx(pa_use(GadgetScreenController)), bool isActive, bool sigActivation, bool isBuzzing, const PressSignal& press) {
    pa_when_suspend (!isActive, GadgetScreenController, sigActivation, isBuzzing, press);
} pa_end

pa_activity (SettingsScreenController, pa_ctx(pa_use(SettingsScreen)), const PressSignal& up, const PressSignal& down, bool isBuzzing, bool& showSettings) {
    pa_every (up || down) {
        showSettings = true;
        pa_when_abort (isBuzzing, SettingsScreen, up, down);
        showSettings = false;
    } pa_every_end
} pa_end

pa_activity (OnScreenController, pa_ctx(pa_co_res(3); pa_use(SettingsScreenController); 
                                        pa_use(SuspendingGadgetScreenController); pa_use(RaisingEdgeDetector);
                                        bool showSettings; bool sigActivation), 
                                 const PressSignal& press, const PressSignal& up, const PressSignal& down, bool isBuzzing) {
    pa_co(3) {
        pa_with (SettingsScreenController, up, down, isBuzzing, pa_self.showSettings);
        pa_with (RaisingEdgeDetector, !pa_self.showSettings, pa_self.sigActivation);
        pa_with (SuspendingGadgetScreenController, !pa_self.showSettings, pa_self.sigActivation, isBuzzing, press);
    } pa_co_end
} pa_end

pa_activity (UI, pa_ctx(pa_use(OnScreenController); pa_use(OffScreenController)), 
                 const PressSignal& press, const PressSignal& up, const PressSignal& down, bool isBuzzing) {
    pa_when_abort (press && press.val() == Press::short_press, OnScreenController, press, up, down, isBuzzing);

    pa_repeat {
        pa_when_abort (press || up || down || isBuzzing, OffScreenController);
    
        if (isBuzzing) {
            pa_when_abort ((press && press.val() == Press::short_press) | !isBuzzing, OnScreenController, press, up, down, isBuzzing);
        } else {
            pa_when_abort ((press && press.val() == Press::short_press), OnScreenController, press, up, down, isBuzzing);
        }
    }
} pa_end

// Buzzer

pa_activity (AlarmTimeChecker, pa_ctx(), bool& active) {
    pa_always {
        auto alarm = prefs.readAlarm();
        if (!alarm.enabled) {
            active = false;
        } else {
            struct tm time;
            if (!getLocalTime(&time, 50)) {
                active = false;
            } else {
                active = time.tm_hour == alarm.hour && time.tm_min == alarm.minute && time.tm_sec == 0;
            }
        }
    } pa_always_end
} pa_end

static const int buzzerChannel = 0;

static void initBuzzer() {
    pinMode(6, OUTPUT);
    digitalWrite(6, 0);

    const int spk_pin   = 5;
    int freq            = 50;
    int resolution      = 10;

    ledcSetup(buzzerChannel, freq, resolution);
    ledcAttachPin(spk_pin, buzzerChannel);
}

pa_activity (AudioRequestAccumulator, pa_ctx(), int& requests, int& totalRequests) {
    pa_always {
        totalRequests += requests;
        requests = 0;
    } pa_always_end
} pa_end

pa_activity (AudioRequestController, pa_ctx(), int totalRequests, bool& enabled) {
    pa_every (totalRequests > 0) {
        digitalWrite(6, 1);
        enabled = true; // will indicate availibility in next tick, as AudioManager runs last

        pa_await (totalRequests <= 0);
        digitalWrite(6, 0);
        enabled = false;
    } pa_every_end
} pa_end

pa_activity (AudioManager, pa_ctx(pa_co_res(2); pa_use(AudioRequestAccumulator); pa_use(AudioRequestController); int totalRequests), int& requests, bool& enabled) {
    pa_co(2) {
        pa_with (AudioRequestAccumulator, requests, pa_self.totalRequests);
        pa_with (AudioRequestController, pa_self.totalRequests, enabled);
    } pa_co_end
} pa_end

pa_activity (BuzzMaker, pa_ctx_tm(pa_defer_res; int i), bool withGap) {
    pa_defer {
        ledcWriteTone(buzzerChannel, 0);
    };
    pa_repeat {
        for (pa_self.i = 0; pa_self.i < 4; ++pa_self.i) {
            ledcWriteNote(buzzerChannel, NOTE_C, 4);
            pa_delay_ms (100);
            ledcWriteTone(buzzerChannel, 0);
            pa_delay_ms (100);
        }

        if (withGap) {
            pa_delay_ms (500);
        }
    }
} pa_end

pa_activity (BuzzGenerator, pa_ctx_tm(pa_use(BuzzMaker)), bool audioEnabled) {
    pa_await_immediate (audioEnabled);
    pa_after_s_abort (20, BuzzMaker, true);
    pa_delay_ms (300);
    pa_after_s_abort (10, BuzzMaker, false);
} pa_end

pa_activity (BuzzerController, pa_ctx(pa_use(BuzzGenerator)), 
                               bool active, const PressSignal& press, const PressSignal& up, const PressSignal& down, 
                               bool audioEnabled, int& audioRequests, bool& isBuzzing) {
    pa_every (active) {
        ++audioRequests;
        isBuzzing = true;
        pa_when_abort (press || up || down, BuzzGenerator, audioEnabled);
        isBuzzing = false;
        --audioRequests;
    } pa_every_end
} pa_end

pa_activity (Buzzer, pa_ctx(pa_co_res(2); pa_use(AlarmTimeChecker); pa_use(BuzzerController); bool active), 
                     const PressSignal& press, const PressSignal& up, const PressSignal& down, 
                     bool audioEnabled, int& audioRequests, bool& isBuzzing) {
    pa_co(2) {
        pa_with (AlarmTimeChecker, pa_self.active);
        pa_with (BuzzerController, pa_self.active, press, up, down, audioEnabled, audioRequests, isBuzzing);
    } pa_co_end
} pa_end

// Press Tone Generator

pa_activity (ShortToneMaker, pa_ctx_tm(pa_defer_res), bool audioEnabled) {
    pa_defer {
        ledcWriteTone(buzzerChannel, 0);
    };
    pa_await_immediate (audioEnabled);
    ledcWriteNote(buzzerChannel, NOTE_C, 4);
    pa_delay_ms (100);
} pa_end

pa_activity (DoubleToneMaker, pa_ctx_tm(pa_defer_res), bool audioEnabled) {
    pa_defer {
        ledcWriteTone(buzzerChannel, 0);
    };
    pa_await_immediate (audioEnabled);
    ledcWriteNote(buzzerChannel, NOTE_E, 4);
    pa_delay_ms (100);
    ledcWriteTone(buzzerChannel, 0);
    pa_delay_ms (100);
    ledcWriteNote(buzzerChannel, NOTE_E, 4);
    pa_delay_ms (100);
} pa_end

pa_activity (LongToneMaker, pa_ctx_tm(pa_defer_res), bool pressed, bool audioEnabled) {
    pa_defer {
        ledcWriteTone(buzzerChannel, 0);
    };
    pa_await_immediate (audioEnabled);
    ledcWriteNote(buzzerChannel, NOTE_G, 4);
    pa_await (!pressed); // Wait until the button is released
} pa_end

pa_activity (PressToneGenerator, pa_ctx_tm(pa_use(ShortToneMaker); pa_use(DoubleToneMaker); pa_use(LongToneMaker)), 
                                 const PressSignal& press, bool audioEnabled, int& audioRequests) {
    pa_repeat {
        pa_await_immediate (press);

        ++audioRequests;

        if (press.val() == Press::short_press) {
            pa_when_abort (press, ShortToneMaker, audioEnabled);
        } else if (press.val() == Press::double_press) {
            pa_when_abort (press, DoubleToneMaker, audioEnabled);
        } else if (press.val() == Press::long_press) {
            pa_when_abort (press && press.val() != Press::long_press, LongToneMaker, press, audioEnabled);
        }

        --audioRequests;
    }
} pa_end

// Main

pa_activity (Main, pa_ctx(pa_co_res(8); pa_signal_res;
                          pa_use(WiFiAndNTPConnector); pa_use(WiFiConnectionMaintainer); pa_use(PressRecognizer); 
                          pa_use(AudioManager); pa_use(PressToneGenerator);
                          pa_use(UI); pa_use(Buzzer); pa_use(DisplayUpdater); pa_use(WaitScreen); pa_use(InputReceiver);
                          pa_def_val_signal(Press, press); pa_def_val_signal(Press, up); pa_def_val_signal(Press, down);
                          bool isBuzzing; bool audioEnabled; int audioRequests),
                   bool didOverrun) {
    pa_co (3) {
        pa_with (WiFiAndNTPConnector);
        pa_with_weak (WaitScreen);
        pa_with_weak (DisplayUpdater);
    } pa_co_end

    pa_co(8) {
        pa_with (WiFiConnectionMaintainer);
        pa_with (PressRecognizer, 41, pa_self.press);
        pa_with (InputReceiver, pa_self.press, pa_self.up, pa_self.down);
        pa_with (PressToneGenerator, pa_self.press, pa_self.audioEnabled, pa_self.audioRequests);
        pa_with (Buzzer, pa_self.press, pa_self.up, pa_self.down, pa_self.audioEnabled, pa_self.audioRequests, pa_self.isBuzzing);
        pa_with (UI, pa_self.press, pa_self.up, pa_self.down, pa_self.isBuzzing);
        pa_with (AudioManager, pa_self.audioRequests, pa_self.audioEnabled);
        pa_with (DisplayUpdater);
    } pa_co_end
} pa_end

// Setup and Loop

static pa_use(Main);

void setup() {
    // Shut up the speaker as early as possible.
    initBuzzer();
    
    auto config = M5.config();
    M5.begin(config);

    dpy.init();
    prefs.init();
}

void loop() {
    TickType_t prevWakeTime = xTaskGetTickCount();
    bool wasDelayed = false;

    while (true) {
        M5.update();

        pa_tick(Main, !wasDelayed);

        // We run at 10 Hz.
        wasDelayed = xTaskDelayUntil(&prevWakeTime, 100);

        if (!wasDelayed) {
            //Serial.println("DID OVERRUN");
        }
    }
}
