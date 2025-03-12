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

pa_activity (Delay, pa_ctx_tm(), int ticks) {
    pa_delay (ticks);
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
    } pa_always_end;
} pa_end;

pa_activity (ScreenWakeup, pa_ctx()) {
    M5.Display.wakeup();
    pa_pause; // This is not really needed.
} pa_end;

// Input Receiver

static void emitPressIfSet(PressSignal& press, uint8_t val) {
    switch (val) {
        case 1: pa_emit_val(press, Press::short_press); break;
        case 2: pa_emit_val(press, Press::long_press); break;
        case 3: pa_emit_val(press, Press::double_press); break;
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
} pa_end;

pa_activity (WiFiConnector, pa_ctx(pa_co_res(2); pa_use(WiFiConnectorAux); pa_use(Delay); bool connected)) {
    pa_repeat {
        pa_co(2) {
            pa_with_weak (WiFiConnectorAux, pa_self.connected);
            pa_with_weak (Delay, 50);
        } pa_co_end;

        if (pa_self.connected) {
            break;
        }

        disconnectWifi();
        pa_run (Delay, 50);
    }
} pa_end;

pa_activity (WiFiConnectionMaintainer, pa_ctx(pa_use(WiFiConnector); pa_use(Delay))) {
    pa_every (!WiFi.isConnected()) {
        pa_run (Delay, 10); // Wait 1 second reconnecting
        pa_run (WiFiConnector);
    } pa_every_end;
} pa_end;

// NTP

pa_activity (NTPEstablisher, pa_ctx(pa_use(Delay))) {
    //Serial.println("Config Time...");

    // TODO: use https://timeapi.io/swagger/index.html instead
    configTime(3600, 0, "time.ovgu.de"); // winter time
    //configTime(3600, 3600, "time.ovgu.de"); // summer time

    //Serial.println("Getting local time..."); //Serial.flush();
    struct tm timeinfo;
    while (!getLocalTime(&timeinfo, 100)) {
        pa_run (Delay, 10);
    }
    //Serial.println("Getting local time...done");  
} pa_end;

pa_activity (WiFiAndNTPConnector, pa_ctx(pa_use(WiFiConnector); pa_use(NTPEstablisher))) {
    pa_run (WiFiConnector);
    pa_run (NTPEstablisher);
} pa_end;

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

constexpr auto ARC_LEN = 180.0 / 4.0;
constexpr auto ARC_INC = 180.0 / 10.0;

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
    } pa_always_end;
} pa_end;

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

pa_activity (ClockScreen, pa_ctx(pa_use(Delay); pa_use(ScreenWakeup)), bool analog) {
    pa_run (ScreenWakeup);

    pa_repeat {
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
        pa_run (Delay, 10);
    }
} pa_end;

pa_activity (ClockScreenController, pa_ctx(pa_use(ClockScreen)), const PressSignal& press) {
    pa_repeat {
        if (prefs.readIsAnalogClock()) {
            pa_when_abort (press && press.val() == Press::double_press, ClockScreen, true);
            prefs.writeIsAnalogClock(false);
        }
        pa_when_abort (press && press.val() == Press::double_press, ClockScreen, false);
        prefs.writeIsAnalogClock(true);
    }
} pa_end;

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
    } pa_every_end;
} pa_end;

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
} pa_end;

pa_activity (SettingsPersister, pa_ctx(), Alarm& alarm) {
    pa_every (alarm.dirty) {
        prefs.writeAlarm(alarm);
        alarm.dirty = false;
    } pa_every_end;
} pa_end;

pa_activity (SettingsTimeout, pa_ctx(pa_use(Delay)), const PressSignal& up, const PressSignal& down) {
    pa_when_reset (up || down, Delay, 40);
} pa_end;

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
    } pa_co_end;
} pa_end;

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

pa_activity (WeatherProvider, pa_ctx(pa_use(Delay); int tries), WeatherData& weather) {
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
            pa_run (Delay, 20); // retry every 2 seconds
        }

        if (weatherAccessor.getWeather().isValid) {
            cachedWeather = weatherAccessor.getWeather();
            weather = weatherAccessor.getWeather();

            //Serial.println("Succeeded retrieving weather - refreshing in 15 min");
            pa_run (Delay, 36000 / 4); // update every 15 minutes in case of success
        } 
        else {
            weather = cachedWeather; // Better show old data than a spinner for 1 min.

            //Serial.println("Failed retrieving weather - retrying in 1 min");
            pa_run (Delay, 600); // retry every minute in case of error
        }
    }
} pa_end;

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
} pa_end;

void drawWeatherCode(int weatherCode) {
    const uint8_t* weatherSymbol = nullptr;
    size_t weatherSymbolSize = 0;

    switch (weatherCode) {
        case 0: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_00; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_00); break;
        case 1: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_01; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_01); break;
        case 2: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_02; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_02); break;
        case 3: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_03; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_03); break;
        case 4: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_04; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_04); break;
        case 5: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_05; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_05); break;
        case 6: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_06; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_06); break;
        case 7: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_07; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_07); break;
        case 8: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_08; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_08); break;
        case 9: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_09; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_09); break;
        case 10: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_10; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_10); break;
        case 11: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_11; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_11); break;
        case 12: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_12; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_12); break;
        case 13: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_13; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_13); break;
        case 14: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_14; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_14); break;
        case 15: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_15; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_15); break;
        case 16: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_16; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_16); break;
        case 17: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_17; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_17); break;
        case 18: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_18; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_18); break;
        case 19: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_19; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_19); break;
        case 20: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_20; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_20); break;
        case 21: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_21; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_21); break;
        case 22: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_22; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_22); break;
        case 23: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_23; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_23); break;
        case 24: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_24; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_24); break;
        case 25: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_25; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_25); break;
        case 26: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_26; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_26); break;
        case 27: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_27; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_27); break;
        case 28: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_28; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_28); break;
        case 29: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_29; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_29); break;
        case 30: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_30; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_30); break;
        case 31: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_31; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_31); break;
        case 32: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_32; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_32); break;
        case 33: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_33; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_33); break;
        case 34: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_34; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_34); break;
        case 35: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_35; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_35); break;
        case 36: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_36; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_36); break;
        case 37: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_37; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_37); break;
        case 38: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_38; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_38); break;
        case 39: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_39; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_39); break;
        case 40: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_40; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_40); break;
        case 41: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_41; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_41); break;
        case 42: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_42; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_42); break;
        case 43: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_43; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_43); break;
        case 44: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_44; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_44); break;
        case 45: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_45; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_45); break;
        case 46: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_46; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_46); break;
        case 47: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_47; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_47); break;
        case 48: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_48; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_48); break;
        case 49: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_49; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_49); break;
        case 50: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_50; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_50); break;
        case 51: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_51; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_51); break;
        case 52: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_52; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_52); break;
        case 53: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_53; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_53); break;
        case 54: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_54; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_54); break;
        case 55: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_55; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_55); break;
        case 56: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_56; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_56); break;
        case 57: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_57; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_57); break;
        case 58: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_58; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_58); break;
        case 59: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_59; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_59); break;
        case 60: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_60; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_60); break;
        case 61: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_61; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_61); break;
        case 62: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_62; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_62); break;
        case 63: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_63; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_63); break;
        case 64: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_64; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_64); break;
        case 65: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_65; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_65); break;
        case 66: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_66; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_66); break;
        case 67: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_67; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_67); break;
        case 68: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_68; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_68); break;
        case 69: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_69; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_69); break;
        case 70: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_70; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_70); break;
        case 71: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_71; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_71); break;
        case 72: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_72; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_72); break;
        case 73: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_73; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_73); break;
        case 74: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_74; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_74); break;
        case 75: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_75; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_75); break;
        case 76: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_76; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_76); break;
        case 77: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_77; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_77); break;
        case 78: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_78; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_78); break;
        case 79: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_79; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_79); break;
        case 80: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_80; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_80); break;
        case 81: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_81; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_81); break;
        case 82: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_82; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_82); break;
        case 83: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_83; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_83); break;
        case 84: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_84; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_84); break;
        case 85: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_85; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_85); break;
        case 86: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_86; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_86); break;
        case 87: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_87; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_87); break;
        case 88: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_88; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_88); break;
        case 89: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_89; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_89); break;
        case 90: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_90; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_90); break;
        case 91: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_91; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_91); break;
        case 92: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_92; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_92); break;
        case 93: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_93; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_93); break;
        case 94: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_94; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_94); break;
        case 95: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_95; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_95); break;
        case 96: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_96; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_96); break;
        case 97: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_97; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_97); break;
        case 98: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_98; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_98); break;
        case 99: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_99; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_99); break;
        default: weatherSymbol = WeatherSymbol_WMO_PresentWeather_ww_00; weatherSymbolSize = sizeof(WeatherSymbol_WMO_PresentWeather_ww_00); break;
    }

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
} pa_end;

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
} pa_end;

pa_activity (WeatherScreenController, pa_ctx(pa_co_res(2); WeatherData weather;
                                             pa_use(WeatherProvider); pa_use(WeatherOrWaitScreenController)), 
                                      bool sigActivation, const PressSignal& press) {
    pa_co(2) {
        pa_with (WeatherProvider, pa_self.weather)
        pa_with (WeatherOrWaitScreenController, sigActivation, pa_self.weather, press);
    } pa_co_end;
} pa_end;

// Off Screen

pa_activity (OffScreenController, pa_ctx()) {
    dpy().clear();
    dpy.setNeedsDisplay();

    M5.Display.sleep();
    pa_halt;
} pa_end;

// UI

pa_activity (GadgetScreenController, pa_ctx(pa_use(ClockScreenController); pa_use(WeatherScreenController)), bool sigActivation, bool isBuzzing, const PressSignal& press) {
    pa_repeat {
        pa_when_abort (press && press.val() == Press::long_press, ClockScreenController, press);
        pa_await_immediate (!press);
        pa_when_abort ((press && press.val() == Press::long_press) || isBuzzing, WeatherScreenController, sigActivation, press);
        pa_await_immediate (!press);
    }
} pa_end;

pa_activity (SuspendingGadgetScreenController, pa_ctx(pa_use(GadgetScreenController)), bool isActive, bool sigActivation, bool isBuzzing, const PressSignal& press) {
    pa_when_suspend (!isActive, GadgetScreenController, sigActivation, isBuzzing, press);
} pa_end;

pa_activity (SettingsScreenController, pa_ctx(pa_use(SettingsScreen)), const PressSignal& up, const PressSignal& down, bool isBuzzing, bool& showSettings) {
    pa_every (up || down) {
        showSettings = true;
        pa_when_abort (isBuzzing, SettingsScreen, up, down);
        showSettings = false;
    } pa_every_end;
} pa_end;

pa_activity (OnScreenController, pa_ctx(pa_co_res(4); pa_use(SettingsScreenController); 
                                        pa_use(SuspendingGadgetScreenController); pa_use(RaisingEdgeDetector);
                                        bool showSettings; bool sigActivation), 
                                 const PressSignal& press, const PressSignal& up, const PressSignal& down, bool isBuzzing) {
    pa_co(3) {
        pa_with (SettingsScreenController, up, down, isBuzzing, pa_self.showSettings);
        pa_with (RaisingEdgeDetector, !pa_self.showSettings, pa_self.sigActivation);
        pa_with (SuspendingGadgetScreenController, !pa_self.showSettings, pa_self.sigActivation, isBuzzing, press);
    } pa_co_end;
} pa_end;

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
} pa_end;

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
    } pa_always_end;
} pa_end;

static const int buzzerChannel = 0;

static void enableSpeaker() {
    digitalWrite(6, 1);
}

static void disableSpeaker() {
    digitalWrite(6, 0);
}

static void initBuzzer() {
    pinMode(6, OUTPUT);
    disableSpeaker();

    const int spk_pin   = 5;
    int freq            = 50;
    int resolution      = 10;

    ledcSetup(buzzerChannel, freq, resolution);
    ledcAttachPin(spk_pin, buzzerChannel);
}

pa_activity (BuzzMaker, pa_ctx(pa_use(Delay); int i), bool withGap) {
    pa_repeat {
        for (pa_self.i = 0; pa_self.i < 4; ++pa_self.i) {
            ledcWriteNote(buzzerChannel, NOTE_C, 4);
            pa_run (Delay, 1);
            ledcWriteTone(buzzerChannel, 0);
            pa_run (Delay, 1);
        }

        if (withGap) {
            pa_run (Delay, 5);
        }
    }
} pa_end;

pa_activity (BuzzGenerator, pa_ctx(pa_co_res(2); pa_use(Delay); pa_use(BuzzMaker); int i)) {
    enableSpeaker();
    pa_pause;

    pa_co(2) {
        pa_with_weak (BuzzMaker, true);
        pa_with (Delay, 200);
    } pa_co_end;

    disableSpeaker();
    pa_pause;
    enableSpeaker();
    pa_pause;

    pa_co(2) {
        pa_with_weak (BuzzMaker, false);
        pa_with (Delay, 100);
    } pa_co_end;
} pa_end;

pa_activity (BuzzerController, pa_ctx(pa_use(BuzzGenerator)), bool active, const PressSignal& press, const PressSignal& up, const PressSignal& down, bool& isBuzzing) {
    pa_every (active) {
        isBuzzing = true;
        pa_when_abort (press || up || down, BuzzGenerator);
        isBuzzing = false;
        disableSpeaker();
    } pa_every_end;
} pa_end;

pa_activity (Buzzer, pa_ctx(pa_co_res(2); pa_use(AlarmTimeChecker); pa_use(BuzzerController); bool active), const PressSignal& press, const PressSignal& up, const PressSignal& down, bool& isBuzzing) {
    pa_co(2) {
        pa_with (AlarmTimeChecker, pa_self.active);
        pa_with (BuzzerController, pa_self.active, press, up, down, isBuzzing);
    } pa_co_end;
} pa_end;

// Main

pa_activity (Main, pa_ctx(pa_co_res(8); pa_signal_res;
                          pa_use(WiFiAndNTPConnector); pa_use(WiFiConnectionMaintainer); pa_use(PressRecognizer);
                          pa_use(UI); pa_use(Buzzer); pa_use(DisplayUpdater); pa_use(WaitScreen); pa_use(InputReceiver);
                          pa_def_val_signal(Press, press); pa_def_val_signal(Press, up); pa_def_val_signal(Press, down);
                          bool isBuzzing),
                   bool didOverrun) {
    pa_co (3) {
        pa_with (WiFiAndNTPConnector);
        pa_with_weak (WaitScreen);
        pa_with_weak (DisplayUpdater);
    } pa_co_end;

    pa_co(6) {
        pa_with (WiFiConnectionMaintainer);
        pa_with (PressRecognizer, 41, pa_self.press);
        pa_with (InputReceiver, pa_self.press, pa_self.up, pa_self.down);
        pa_with (Buzzer, pa_self.press, pa_self.up, pa_self.down, pa_self.isBuzzing);
        pa_with (UI, pa_self.press, pa_self.up, pa_self.down, pa_self.isBuzzing);
        pa_with (DisplayUpdater);
    } pa_co_end;
} pa_end;

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
