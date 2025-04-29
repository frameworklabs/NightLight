// Project: NightLight
// Copyright: (c) 2025 Framework Labs

#include <proto_activities.h>
#include <pa_ard_utils.h>

#include <FastLED.h>
#include <NeoPixelBusLg.h>
#include <M5_PbHub.h>

#include <M5Atom.h>

using namespace proto_activities::ard_utils;

// Event utils

pa_activity (DelayS, pa_ctx_tm(), unsigned int s) {
    pa_delay_s (s);
} pa_end

pa_activity (EventExtender, pa_ctx(pa_use(DelayS)), bool event, unsigned int duration_s, bool& prolongedEvent) {
    prolongedEvent = false;
    pa_every (event) {
        prolongedEvent = true;
        pa_when_reset (event, DelayS, duration_s);
        prolongedEvent = false;
    } pa_every_end
} pa_end

// EMA filter

static int ema(int value, int average, float alpha) {
    return static_cast<int>(alpha * value + (1 - alpha) * average);
}

pa_activity (EMAFilter, pa_ctx(int average), int inValue, float alpha, int& outValue) {
    outValue = pa_self.average = inValue;
    pa_pause;

    pa_always {
        outValue = pa_self.average = ema(inValue, pa_self.average, alpha);
        //Serial.printf("ema: %d\n", outValue);
    } pa_always_end
} pa_end

// Light

static NeoPixelBusLg<NeoGrbFeature, NeoEsp32I2s0Sk6812Method> strip(9, 23);

static const RgbColor WHITE(255, 255, 255);
static const RgbColor BLACK(0);

pa_activity (LightChangeDetector, pa_ctx(RgbColor prevColor; uint8_t prevBrightness), RgbColor color, int brightness, bool& didChange) {
    didChange = true;
    pa_self.prevColor = color;
    pa_self.prevBrightness = brightness;
    pa_pause;

    pa_always {
        didChange = pa_self.prevColor != color || pa_self.prevBrightness != brightness;
        pa_self.prevColor = color;
        pa_self.prevBrightness = brightness;
    } pa_always_end
} pa_end

pa_activity (BrightnessCalculator, pa_ctx(bool prevAdjustedBrigtness), 
                                   int brightness, bool isDay, bool isChanging, int& adjustedBrightness, bool& didChange) {
    pa_self.prevAdjustedBrigtness = brightness;
    pa_always {
        if (isChanging) {
            adjustedBrightness = brightness;
        } else if (isDay) {
            adjustedBrightness = 0;
        } else {
            adjustedBrightness = brightness;
        }
        didChange |= adjustedBrightness != pa_self.prevAdjustedBrigtness;
        pa_self.prevAdjustedBrigtness = adjustedBrightness;
    } pa_always_end
} pa_end

pa_activity (LightDriver, pa_ctx(), RgbColor color, int brightness, bool didChange) {
    strip.Begin();

    pa_every (didChange) {
        strip.SetLuminance(brightness);
        for (uint16_t i = 0; i < 9; ++i) {
            strip.SetPixelColor(i, color);
        }
        strip.Show();
    } pa_every_end
} pa_end

pa_activity (Light, pa_ctx(pa_co_res(4); pa_use(LightChangeDetector); pa_use(EventExtender); pa_use(BrightnessCalculator); pa_use(LightDriver); 
                           bool didChange; bool isChanging; int adjustedBrightness), 
                    RgbColor color, int brightness, bool isDay) {
    pa_co(4) {
        pa_with (LightChangeDetector, color, brightness, pa_self.didChange);
        pa_with (EventExtender, pa_self.didChange, 2, pa_self.isChanging);
        pa_with (BrightnessCalculator, brightness, isDay, pa_self.isChanging, pa_self.adjustedBrightness, pa_self.didChange);
        pa_with (LightDriver, color, pa_self.adjustedBrightness, pa_self.didChange);
    } pa_co_end
} pa_end

static void turnOffLight() {
    strip.ClearTo(BLACK);
    strip.SetLuminance(0);
    strip.Show();
}

// Pb.Hub

static M5_PbHub pbHub;

static constexpr uint8_t CH_PHO = 1;
static constexpr uint8_t CH_PIR = 2;
static constexpr uint8_t CH_POT = 3;
static constexpr uint8_t CH_PAD = 4;

// Motion

pa_activity (MotionSensor, pa_ctx(), bool& state) {
    pa_always {
        state = pbHub.digitalRead(CH_PIR);
    } pa_always_end
} pa_end

// Photons

pa_activity (PhotonReader, pa_ctx(), int& level) {
    pa_always {
        const auto value = pbHub.analogRead(CH_PHO);
        level = map(value, 0, 4095, 0, 31);
    } pa_always_end
} pa_end

pa_activity (PhotonThresholder, pa_ctx(), int level, int threshold, bool& triggered) {
    pa_always {
        // Serial.printf("photon: %d\n", level);
        triggered = level <= threshold;
    } pa_always_end
} pa_end

pa_activity (PhotonSensor, pa_ctx(pa_co_res(4); pa_use(PhotonReader); pa_use(EMAFilter); pa_use(PhotonThresholder); int rawLevel; int level), bool& triggered) {
    pa_co(3) {
        pa_with (PhotonReader, pa_self.rawLevel);
        pa_with (EMAFilter, pa_self.rawLevel, 0.4, pa_self.level);
        pa_with (PhotonThresholder, pa_self.level, 27, triggered);
    } pa_co_end
} pa_end

// Brightness Dial

pa_activity (BrightnessReader, pa_ctx(), int& brightness) {
    pa_always {
        const auto value = pbHub.analogRead(CH_POT);
        brightness = map(value, 0, 4095, 0, 255);
    } pa_always_end
} pa_end

pa_activity (BrightnessDial, pa_ctx(pa_co_res(2); pa_use(BrightnessReader); pa_use(EMAFilter); int rawBrightness), int& brightness) {
    pa_co(2) {
        pa_with (BrightnessReader, pa_self.rawBrightness);
        pa_with (EMAFilter, pa_self.rawBrightness, 0.4, brightness);
    } pa_co_end
} pa_end

// Color Slider

static NeoPixelBusLg<NeoGrbFeature, NeoEsp32I2s1Sk6812Method> sliderStrip(14, 26);

static constexpr int sliderQuant = 47;

pa_activity (SliderReader, pa_ctx(), int& rawValue) {
    adcAttachPin(32);

    pa_always {
        const auto value = analogRead(32);
        rawValue = map(value, 0, 4095, 0, sliderQuant);
    } pa_always_end
} pa_end

pa_activity (SliderToColorConverter, pa_ctx(), int value, RgbColor& color) {
    pa_always {
        // Serial.printf("color: %d\n", value);
        if (value == 19 || value == 20) {
            color = WHITE;
        } else {
            const float hue = static_cast<float>(value) / sliderQuant;
            color = HsbColor(hue, 1.0f, 1.0f);
        }
    } pa_always_end
} pa_end

pa_activity (ColorChangeDetector, pa_ctx(RgbColor prevColor), RgbColor color, bool& didChange) {
    pa_always {
        didChange = color != pa_self.prevColor;
        pa_self.prevColor = color;
    } pa_always_end
} pa_end

pa_activity (Colorizer, pa_ctx(bool prevIsOn), bool isOn) {
    sliderStrip.Begin();
    sliderStrip.SetLuminance(64);
    
    pa_repeat {
        if (isOn) {
            for (uint16_t i = 0; i < 7; ++i) {
                RgbColor color = WHITE;
                if (i != 3) {
                    const float hue = -static_cast<float>(i) / 6.0f;
                    color = HsbColor(hue, 1.0f, 1.0f);
                }                
                sliderStrip.SetPixelColor(i, color);
                sliderStrip.SetPixelColor(13 - i, color);
            }
        } else {
            sliderStrip.ClearTo(BLACK);
        }
        sliderStrip.Show();

        pa_self.prevIsOn = isOn;
        pa_await (pa_self.prevIsOn != isOn);
    }
} pa_end

static void turnOffSlider() {
    sliderStrip.ClearTo(BLACK);
    sliderStrip.SetLuminance(0);
    sliderStrip.Show();
}

pa_activity (ColorSlider, pa_ctx(pa_co_res(6); pa_use(SliderReader); pa_use(EMAFilter); pa_use(SliderToColorConverter);
                                 pa_use(EventExtender); pa_use(Colorizer); pa_use(ColorChangeDetector); 
                                 int rawSlider; int slider; bool didChange; bool colorizingOn), 
                          RgbColor& color) {    
    pa_co(6) {
        pa_with (SliderReader, pa_self.rawSlider);
        pa_with (EMAFilter, pa_self.rawSlider, 0.4, pa_self.slider);
        pa_with (SliderToColorConverter, pa_self.slider, color);
        pa_with (ColorChangeDetector, color, pa_self.didChange);
        pa_with (EventExtender, pa_self.didChange, 2, pa_self.colorizingOn);
        pa_with (Colorizer, pa_self.colorizingOn);
    } pa_co_end
} pa_end

// Mode control

pa_activity (OnMode, pa_ctx(pa_co_res(4); pa_use(Light); pa_use(BrightnessDial); pa_use(PhotonSensor);
                            pa_use(ColorSlider); int brightness; RgbColor color; bool isDay)) {
    pa_co(4) {
        pa_with (BrightnessDial, pa_self.brightness);
        pa_with (ColorSlider, pa_self.color);
        pa_with (PhotonSensor, pa_self.isDay);
        pa_with (Light, pa_self.color, pa_self.brightness, pa_self.isDay);
    } pa_co_end
} pa_end

pa_activity (OffMode, pa_ctx()) {
    turnOffSlider();
    turnOffLight();
    pa_halt;
} pa_end

pa_activity (ModeController, pa_ctx(pa_use(OnMode); pa_use(OffMode)), const PressSignal& press, bool& isOn) {
    pa_repeat {
        isOn = true;
        pa_when_abort (press && press.val() == Press::short_press, OnMode);

        isOn = false;
        pa_when_abort (press && press.val() == Press::short_press, OffMode);
    }
} pa_end

// Mode Indicator

static CRGB mainLED;

static void initLED() {
    FastLED.addLeds<NEOPIXEL, 27>(&mainLED, 1);
    FastLED.setBrightness(5);
}

static void setLED(CRGB color) {
    mainLED = color;
    FastLED.show();
}

pa_activity (Indicator, pa_ctx(bool prevIndicationOn; bool prevIsOn), bool indicationOn, bool isOn) {
    initLED();

    pa_repeat {
        if (indicationOn) {
            setLED(isOn ? CRGB::Green : CRGB::Red);
        } else {
            setLED(CRGB::Black);
        }

        pa_self.prevIndicationOn = indicationOn;
        pa_self.prevIsOn = isOn;

        pa_await (pa_self.prevIndicationOn != indicationOn || pa_self.prevIsOn != isOn);
    }
} pa_end

pa_activity (ModeIndicator, pa_ctx(pa_co_res(4); pa_use(MotionSensor); pa_use(EventExtender); pa_use(Indicator); bool motionState; bool indicationOn), bool isOn) {
    pa_co(3) {
        pa_with (MotionSensor, pa_self.motionState);
        pa_with (EventExtender, pa_self.motionState, 10, pa_self.indicationOn);
        pa_with (Indicator, pa_self.indicationOn, isOn);
    } pa_co_end
} pa_end

// Main

pa_activity (Main, pa_ctx(pa_co_res(4); pa_signal_res;
                          pa_use(ModeController); pa_use(ModeIndicator); bool isOn;
                          pa_use(PressRecognizer); pa_def_val_signal(Press, press))) {
    pa_co(3) {
        pa_with (PressRecognizer, 39, pa_self.press);
        pa_with (ModeController, pa_self.press, pa_self.isOn);
        pa_with (ModeIndicator, pa_self.isOn);
    } pa_co_end
} pa_end

// Setup and Loop

static pa_use(Main);

void setup() {
    setCpuFrequencyMhz(80);
  
    M5.begin(true, false);
    Wire.begin(25, 21);
}

void loop() {
    TickType_t prevWakeTime = xTaskGetTickCount();

    while (true) {
        M5.update();

        pa_tick(Main);

        // We run at 10 Hz.
        vTaskDelayUntil(&prevWakeTime, 100);
    }
}
