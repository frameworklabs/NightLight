// Project: NightLight
// Copyright: (c) 2025 Framework Labs

#include <M5AtomS3.h>

#include <M5UnitUnifiedGESTURE.h>
#include <M5UnitUnified.h>

#include <pa_ard_utils.h>
#include <proto_activities.h>

using namespace proto_activities::ard_utils;

// Helpers

pa_activity (Clock_ms, pa_ctx_tm(), pa_time_t ms, pa_signal& clock) {
    pa_every_ms (ms) {
        pa_emit (clock);
    } pa_every_end
} pa_end

// Gesture

m5::unit::UnitUnified Units;
m5::unit::UnitGESTURE unit;

namespace paj = m5::unit::paj7620u2;

constexpr const char* gstr[] = {
    "None", "Left ",    "Right",     "Down",          "Up",      "Forward", "Backward", "Clockwise", "CounterClockwise",
    "Wave", "Approach", "HasObject", "WakeupTrigger", "Confirm", "Abort",   "Reserve",  "NoObject",
};

const char* gesture_to_string(const paj::Gesture g) {
    const auto gg = m5::stl::to_underlying(g);
    
    const uint32_t idx = (gg == 0) ? 0 : __builtin_ctz(gg) + 1;
    return idx < m5::stl::size(gstr) ? gstr[idx] : "ERR";
}

pa_activity (UnitsUpdater, pa_ctx()) {
    pa_always {
        Units.update();
    } pa_always_end
} pa_end

pa_activity (GestureRecognizer, pa_ctx_tm(pa_use(UnitsUpdater)), PressSignal& press, pa_signal& failed) {
    
    // Setup I2C Wire
    Wire.begin();
    
    // Setup Unit
    if (!Units.add(unit, Wire) || !Units.begin()) {
        pa_emit (failed);    
        pa_halt;
    }
    
    // Read gestures
    pa_always {
        Units.update();
        if (unit.updated()) {
            Serial.printf("Gesture: %s (%d)\n", gesture_to_string(unit.gesture()), int(unit.gesture()));
            
            // Left  (1)
            // Right (2)
            // Down (4)
            // Up (8)
            // Forward (16)
            // Backward (32)
            // Clockwise (64)
            // CounterClockwise (128)
            // Wave (256)
            
            switch (int(unit.gesture())) {
                case 16: pa_emit_val(press, Press::short_press); break;
                case 1: pa_emit_val(press, Press::long_press); break;
                case 2: pa_emit_val(press, Press::long_press); break;
                case 4: pa_emit_val(press, Press::double_press); break;
                case 8: pa_emit_val(press, Press::double_press); break;
                default: break;
            } 
            
            if (press) {
                // Eat units for 1.2 secs
                pa_after_ms_abort (1200, UnitsUpdater);
            }
        }
    } pa_always_end
} pa_end

// Gesture Control

pa_activity (Toggle, pa_ctx(), bool pressed, bool& enabled) {
    pa_every (pressed) {
        enabled = !enabled;
    } pa_every_end
} pa_end

// Indicators

pa_activity (OnOffIndicator, pa_ctx_tm(bool enabled), bool enabled) {
    AtomS3.dis.drawpix(CRGB(enabled ? 0x00ff00 : 0xff0000));
    AtomS3.dis.show();
    pa_delay_ms (800);
    
    AtomS3.dis.clear();
    AtomS3.dis.show();
} pa_end

pa_activity (ActionIndicator, pa_ctx_tm(), Press press, bool enabled) {
    if (!enabled) {
        AtomS3.dis.drawpix(CRGB::Red);
    } else {
        switch (press) {
            case Press::short_press: AtomS3.dis.drawpix(CRGB::White); break;
            case Press::long_press: AtomS3.dis.drawpix(CRGB::Blue); break;
            case Press::double_press: AtomS3.dis.drawpix(CRGB::Orange); break;
        }
    }
    AtomS3.dis.show();
    pa_delay_ms (1200);
    
    AtomS3.dis.clear();
    AtomS3.dis.show();
} pa_end

pa_activity (GestureIndicator, pa_ctx(bool prev_enabled; pa_use(OnOffIndicator); pa_use(ActionIndicator)), 
                               const PressSignal& press, bool enabled) {
    pa_repeat {
        pa_await_immediate (press || enabled != pa_self.prev_enabled);
        
        if (press) {
            pa_when_abort (enabled != pa_self.prev_enabled, ActionIndicator, press.val(), enabled);
        } else {
            pa_self.prev_enabled = enabled;
            pa_when_abort (press || enabled != pa_self.prev_enabled, OnOffIndicator, enabled);
        }
    }
} pa_end

pa_activity (FailureIndicator, pa_ctx_tm()) {
    pa_repeat {
        Serial.println("Failed to connect to the gesture sensor.");
        
        AtomS3.dis.drawpix(CRGB::Red);
        AtomS3.dis.show();
        pa_delay_ms (500);
        
        AtomS3.dis.clear();
        AtomS3.dis.show();
        pa_delay_ms (200);
    }
} pa_end

pa_activity (Indicator, pa_ctx(pa_use(GestureIndicator); pa_use(FailureIndicator)), 
                        const PressSignal& press, bool enabled, bool failed) {
    AtomS3.dis.setBrightness(50);
    AtomS3.dis.clear();
    AtomS3.dis.show();
    
    if (!failed) {
        pa_when_abort (failed, GestureIndicator, press, enabled);
    }
    pa_run (FailureIndicator);
} pa_end

// Input synchronization and sending

struct Input {
    uint8_t data{};
    
    enum Slot : uint8_t {
        press_slot,
        up_slot,
        down_slot
    };
    
    Input() = default;
    Input(const Input&) = default;
    Input& operator=(const Input&) = default;
    
    Input(Input&& other) : data(other.data) {
        other.data = 0;
    }
    
    Input& operator=(Input&& other) {
        if (&other != this) {
            data = other.data;
            other.data = 0;
        }
        return *this;
    }
    
    bool get_press(uint8_t slot, Press& press) {
        const uint8_t val = 0xff && (data >> (slot * 2));
        if (!val) {
            return false;
        }
        press = Press(val - 1);
        return true;
    }
    
    void set_press(Press press, uint8_t slot) {
        data |= (m5::stl::to_underlying(press) + 1) << (slot * 2);
    }
    
    void clear_press(uint8_t slot) {
        data &= ~(0b00000011 << (slot * 2));
    }
    
    void impose_press(const PressSignal& press, uint8_t slot) {
        if (!press) {
            return;
        }
        Press old_press;
        if (!get_press(slot, old_press)) {
            set_press(press.val(), slot);
            return;
        }
        const auto old_val = m5::stl::to_underlying(old_press);
        const auto new_val = m5::stl::to_underlying(press.val());
        clear_press(slot);
        set_press(Press(max(old_val, new_val)), slot);
    }
};

using InputSignal = pa_val_signal<Input>;

pa_activity (Synchronizer, pa_ctx(Input accu), 
                           bool sync, const PressSignal& press, bool gesture_enabled, 
                           const PressSignal& up_press, const PressSignal& down_press, InputSignal& input) {
    pa_always {
        if (gesture_enabled) {
            pa_self.accu.impose_press(press, Input::press_slot);
        }
        pa_self.accu.impose_press(up_press, Input::up_slot);
        pa_self.accu.impose_press(down_press, Input::down_slot);
        if (sync) {
            pa_emit_val (input, std::move(pa_self.accu));
        }
    } pa_always_end
} pa_end

pa_activity (Sender, pa_ctx(), const InputSignal& input) {
    Serial1.begin(9600, SERIAL_8N1, 6, 5);
    pa_every (input) {
        Serial.printf("Sending: %d\n", input.val().data);
        Serial1.write(input.val().data);
    } pa_every_end
} pa_end

// Main

pa_activity (Main, pa_ctx(pa_co_res(9); pa_signal_res;
                          pa_use_as(PressRecognizer, MainRecognizer); pa_def_val_signal(Press, main_press); 
                          pa_use(Toggle); bool gesture_enabled;
                          pa_use(GestureRecognizer);  pa_def_val_signal(Press, press); pa_def_signal(gesture_failed);
                          pa_use(Indicator);
                          pa_use_as(PressRecognizer, UpRecognizer); pa_def_val_signal(Press, up_press); PressRecognizerConfig up_config;
                          pa_use_as(PressRecognizer, DownRecognizer); pa_def_val_signal(Press, down_press); PressRecognizerConfig down_config;
                          pa_use(Clock_ms); pa_def_signal(sync);
                          pa_use(Synchronizer); pa_def_val_signal(Input, input);
                          pa_use(Sender))) 
{
    pa_self.up_config.button_config.inspect_msg = "Up";
    pa_self.down_config.button_config.inspect_msg = "Down";
    pa_self.gesture_enabled = true;
    
    pa_co(9) {
        pa_with_as (PressRecognizer, MainRecognizer, 41, pa_self.main_press);
        pa_with (Toggle, pa_self.main_press, pa_self.gesture_enabled);
        pa_with (GestureRecognizer, pa_self.press, pa_self.gesture_failed);
        pa_with (Indicator, pa_self.press, pa_self.gesture_enabled, pa_self.gesture_failed);
        pa_with_as (PressRecognizer, UpRecognizer, 7, pa_self.up_press, pa_self.up_config);
        pa_with_as (PressRecognizer, DownRecognizer, 8, pa_self.down_press, pa_self.down_config);
        pa_with (Clock_ms, 100, pa_self.sync);
        pa_with (Synchronizer, pa_self.sync, pa_self.press, pa_self.gesture_enabled, pa_self.up_press, pa_self.down_press, pa_self.input);
        pa_with (Sender, pa_self.input);
    } pa_co_end
} pa_end

pa_use(Main);

// Setup and Loop

void setup() {
    auto config = M5.config();
    config.serial_baudrate = 115200;
    AtomS3.begin(config);
    AtomS3.dis.begin();
}

void loop() {
    AtomS3.update();
    pa_tick(Main);
    delay(10); // Limit to max 100Hz
}
