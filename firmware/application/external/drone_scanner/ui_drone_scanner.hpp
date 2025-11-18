#ifndef __UI_DRONE_SCANNER_HPP__
#define __UI_DRONE_SCANNER_HPP__

#include "ui.hpp"
#include "ui_widget.hpp"
#include "ui_navigation.hpp"
#include "ui_receiver.hpp"
#include "ui_font_fixed_8x16.hpp"
#include <vector>

namespace ui {

enum class SignalType {
    UNKNOWN,
    ANALOG_VIDEO,      // Wideband, Continuous
    DIGITAL_VIDEO,     // Wideband, Bursty (DJI/Walksnail)
    TELEMETRY_LORA,    // Narrowband, Hopping (ELRS/Crossfire)
    TELEMETRY_GENERIC, // Narrowband, Standard FSK
    STALE              // Special status for old signals
};

struct DetectedSignal {
    uint64_t freq;
    int32_t max_db;
    SignalType type;
    uint32_t last_seen_tick; // For stale detection
    int confidence;          // 0-100%
    int detection_count;     // How many times seen
};

// --- Foxhunt / Direction Finding View ---
class DirectionFindingView : public View {
public:
    DirectionFindingView(NavigationView& nav, DetectedSignal target_sig);
    ~DirectionFindingView();

    void focus() override;
    void on_show() override;
    void on_hide() override;
    
    std::string title() const override { return "FOXHUNT MODE"; };

private:
    NavigationView& nav_;
    DetectedSignal target_signal;
    
    Text text_target_info {
        { 2 * 8, 1 * 16, 26 * 8, 16 },
        ""
    };

    Text text_instructions {
        { 1 * 8, 3 * 16, 28 * 8, 16 },
        "ROTATE ANTENNA FOR PEAK"
    };
    
    // Huge RSSI display
    Text text_rssi_big {
        { 6 * 8, 5 * 16, 10 * 8, 32 },
        "-00"
    };
    
    ProgressBar rssi_bar {
        { 2 * 8, 9 * 16, 26 * 8, 32 }
    };
    
    Text text_peak {
        { 2 * 8, 13 * 16, 26 * 8, 16 },
        "PEAK: -100dBm"
    };

    Button button_back {
        { 8 * 8, 16 * 16, 14 * 8, 32 },
        "BACK TO LIST"
    };

    int32_t peak_db = -120;

    MessageHandlerRegistration message_handler_stats {
        Message::ID::ChannelStatistics,
        [this](const Message* const p) {
            this->on_channel_stats(static_cast<const ChannelStatisticsMessage*>(p));
        }
    };

    void on_channel_stats(const ChannelStatisticsMessage* p);
};

// --- Main Scanner View ---
class DroneScannerView : public View {
public:
    DroneScannerView(NavigationView& nav);
    ~DroneScannerView();

    void focus() override;
    void on_show() override;
    void on_hide() override;

    std::string title() const override { return "Drone Sentinel Pro"; };

private:
    NavigationView& nav_;

    Text text_status {
        { 0, 0, 30 * 8, 16 },
        "Status: Idle"
    };

    Labels labels_range {
        { 0, 2 * 16 },
        { "START:", "END:", "THR:" },
        Color::grey()
    };

    FrequencyField field_freq_start {
        { 6 * 8, 2 * 16 },
    };

    FrequencyField field_freq_end {
        { 17 * 8, 2 * 16 },
    };
    
    NumberField field_threshold {
        { 26 * 8, 2 * 16 },
        3, { -90, -10 }, 1, ' '
    };

    // Headers: FREQ, TYPE, CONF, LVL
    Text text_headers {
        { 0, 4 * 16, 30 * 8, 16 },
        "FREQ    TYPE     CNF LVL"
    };
    
    MenuView menu_view {
        { 0, 5 * 16, 240, 128 } 
    };

    Button button_scan {
        { 2 * 8, 15 * 16, 10 * 8, 32 },
        "SCAN"
    };

    Button button_clear {
        { 18 * 8, 15 * 16, 10 * 8, 32 },
        "CLEAR"
    };

    // Logic
    bool scanning = false;
    uint64_t current_freq = 0;
    uint64_t scan_start = 400000000; 
    uint64_t scan_end = 6000000000; 
    uint64_t step_size = 2000000;    
    int32_t threshold_db = -40;

    enum class ScanState {
        SEARCHING,
        ANALYZING
    };
    ScanState state = ScanState::SEARCHING;
    
    int analysis_samples = 0;
    int high_duty_ticks = 0;
    uint32_t global_tick = 0; // For stale timing

    std::vector<DetectedSignal> detected_list;

    MessageHandlerRegistration message_handler_stats {
        Message::ID::ChannelStatistics,
        [this](const Message* const p) {
            this->on_channel_stats(static_cast<const ChannelStatisticsMessage*>(p));
        }
    };

    void on_channel_stats(const ChannelStatisticsMessage* p);
    void start_scan();
    void stop_scan();
    void step_frequency();
    void classify_and_store(uint64_t freq, int32_t db, int duty_score);
    void update_list_ui();
    void open_foxhunt(const DetectedSignal& sig);
    std::string type_to_string(SignalType type);
};

} /* namespace ui */

#endif /* __UI_DRONE_SCANNER_HPP__ */
