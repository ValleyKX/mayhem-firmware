#include "ui_drone_scanner.hpp"
#include "receiver_model.hpp"
#include "baseband_api.hpp"
#include "portapack.hpp"
#include "string_format.hpp"

using namespace portapack;

namespace ui {

// =============================================================================
// Direction Finding (Foxhunt) View Implementation
// =============================================================================

DirectionFindingView::DirectionFindingView(NavigationView& nav, DetectedSignal target_sig) 
    : nav_(nav), target_signal(target_sig) 
{
    add_children({
        &text_target_info,
        &text_instructions,
        &text_rssi_big,
        &rssi_bar,
        &text_peak,
        &button_back
    });

    // Display the target frequency we are hunting
    std::string freq_str = to_string_short_freq(target_signal.freq);
    text_target_info.set("TARGET: " + freq_str);

    // Initialize RSSI bar range. 
    // We map -90dBm (floor) to -10dBm (saturation) into a 0-80 pixel width/value
    rssi_bar.set_max(80); 
    rssi_bar.set_value(0);

    button_back.on_select = [this](Button&) {
        nav_.pop();
    };

    // Setup Receiver for single frequency lock (Manual Mode equivalent)
    receiver_model.set_sampling_rate(3072000);
    receiver_model.set_baseband_bandwidth(2500000);
    receiver_model.set_tuning_frequency(target_signal.freq);
    receiver_model.enable();
    
    // Ensure the wideband image is loaded for accurate RSSI stats
    baseband::run_image(portapack::spi_flash::image_tag_wideband_spectrum);
}

DirectionFindingView::~DirectionFindingView() {
    receiver_model.disable();
}

void DirectionFindingView::focus() {
    button_back.focus();
}

void DirectionFindingView::on_show() {
    receiver_model.enable();
}

void DirectionFindingView::on_hide() {
    receiver_model.disable();
}

void DirectionFindingView::on_channel_stats(const ChannelStatisticsMessage* p) {
    // 1. Update Big RSSI Number Text
    text_rssi_big.set(to_string_dec_int(p->max_db) + "dB");

    // 2. Update Bar Graph Visual
    // Normalize -90 to -10 range into 0-80 for the progress bar
    int bar_val = p->max_db + 90;
    if (bar_val < 0) bar_val = 0;
    if (bar_val > 80) bar_val = 80;
    rssi_bar.set_value(bar_val);

    // 3. Peak Hold Logic (Hot/Cold indicator)
    // If current signal is stronger than our best peak, update peak
    if (p->max_db > peak_db) {
        peak_db = p->max_db;
        text_peak.set("PEAK: " + to_string_dec_int(peak_db) + "dB (HOT!)");
        text_peak.set_style(&style_white); // Highlight new peak
    } 
    // If signal drops significantly below peak, we are "Cold"
    else if (p->max_db < peak_db - 10) {
        text_peak.set_style(&style_grey); // Dim text to indicate signal loss
    }
}

// =============================================================================
// Main Drone Scanner View Implementation
// =============================================================================

DroneScannerView::DroneScannerView(NavigationView& nav) : nav_(nav) {
    add_children({
        &text_status,
        &labels_range,
        &field_freq_start,
        &field_freq_end,
        &field_threshold,
        &text_headers,
        &menu_view,
        &button_scan,
        &button_clear
    });

    // Initialize fields with defaults
    field_freq_start.set_value(scan_start);
    field_freq_start.on_change = [this](auto v) { scan_start = v; };
    
    field_freq_end.set_value(scan_end);
    field_freq_end.on_change = [this](auto v) { scan_end = v; };

    field_threshold.set_value(threshold_db);
    field_threshold.on_change = [this](auto v) { threshold_db = v; };

    button_scan.on_select = [this](Button&) {
        if (scanning) stop_scan();
        else start_scan();
    };

    button_clear.on_select = [this](Button&) {
        detected_list.clear();
        update_list_ui();
    };

    // Pre-configure receiver settings
    receiver_model.set_sampling_rate(3072000); 
    receiver_model.set_baseband_bandwidth(2500000);
}

DroneScannerView::~DroneScannerView() {
    stop_scan();
}

void DroneScannerView::focus() {
    button_scan.focus();
}

void DroneScannerView::on_show() {
    baseband::run_image(portapack::spi_flash::image_tag_wideband_spectrum);
    update_list_ui(); // Refresh list (re-calculate stale status)
}

void DroneScannerView::on_hide() {
    stop_scan();
}

void DroneScannerView::start_scan() {
    scanning = true;
    current_freq = scan_start;
    button_scan.set_text("STOP");
    text_status.set("Scanning...");
    state = ScanState::SEARCHING;
    step_frequency();
}

void DroneScannerView::stop_scan() {
    scanning = false;
    button_scan.set_text("SCAN");
    text_status.set("Idle");
    receiver_model.disable();
}

void DroneScannerView::step_frequency() {
    if (!scanning) return;

    // Loop back to start if we hit end
    if (current_freq > scan_end) {
        current_freq = scan_start; 
    }

    receiver_model.set_tuning_frequency(current_freq);
    receiver_model.enable();
}

void DroneScannerView::on_channel_stats(const ChannelStatisticsMessage* p) {
    if (!scanning) return;

    global_tick++; // Increment internal time ticker

    // Periodic List Refresh (every ~1 sec / 50 ticks) to update "Stale" status visuals
    if (global_tick % 50 == 0) {
        update_list_ui();
    }

    if (state == ScanState::SEARCHING) {
        // Fast Sweep Mode
        if (p->max_db > threshold_db) {
            // Signal Found -> Switch to Analyze Mode
            state = ScanState::ANALYZING;
            analysis_samples = 0;
            high_duty_ticks = 0;
            text_status.set("Analyzing...");
        } else {
            // No signal -> Next frequency
            current_freq += step_size;
            step_frequency();
        }
    } 
    else if (state == ScanState::ANALYZING) {
        // Dwell Mode (Analyzing signal characteristics)
        analysis_samples++;
        
        // Check if signal persists above threshold
        if (p->max_db > threshold_db) {
            high_duty_ticks++;
        }

        // Analyze for ~20 samples (approx 400ms)
        if (analysis_samples > 20) {
            classify_and_store(current_freq, p->max_db, high_duty_ticks);
            
            // Resume searching
            state = ScanState::SEARCHING;
            current_freq += step_size;
            step_frequency();
        }
    }
}

void DroneScannerView::classify_and_store(uint64_t freq, int32_t db, int duty_score) {
    SignalType type = SignalType::UNKNOWN;
    int confidence_boost = 0;
    
    // --- HEURISTIC CLASSIFICATION ---
    bool is_low_band = (freq >= 400000000 && freq <= 950000000);
    bool is_vid_band = (freq >= 1000000000);
    
    if (is_vid_band) {
        if (duty_score > 18) {
            // High Duty Cycle (>90%) -> Likely Analog Video
            type = SignalType::ANALOG_VIDEO; 
            confidence_boost = 30; 
        } else {
            // High Frequency but bursty -> Digital Video (DJI/Walksnail) or WiFi
            type = SignalType::DIGITAL_VIDEO; 
            confidence_boost = 15; 
        }
    } else if (is_low_band) {
        // Low band hopping/bursts -> LORA Control Link
        type = SignalType::TELEMETRY_LORA; 
        confidence_boost = 10;
    }

    // --- CONFIDENCE CALCULATION ---
    // Baseline: Stronger signal = higher confidence.
    // Map -90dBm to 0 score, -40dBm to 50 score.
    int signal_confidence = (db + 90); 
    if (signal_confidence < 0) signal_confidence = 0;
    
    int final_confidence = signal_confidence + confidence_boost;
    
    // --- LIST MANAGEMENT ---
    bool update = false;
    for (auto& s : detected_list) {
        // Group signals if they are within 5MHz of an existing entry
        if (abs((int64_t)s.freq - (int64_t)freq) < 5000000) {
            s.max_db = db;
            s.type = type;
            s.last_seen_tick = global_tick;
            s.detection_count++;
            
            // Rolling average for confidence, weighted by repetition
            s.confidence = (s.confidence + final_confidence) / 2 + (s.detection_count * 2);
            if (s.confidence > 99) s.confidence = 99;
            
            update = true;
            break;
        }
    }
    
    if (!update) {
        // New Detection
        DetectedSignal sig;
        sig.freq = freq;
        sig.max_db = db;
        sig.type = type;
        sig.last_seen_tick = global_tick;
        sig.detection_count = 1;
        sig.confidence = final_confidence;
        if (sig.confidence > 99) sig.confidence = 99;
        
        detected_list.push_back(sig);
        
        // FIFO Buffer limit
        if (detected_list.size() > 20) detected_list.erase(detected_list.begin());
    }

    update_list_ui();
}

void DroneScannerView::open_foxhunt(const DetectedSignal& sig) {
    stop_scan(); // Must stop the scanner before pushing new view
    nav_.push<DirectionFindingView>(sig);
}

std::string DroneScannerView::type_to_string(SignalType type) {
    switch(type) {
        case SignalType::ANALOG_VIDEO: return "VID-A";
        case SignalType::DIGITAL_VIDEO: return "VID-D";
        case SignalType::TELEMETRY_LORA: return "LORA ";
        case SignalType::TELEMETRY_GENERIC: return "TELEM";
        case SignalType::STALE: return "STALE";
        default: return "UNK  ";
    }
}

void DroneScannerView::update_list_ui() {
    menu_view.clear_items();
    
    // 10 seconds approx timeout (assuming ~50 ticks/sec update rate)
    const uint32_t STALE_TIMEOUT = 500; 

    // Iterate backwards to show newest first
    for (auto it = detected_list.rbegin(); it != detected_list.rend(); ++it) {
        auto& s = *it;
        SignalType display_type = s.type;
        
        // Check if signal is stale
        if (global_tick > s.last_seen_tick + STALE_TIMEOUT) {
            display_type = SignalType::STALE;
        }

        std::string f_str = to_string_short_freq(s.freq);
        std::string t_str = type_to_string(display_type);
        std::string c_str = to_string_dec_int(s.confidence) + "%";
        std::string l_str = to_string_dec_int(s.max_db);
        
        // Construct List Item String: "5.8G VID-A 90% -40"
        std::string entry = f_str + " " + t_str + " " + c_str + " " + l_str;
        
        // Add to menu with callback to trigger Foxhunt mode
        menu_view.add_item({
            entry, 
            ui::Color::white(), 
            nullptr, 
            [this, s](){ this->open_foxhunt(s); }
        });
    }
    
    menu_view.set_dirty();
}

} /* namespace ui */
