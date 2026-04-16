#ifndef B2LORA_MCU_CONFIG_H
#define B2LORA_MCU_CONFIG_H

#include <stddef.h>
#include <stdint.h>

typedef struct {
    double clock_hz;
    float vdd_v;
    float active_current_a;
    size_t sram_budget_bytes;
    int cycles_complex_mul;
    int cycles_complex_add;
    int cycles_abs2;
    int cycles_compare;
    int cycles_trig;
    int cycles_scalar;
} B2LoRaPowerModel;

typedef struct {
    /* Signal parameters */
    double sample_rate_hz;
    double bandwidth_hz;
    uint8_t sf;
    double total_duration_s;
    double packet_period_s;

    /* Packet structure */
    int preamble_up_chirps;
    int preamble_down_chirps;
    float preamble_down_tail_ratio;
    int packet_chirps;

    /* Synthetic collision scene */
    int collision_packet_count;
    int collision_start_sample;
    int collision_spacing_samples;
    float collision_base_cfo_hz;
    float collision_cfo_step_hz;
    float collision_amplitude_step;
    float collision_phase_step_rad;
    int signal_length_samples;
    float noise_sigma;

    /* Chaining-dechirp and detection */
    int chaining_step_samples;
    int chaining_inner_stride;
    int detection_guard_windows;
    float detection_threshold_k;
    int detection_min_separation_samples;
    int detection_max_candidates;
    int expected_packets;

    /* TO alignment */
    int to_coarse_range;
    int to_coarse_step;
    int to_fine_range;
    int to_corr_stride;

    /* FO alignment */
    int fo_estimation_samples;
    int fo_compensation_samples;

    /* Inter-packet phase correction test */
    int phase_search_steps;
    int phase_test_samples;

    /* Streaming I/O (microSD/FatFs or host file emulation) */
    int io_chunk_samples;
    float io_iq_scale;

    /* MCU power model */
    B2LoRaPowerModel power;
} B2LoRaConfig;

static inline B2LoRaConfig b2lora_default_config(void) {
    B2LoRaConfig cfg;

    cfg.sample_rate_hz = 500000.0;
    cfg.bandwidth_hz = 125000.0;
    cfg.sf = 11;
    cfg.total_duration_s = 12.0 * 60.0;
    cfg.packet_period_s = 30.0;

    cfg.preamble_up_chirps = 8;
    cfg.preamble_down_chirps = 2;
    cfg.preamble_down_tail_ratio = 0.25f;
    cfg.packet_chirps = 100;

    cfg.signal_length_samples = (int)(cfg.total_duration_s * cfg.sample_rate_hz);
    cfg.collision_spacing_samples = (int)(cfg.packet_period_s * cfg.sample_rate_hz);
    cfg.collision_start_sample = (int)(5.0 * cfg.sample_rate_hz);
    cfg.collision_packet_count = (int)(cfg.total_duration_s / cfg.packet_period_s);
    if (cfg.collision_packet_count < 1) {
        cfg.collision_packet_count = 1;
    }
    while (cfg.collision_packet_count > 1 &&
           cfg.collision_start_sample + (cfg.collision_packet_count - 1) * cfg.collision_spacing_samples >= cfg.signal_length_samples) {
        cfg.collision_packet_count--;
    }
    cfg.collision_base_cfo_hz = 120.0f;
    cfg.collision_cfo_step_hz = 8.0f;
    cfg.collision_amplitude_step = 0.02f;
    cfg.collision_phase_step_rad = 0.6f;
    cfg.noise_sigma = 0.06f;

    cfg.chaining_step_samples = 8192;
    cfg.chaining_inner_stride = 1;
    cfg.detection_guard_windows = 2;
    cfg.detection_threshold_k = 2.2f;
    cfg.detection_min_separation_samples = cfg.collision_spacing_samples / 2;
    cfg.detection_max_candidates = cfg.collision_packet_count;
    cfg.expected_packets = cfg.collision_packet_count;

    cfg.to_coarse_range = 120000;
    cfg.to_coarse_step = 512;
    cfg.to_fine_range = 512;
    cfg.to_corr_stride = 128;

    cfg.fo_estimation_samples = 8192;
    cfg.fo_compensation_samples = 65536;

    cfg.phase_search_steps = 4;
    cfg.phase_test_samples = 8192;

    cfg.io_chunk_samples = 2048;
    cfg.io_iq_scale = 1.0f / 32768.0f;

    cfg.power.clock_hz = 48000000.0;
    cfg.power.vdd_v = 3.3f;
    cfg.power.active_current_a = 0.010f;
    cfg.power.sram_budget_bytes = 256u * 1024u;
    cfg.power.cycles_complex_mul = 6;
    cfg.power.cycles_complex_add = 2;
    cfg.power.cycles_abs2 = 3;
    cfg.power.cycles_compare = 1;
    cfg.power.cycles_trig = 30;
    cfg.power.cycles_scalar = 1;

    return cfg;
}

static inline int b2lora_runtime_packet_count(const B2LoRaConfig *cfg, int signal_len_samples) {
    int count = 0;
    for (int p = 0; p < cfg->collision_packet_count; ++p) {
        int start = cfg->collision_start_sample + p * cfg->collision_spacing_samples;
        if (start >= signal_len_samples) {
            break;
        }
        count++;
    }
    return count;
}

static inline int b2lora_samples_per_symbol(const B2LoRaConfig *cfg) {
    int sf_samples = 1 << cfg->sf;
    return (int)(sf_samples * (cfg->sample_rate_hz / cfg->bandwidth_hz));
}

static inline int b2lora_preamble_samples(const B2LoRaConfig *cfg) {
    int sps = b2lora_samples_per_symbol(cfg);
    int full = (cfg->preamble_up_chirps + cfg->preamble_down_chirps) * sps;
    int tail = (int)(cfg->preamble_down_tail_ratio * (float)sps);
    return full + tail;
}

#endif
