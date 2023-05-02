#include <esp_adc/adc_continuous.h>
#include <esp_task.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <array>
#include <cstdio>

static constexpr auto max_number_of_samples{100'000uz};
static constexpr auto sampling_frequency{10u * 1000u};
static constexpr auto expected_time{static_cast<double>(max_number_of_samples) /
                                    static_cast<double>(sampling_frequency)};
adc_continuous_handle_t adc1_handle{};
std::array<uint8_t, 256uz> result{};

// Times how long ADC takes to take "max_number_of_samples" at configured
// sampling frequency
void task_function(void*) {
  uint32_t ret_num{};
  size_t number_of_samples{};

  // Start ADC
  int64_t previous_us{esp_timer_get_time()};
  ESP_ERROR_CHECK(adc_continuous_start(adc1_handle));

  for (;;) {
    // Read samples
    if (adc_continuous_read(
          adc1_handle, data(result), size(result), &ret_num, 0u) != ESP_OK)
      continue;

    // Sum all, once done print time it took to measure them
    number_of_samples += ret_num;
    if (number_of_samples < max_number_of_samples) continue;
    number_of_samples = 0uz;

    // Convert time it took from us to s (10^6)
    auto const actual_us{esp_timer_get_time()};
    auto const delta_time{(actual_us - previous_us) / 1e6};
    printf("Taking %zu samples @ %uHz took %fs instead of %fs\n",
           max_number_of_samples,
           sampling_frequency,
           delta_time,
           expected_time);

    // Stop ADC and read till empty (might print error message that ADC is
    // already stopped since print before takes long)
    adc_continuous_stop(adc1_handle);
    while (adc_continuous_read(
             adc1_handle, data(result), size(result), &ret_num, 0u) == ESP_OK)
      ;

    // Restart ADC
    previous_us = esp_timer_get_time();
    ESP_ERROR_CHECK(adc_continuous_start(adc1_handle));
  }
}

extern "C" void app_main() {
  // Init ADC in continuous mode
  adc_continuous_handle_cfg_t adc_config{
    .max_store_buf_size = size(result) * SOC_ADC_DIGI_RESULT_BYTES * 8u,
    .conv_frame_size = size(result),
  };
  ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &adc1_handle));

  std::array patterns{
    adc_digi_pattern_config_t{
      .atten = ADC_ATTEN_DB_0,
      .channel = ADC_CHANNEL_0,
      .unit = ADC_UNIT_1,
      .bit_width = ADC_BITWIDTH_12,
    },
    adc_digi_pattern_config_t{
      .atten = ADC_ATTEN_DB_0,
      .channel = ADC_CHANNEL_0,
      .unit = ADC_UNIT_1,
      .bit_width = ADC_BITWIDTH_12,
    },
  };

  adc_continuous_config_t dig_cfg{
    .pattern_num = size(patterns),
    .adc_pattern = data(patterns),
    .sample_freq_hz = sampling_frequency,
    .conv_mode = ADC_CONV_SINGLE_UNIT_1,
    .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
  };
  ESP_ERROR_CHECK(adc_continuous_config(adc1_handle, &dig_cfg));

  // Create a max priority task on core 1
  xTaskCreatePinnedToCore(
    task_function, NULL, 4096uz, NULL, ESP_TASK_PRIO_MAX, NULL, 1);

  for (;;)
    vTaskDelay(pdMS_TO_TICKS(1000u));
}
