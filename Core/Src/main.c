/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BIG_FILE_PATH           "0:/11_06_~4.CFILE"
#define BIG_READ_CHUNK_BYTES    4096U
#define BIG_PROGRESS_STEP_BYTES (4UL * 1024UL * 1024UL)

#define B2LORA_SAMPLE_RATE_HZ   500000UL
#define B2LORA_BENCH_SECONDS    30UL
#define B2LORA_IQ_BYTES_PER_SMP 4UL
#define B2LORA_BENCH_BYTES      (B2LORA_SAMPLE_RATE_HZ * B2LORA_BENCH_SECONDS * B2LORA_IQ_BYTES_PER_SMP)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

ETH_TxPacketConfig TxConfig;
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

ETH_HandleTypeDef heth;

SD_HandleTypeDef hsd;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
FIL fil;
FILINFO fno;
FRESULT fres;
UINT br;

char msg[192];

uint8_t big_buf[BIG_READ_CHUNK_BYTES];

FSIZE_t big_file_size;
FSIZE_t total_read;
uint32_t big_hash;

typedef struct
{
  uint64_t cycles_joint_sniffing;
  uint64_t cycles_to_align;
  uint64_t cycles_fo_align;
  uint64_t cycles_phase_align;
  uint64_t cycles_total;

  uint64_t iq_pairs;
  uint64_t joint_metric;
  uint64_t to_metric;
  int64_t fo_metric;
  uint64_t phase_metric;

  int16_t prev_i;
  int16_t prev_q;
  uint8_t has_prev;
} B2LoRaTimingStats;

B2LoRaTimingStats b2_stats;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_SDIO_SD_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void uart_print(const char *s)
{
  HAL_UART_Transmit(&huart3, (uint8_t *)s, strlen(s), HAL_MAX_DELAY);
}

static uint32_t rolling_hash_update(uint32_t hash, const uint8_t *data, UINT len)
{
  UINT i;

  for (i = 0; i < len; i++)
  {
    hash = hash * 131u + data[i];
  }

  return hash;
}

static void u64_to_dec(uint64_t value, char *out, size_t out_size)
{
  char tmp[24];
  size_t i = 0;
  size_t j;

  if (out_size == 0u)
  {
    return;
  }

  if (value == 0u)
  {
    out[0] = '0';
    if (out_size > 1u)
    {
      out[1] = '\0';
    }
    return;
  }

  while ((value > 0u) && (i < sizeof(tmp)))
  {
    tmp[i++] = (char)('0' + (value % 10u));
    value /= 10u;
  }

  if ((i + 1u) > out_size)
  {
    out[0] = '?';
    if (out_size > 1u)
    {
      out[1] = '\0';
    }
    return;
  }

  for (j = 0u; j < i; j++)
  {
    out[j] = tmp[i - 1u - j];
  }
  out[i] = '\0';
}

static void i64_to_dec(int64_t value, char *out, size_t out_size)
{
  uint64_t mag;

  if (out_size == 0u)
  {
    return;
  }

  if (value >= 0)
  {
    u64_to_dec((uint64_t)value, out, out_size);
    return;
  }

  out[0] = '-';
  if (out_size == 1u)
  {
    return;
  }

  mag = (uint64_t)(-(value + 1)) + 1u;
  u64_to_dec(mag, out + 1, out_size - 1u);
}

static void uart_print_u64_line(const char *key, uint64_t value)
{
  char num[24];
  u64_to_dec(value, num, sizeof(num));
  snprintf(msg, sizeof(msg), "%s=%s\r\n", key, num);
  uart_print(msg);
}

static void uart_print_fixed3_line(const char *key, uint64_t x1000, const char *unit)
{
  unsigned long int_part = (unsigned long)(x1000 / 1000u);
  unsigned long frac_part = (unsigned long)(x1000 % 1000u);
  snprintf(msg, sizeof(msg), "%s=%lu.%03lu %s\r\n", key, int_part, frac_part, unit);
  uart_print(msg);
}

static void uart_print_fixed2_line(const char *key, uint64_t x100, const char *unit)
{
  unsigned long int_part = (unsigned long)(x100 / 100u);
  unsigned long frac_part = (unsigned long)(x100 % 100u);
  snprintf(msg, sizeof(msg), "%s=%lu.%02lu %s\r\n", key, int_part, frac_part, unit);
  uart_print(msg);
}

static int32_t abs_i32(int32_t x)
{
  return (x < 0) ? -x : x;
}

static void b2lora_profiler_init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0u;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static uint32_t b2lora_profiler_now_cycles(void)
{
  return DWT->CYCCNT;
}

static uint64_t b2lora_profiler_delta_cycles(uint32_t start, uint32_t end)
{
  return (uint64_t)(uint32_t)(end - start);
}

static void b2lora_stage_joint_sniffing(const int16_t *iq, size_t pairs)
{
  size_t i;
  uint64_t metric = b2_stats.joint_metric;

  for (i = 0; i < pairs; i++)
  {
    int32_t i0 = iq[2u * i];
    int32_t q0 = iq[2u * i + 1u];
    metric += (uint64_t)((uint32_t)(i0 * i0) + (uint32_t)(q0 * q0));
  }

  b2_stats.joint_metric = metric;
}

static void b2lora_stage_to_align(const int16_t *iq, size_t pairs)
{
  size_t i;
  int32_t prev_i = b2_stats.prev_i;
  int32_t prev_q = b2_stats.prev_q;
  uint8_t has_prev = b2_stats.has_prev;
  uint64_t metric = b2_stats.to_metric;

  for (i = 0; i < pairs; i++)
  {
    int32_t i0 = iq[2u * i];
    int32_t q0 = iq[2u * i + 1u];

    if (has_prev)
    {
      metric += (uint64_t)(abs_i32(i0 - prev_i) + abs_i32(q0 - prev_q));
    }

    prev_i = i0;
    prev_q = q0;
    has_prev = 1u;
  }

  b2_stats.prev_i = (int16_t)prev_i;
  b2_stats.prev_q = (int16_t)prev_q;
  b2_stats.has_prev = has_prev;
  b2_stats.to_metric = metric;
}

static void b2lora_stage_fo_align(const int16_t *iq, size_t pairs)
{
  size_t i;
  int32_t prev_i = b2_stats.prev_i;
  int32_t prev_q = b2_stats.prev_q;
  uint8_t has_prev = b2_stats.has_prev;
  int64_t metric = b2_stats.fo_metric;

  for (i = 0; i < pairs; i++)
  {
    int32_t i0 = iq[2u * i];
    int32_t q0 = iq[2u * i + 1u];

    if (has_prev)
    {
      metric += ((int64_t)prev_i * (int64_t)q0) - ((int64_t)prev_q * (int64_t)i0);
    }

    prev_i = i0;
    prev_q = q0;
    has_prev = 1u;
  }

  b2_stats.prev_i = (int16_t)prev_i;
  b2_stats.prev_q = (int16_t)prev_q;
  b2_stats.has_prev = has_prev;
  b2_stats.fo_metric = metric;
}

static void b2lora_stage_phase_align(const int16_t *iq, size_t pairs)
{
  size_t i;
  uint64_t metric = b2_stats.phase_metric;

  for (i = 0; i < pairs; i++)
  {
    int32_t i0 = iq[2u * i];
    int32_t q0 = iq[2u * i + 1u];
    int32_t p0 = i0 + q0;
    int32_t p1 = i0 - q0;
    int32_t p2 = -i0 + q0;
    int32_t p3 = -i0 - q0;

    int32_t best = abs_i32(p0);
    int32_t v1 = abs_i32(p1);
    int32_t v2 = abs_i32(p2);
    int32_t v3 = abs_i32(p3);

    if (v1 > best)
    {
      best = v1;
    }
    if (v2 > best)
    {
      best = v2;
    }
    if (v3 > best)
    {
      best = v3;
    }

    metric += (uint64_t)best;
  }

  b2_stats.phase_metric = metric;
}

static void b2lora_print_block_report(const char *name, uint64_t cycles)
{
  char cycle_num[24];
  uint64_t us = 0u;
  unsigned long ms_int = 0u;
  unsigned long ms_frac = 0u;

  u64_to_dec(cycles, cycle_num, sizeof(cycle_num));
  if (SystemCoreClock > 0u)
  {
    us = (cycles * 1000000u) / (uint64_t)SystemCoreClock;
  }
  ms_int = (unsigned long)(us / 1000u);
  ms_frac = (unsigned long)(us % 1000u);

  snprintf(msg,
           sizeof(msg),
           "  %-16s cycles=%s, time=%lu.%03lu ms\r\n",
           name,
           cycle_num,
           ms_int,
           ms_frac);
  uart_print(msg);
}

static void run_b2lora_mcu_pipeline_time_test(void)
{
  FSIZE_t next_progress;
  FSIZE_t target_bytes;
  uint8_t mount_try;
  uint32_t start_tick;
  uint32_t elapsed_ms;
  uint32_t c0;
  uint32_t c1;
  uint8_t read_error = 0;
  uint64_t valid_bytes_total = 0;
  uint64_t throughput_kib_x100 = 0u;
  uint64_t processed_signal_ms = 0u;
  uint64_t realtime_ratio_x1000 = 0u;
  uint64_t bench_seconds_x1000;
  char joint_num[24];
  char to_num[24];
  char fo_num[24];
  char phase_num[24];

  uart_print("\r\n===== B2LORA MCU PIPELINE TEST START =====\r\n");
  snprintf(msg, sizeof(msg), "Target path: %s\r\n", BIG_FILE_PATH);
  uart_print(msg);

  fres = FR_NOT_READY;
  for (mount_try = 1u; mount_try <= 3u; mount_try++)
  {
    fres = f_mount(&SDFatFS, (TCHAR const*)SDPath, 1);
    if (fres == FR_OK)
    {
      break;
    }

    snprintf(msg, sizeof(msg), "f_mount try %u FAILED, res=%d\r\n", mount_try, fres);
    uart_print(msg);
    snprintf(msg,
             sizeof(msg),
             "SD state=%lu, SD error=0x%08lX\r\n",
             (unsigned long)HAL_SD_GetCardState(&hsd),
             (unsigned long)HAL_SD_GetError(&hsd));
    uart_print(msg);

    (void)HAL_SD_DeInit(&hsd);
    HAL_Delay(20);
    if (BSP_SD_Init() == MSD_OK)
    {
      if (hsd.Init.BusWide == SDIO_BUS_WIDE_4B)
      {
        uart_print("SD reinit OK (4-bit)\r\n");
      }
      else
      {
        uart_print("SD reinit OK (1-bit fallback)\r\n");
      }
    }
    else
    {
      uart_print("SD reinit BSP_SD_Init FAILED\r\n");
    }
    HAL_Delay(50);
  }

  if (fres != FR_OK)
  {
    snprintf(msg, sizeof(msg), "f_mount FAILED, res=%d\r\n", fres);
    uart_print(msg);
    snprintf(msg,
             sizeof(msg),
             "SD state=%lu, SD error=0x%08lX\r\n",
             (unsigned long)HAL_SD_GetCardState(&hsd),
             (unsigned long)HAL_SD_GetError(&hsd));
    uart_print(msg);
    uart_print("===== B2LORA MCU PIPELINE TEST END =====\r\n");
    return;
  }
  uart_print("f_mount OK\r\n");

  if (hsd.Init.BusWide == SDIO_BUS_WIDE_4B)
  {
    uart_print("SD bus mode: 4-bit\r\n");
  }
  else if (hsd.Init.BusWide == SDIO_BUS_WIDE_1B)
  {
    uart_print("SD bus mode: 1-bit (fallback)\r\n");
  }
  else
  {
    snprintf(msg, sizeof(msg), "SD bus mode: unknown(0x%08lX)\r\n", (unsigned long)hsd.Init.BusWide);
    uart_print(msg);
  }

  fres = f_stat(BIG_FILE_PATH, &fno);
  if (fres != FR_OK)
  {
    snprintf(msg, sizeof(msg), "f_stat FAILED, res=%d\r\n", fres);
    uart_print(msg);
    uart_print("===== B2LORA MCU PIPELINE TEST END =====\r\n");
    return;
  }

  big_file_size = fno.fsize;
  snprintf(msg, sizeof(msg), "f_stat OK, file_size=%lu bytes\r\n", (unsigned long)big_file_size);
  uart_print(msg);

  fres = f_open(&fil, BIG_FILE_PATH, FA_READ);
  if (fres != FR_OK)
  {
    snprintf(msg, sizeof(msg), "f_open FAILED, res=%d\r\n", fres);
    uart_print(msg);
    uart_print("===== B2LORA MCU PIPELINE TEST END =====\r\n");
    return;
  }
  uart_print("f_open OK\r\n");

  target_bytes = big_file_size;
  if (target_bytes > (FSIZE_t)B2LORA_BENCH_BYTES)
  {
    target_bytes = (FSIZE_t)B2LORA_BENCH_BYTES;
  }

  bench_seconds_x1000 = ((uint64_t)target_bytes * 1000u) /
                        ((uint64_t)B2LORA_SAMPLE_RATE_HZ * (uint64_t)B2LORA_IQ_BYTES_PER_SMP);

  snprintf(msg, sizeof(msg), "benchmark_bytes=%lu (%lu.%03lu s @ %lu Hz)\r\n",
           (unsigned long)target_bytes,
           (unsigned long)(bench_seconds_x1000 / 1000u),
           (unsigned long)(bench_seconds_x1000 % 1000u),
           (unsigned long)B2LORA_SAMPLE_RATE_HZ);
  uart_print(msg);

  total_read = 0;
  big_hash = 0;
  memset(&b2_stats, 0, sizeof(b2_stats));
  b2lora_profiler_init();

  next_progress = (FSIZE_t)BIG_PROGRESS_STEP_BYTES;
  start_tick = HAL_GetTick();

  while (total_read < target_bytes)
  {
    UINT req = sizeof(big_buf);
    FSIZE_t remain = target_bytes - total_read;
    if (remain < (FSIZE_t)req)
    {
      req = (UINT)remain;
    }

    fres = f_read(&fil, big_buf, req, &br);
    if (fres != FR_OK)
    {
      snprintf(msg, sizeof(msg), "f_read FAILED, res=%d\r\n", fres);
      uart_print(msg);
      read_error = 1;
      break;
    }

    if (br == 0)
    {
      uart_print("Reached EOF\r\n");
      break;
    }

    total_read += (FSIZE_t)br;
    big_hash = rolling_hash_update(big_hash, big_buf, br);

    {
      UINT valid_bytes = br & ~(UINT)0x3u;
      size_t pairs = (size_t)(valid_bytes / 4u);
      const int16_t *iq = (const int16_t *)(const void *)big_buf;

      if (pairs > 0u)
      {
        valid_bytes_total += (uint64_t)valid_bytes;
        b2_stats.iq_pairs += (uint64_t)pairs;

        c0 = b2lora_profiler_now_cycles();
        b2lora_stage_joint_sniffing(iq, pairs);
        c1 = b2lora_profiler_now_cycles();
        b2_stats.cycles_joint_sniffing += b2lora_profiler_delta_cycles(c0, c1);

        c0 = b2lora_profiler_now_cycles();
        b2lora_stage_to_align(iq, pairs);
        c1 = b2lora_profiler_now_cycles();
        b2_stats.cycles_to_align += b2lora_profiler_delta_cycles(c0, c1);

        c0 = b2lora_profiler_now_cycles();
        b2lora_stage_fo_align(iq, pairs);
        c1 = b2lora_profiler_now_cycles();
        b2_stats.cycles_fo_align += b2lora_profiler_delta_cycles(c0, c1);

        c0 = b2lora_profiler_now_cycles();
        b2lora_stage_phase_align(iq, pairs);
        c1 = b2lora_profiler_now_cycles();
        b2_stats.cycles_phase_align += b2lora_profiler_delta_cycles(c0, c1);
      }
    }

    if (total_read >= next_progress)
    {
      unsigned long percent = 0;

      if (target_bytes > 0u)
      {
        percent = (unsigned long)(((uint64_t)total_read * 100ULL) / (uint64_t)target_bytes);
      }

      if (percent > 100UL)
      {
        percent = 100UL;
      }

      snprintf(msg, sizeof(msg), "Progress: %lu / %lu bytes (%lu%%)\r\n",
               (unsigned long)total_read,
               (unsigned long)target_bytes,
               percent);
      uart_print(msg);

      while (total_read >= next_progress)
      {
        next_progress += (FSIZE_t)BIG_PROGRESS_STEP_BYTES;
      }
    }
  }

  f_close(&fil);
  elapsed_ms = HAL_GetTick() - start_tick;

  b2_stats.cycles_total = b2_stats.cycles_joint_sniffing
                        + b2_stats.cycles_to_align
                        + b2_stats.cycles_fo_align
                        + b2_stats.cycles_phase_align;

  if (elapsed_ms > 0u)
  {
    throughput_kib_x100 = (valid_bytes_total * 100000u) / ((uint64_t)elapsed_ms * 1024u);
  }

  if (B2LORA_SAMPLE_RATE_HZ > 0u)
  {
    processed_signal_ms = (b2_stats.iq_pairs * 1000u) / (uint64_t)B2LORA_SAMPLE_RATE_HZ;
  }

  if ((elapsed_ms > 0u) && (processed_signal_ms > 0u))
  {
    realtime_ratio_x1000 = (processed_signal_ms * 1000u) / (uint64_t)elapsed_ms;
  }

  uart_print("----- SUMMARY -----\r\n");
  snprintf(msg, sizeof(msg), "file_size=%lu\r\n", (unsigned long)big_file_size);
  uart_print(msg);
  snprintf(msg, sizeof(msg), "benchmark_target_bytes=%lu\r\n", (unsigned long)target_bytes);
  uart_print(msg);
  snprintf(msg, sizeof(msg), "total_read=%lu\r\n", (unsigned long)total_read);
  uart_print(msg);
  uart_print_u64_line("valid_iq_bytes", valid_bytes_total);
  uart_print_u64_line("iq_pairs", b2_stats.iq_pairs);
  snprintf(msg, sizeof(msg), "rolling_hash=0x%08lX\r\n", (unsigned long)big_hash);
  uart_print(msg);
  snprintf(msg, sizeof(msg), "elapsed_ms=%lu\r\n", (unsigned long)elapsed_ms);
  uart_print(msg);
  uart_print_fixed2_line("throughput", throughput_kib_x100, "KiB/s");
  uart_print_fixed3_line("processed_signal", processed_signal_ms, "s");
  uart_print_fixed3_line("real_time_ratio", realtime_ratio_x1000, "x");

  uart_print("\r\n=== DWT Measured Overhead (Module Template) ===\r\n");
  b2lora_print_block_report("joint_sniffing", b2_stats.cycles_joint_sniffing);
  b2lora_print_block_report("to_align", b2_stats.cycles_to_align);
  b2lora_print_block_report("fo_align", b2_stats.cycles_fo_align);
  b2lora_print_block_report("phase_align", b2_stats.cycles_phase_align);
  b2lora_print_block_report("total", b2_stats.cycles_total);

  u64_to_dec(b2_stats.joint_metric, joint_num, sizeof(joint_num));
  u64_to_dec(b2_stats.to_metric, to_num, sizeof(to_num));
  i64_to_dec(b2_stats.fo_metric, fo_num, sizeof(fo_num));
  u64_to_dec(b2_stats.phase_metric, phase_num, sizeof(phase_num));
  snprintf(msg, sizeof(msg), "metrics: joint=%s, to=%s, fo=%s, phase=%s\r\n",
           joint_num,
           to_num,
           fo_num,
           phase_num);
  uart_print(msg);

  if ((!read_error) && (total_read == target_bytes))
  {
    uart_print("benchmark_done=YES\r\n");
  }
  else
  {
    uart_print("benchmark_done=NO\r\n");
  }

  uart_print("===== B2LORA MCU PIPELINE TEST END =====\r\n");
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  run_b2lora_mcu_pipeline_time_test();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_Delay(1000);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  /* Keep 1-bit during card init; BSP_SD_Init switches to 4-bit after init succeeds. */
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  /* 48MHz / (118 + 2) = 400kHz, safer for SD card init phase. */
  hsd.Init.ClockDiv = 118;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 4;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
