# FreeRTOS Task Timing

## STM32 Task Configuration

### Main Control Task

**Priority**: `osPriorityNormal` (default)  
**Stack Size**: 256 words (1024 bytes)  
**Period**: 2 ms (500 Hz)

```
┌─────────────────────────────────────────┐
│         FreeRTOS Scheduler              │
│         (1 kHz tick rate)               │
└─────────────────┬───────────────────────┘
                  │
                  ├─── Tick 0ms
                  │    ├─ Control Task Start
                  │    ├─ Read IMU (I2C)       ~0.3ms
                  │    ├─ Estimator Update     ~0.1ms
                  │    ├─ Controller Update    ~0.2ms
                  │    ├─ Mixer               ~0.05ms
                  │    ├─ PWM Write           ~0.05ms
                  │    └─ vTaskDelay(2)
                  │
                  ├─── Tick 2ms
                  │    └─ Control Task Resume
                  │
                  ├─── Tick 4ms
                  │    └─ Control Task Resume
                  │
                  └─── ...
```

### Execution Time Budget

| Operation | Time (μs) | % of 2ms |
|-----------|-----------|----------|
| I2C Read IMU | 300 | 15% |
| Estimator | 100 | 5% |
| Controller | 200 | 10% |
| Mixer | 50 | 2.5% |
| PWM Write | 50 | 2.5% |
| **Total** | **700** | **35%** |
| **Margin** | 1300 | 65% |

### Additional Tasks (Future)

**Telemetry Task**  
- Priority: Low  
- Period: 50 ms (20 Hz)  
- Function: Send attitude, status via UART

**RC Input Task**  
- Priority: High  
- Period: 10 ms (100 Hz)  
- Function: Read PWM/SBUS from receiver

**LED Task**  
- Priority: Idle  
- Period: 500 ms (2 Hz)  
- Function: Blink status LED

## Timing Diagram

```
Time (ms)    0     2     4     6     8     10
             │     │     │     │     │     │
Control      ▓▓▓▓▓ ▓▓▓▓▓ ▓▓▓▓▓ ▓▓▓▓▓ ▓▓▓▓▓ ...
Task         │     │     │     │     │
             
Telemetry    ▓▓▓▓▓───────────────────────▓▓▓▓▓
Task         │     (sleeps 50ms)         │

RC Input     ▓▓───────▓▓───────▓▓────────
Task         │  (10ms)│  (10ms)│

LED Task     ▓──────────────────────────────...
             │    (sleeps 500ms)
```

Legend:
- `▓` = Task running
- `─` = Task sleeping

## Interrupt Priorities (NVIC)

1. **SysTick**: Priority 15 (lowest) - FreeRTOS tick
2. **I2C**: Priority 5 - IMU data ready
3. **TIM**: Priority 10 - PWM update
4. **UART**: Priority 8 - Telemetry

## Critical Sections

**Estimator State**  
- Protected by: Task suspension (single task access)
- Duration: ~100 μs

**Controller State**  
- Protected by: Task suspension
- Duration: ~200 μs

## Worst-Case Analysis

**Control Loop Jitter**  
- Best case: 2.00 ms  
- Worst case: 2.10 ms (with interrupt preemption)  
- Jitter: ±50 μs (acceptable for 500 Hz)

**IMU Sampling**  
- Hardware: 1 kHz  
- Software read: 500 Hz  
- Latency: <1 ms

## Optimization Tips

1. **DMA for I2C**: Reduce IMU read time from 300 μs → 50 μs
2. **Fast Math**: Use ARM DSP instructions for trigonometry
3. **Inline Functions**: Mark small functions as `inline`
4. **Compiler Optimization**: Use `-O2` or `-O3`

## Timing Verification

Use GPIO pin toggling to measure execution time:

```c
void ControlTask(void *arg) {
    while (1) {
        HAL_GPIO_WritePin(DEBUG_GPIO, GPIO_PIN_SET);
        
        // ... control loop code ...
        
        HAL_GPIO_WritePin(DEBUG_GPIO, GPIO_PIN_RESET);
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}
```

Measure with oscilloscope:
- Pulse width = execution time
- Pulse period = loop rate

---

**Last Updated**: November 2025  
**Author**: habibourakash
