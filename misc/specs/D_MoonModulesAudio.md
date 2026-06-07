# MoonModules Audio Driver — Implementation Spec

**Node name:** `MoonModules Audio`  
**File:** `src/MoonLight/Nodes/Drivers/D_MoonModulesAudio.h`  
**Feature flag:** `FT_MOONLIGHT_AUDIO`  
**Boards:** ESP32-S3, ESP32-P4 only (too memory-constrained on D0)

---

## 1. Purpose

`MoonModulesAudioDriver` is a MoonLight driver node that captures audio locally from an I2S microphone or codec and processes it through the WLEDMM audio pipeline (filters → AGC → FFT → 16-band EQ), writing the results into `sharedData` — the same fields used by the UDP-based `WLEDAudioDriver` (`D_WLEDAudio.h`).

This fills the `loopAudioDriver()` stub that exists in `WLEDAudioDriver`, but as a separate node. The two are mutually exclusive alternatives for populating `sharedData.bands / volume / majorPeak / magnitude`:

| Driver | Audio source | `sharedData` fields written |
|---|---|---|
| `WLEDAudioDriver` (mode 0) | UDP from a WLED/MoonLight transmitter | bands, volume, volumeRaw, majorPeak, magnitude |
| **`MoonModulesAudioDriver`** | Local I2S microphone or codec | bands, volume, volumeRaw, majorPeak, magnitude |
| `FastLEDAudioDriver` | Local I2S (FastLED pipeline) | fl_bass/mid/treble/bpm/beat/kick/snare/hihat/tom |

`MoonModulesAudio` and `FastLEDAudio` write *different* fields so they are not directly interchangeable; effects using `sharedData.bands` (WLED-style) need this driver, not FastLED Audio.

---

## 2. Library: WLED-AudioReactive-Usermod OO-split

**Repository:** `https://github.com/netmindz/WLED-AudioReactive-Usermod`  
**Branch:** `OO-split`  
**Pinned commit:** `e24dbea18274ef8943cc66b7aad92b7b36b234d5` (2026-06-07)

The library provides four C++ classes:

| Header | Class | WLED deps? | Usage |
|---|---|---|---|
| `audio_filters.h` | `AudioFilters` | None | Pre-process samples (DC blocker, PDM bandpass) |
| `agc_controller.h` | `AGCController` | None | Automatic gain control (PI controller) |
| `audio_processor.h` | `AudioProcessor` | None (`ISampleSource` interface) | FFT, 16-band EQ, peak detection; runs its own FreeRTOS task |
| `audio_source.h` | `AudioSource`, `I2SSource`, codec subclasses | Yes — `PinManager`, `i2c_sda/scl`, debug macros | I2S/PDM/ADC/codec audio capture |

The library was specifically refactored (in commit `e24dbea`) so that `AudioProcessor` depends only on the `ISampleSource` abstract interface, not `AudioSource`. This allows MoonLight to use the first three headers cleanly without any compatibility work. The fourth header (`audio_source.h`) is still used here because it provides all supported mic/codec types; its WLED-specific symbols are handled by a compatibility shim (see §4).

**arduinoFFT dependency:** `kosme/arduinoFFT @ 2.0.1` — declared in the library's `library.json` and must also be explicit in MoonLight's build config.

**`override_sqrt.py`:** PlatformIO will run this script automatically (it patches `sqrt` for performance on Xtensa). This is expected and harmless.

---

## 3. Build system changes

### `platformio.ini` — new `[moonlight-audio]` section

```ini
[moonlight-audio]
build_flags =
  -D FT_MOONLIGHT_AUDIO=1
lib_deps =
  https://github.com/netmindz/WLED-AudioReactive-Usermod.git#e24dbea18274ef8943cc66b7aad92b7b36b234d5 ; OO-split 20260607
  kosme/arduinoFFT @ 2.0.1  ; required by wled-audioreactive
```

### `firmware/esp32-s3.ini` — add to `[esp32-s3-base]`

```ini
build_flags =
  ...existing...
  ${moonlight-audio.build_flags}
lib_deps =
  ...existing...
  ${moonlight-audio.lib_deps}
```

### `firmware/esp32-p4.ini` — add to `[esp32-p4-base]`

Same additions as S3.

D0 and C3 boards are **not** modified — the library, arduinoFFT, and the FFT FreeRTOS task would exceed their available flash/RAM.

---

## 4. WLED compatibility shim

`audio_source.h` uses several WLED-specific symbols. These are defined inside `D_MoonModulesAudio.h` **before** the `#include <audio_source.h>` line.

### 4a. Debug macros

All WLED debug macros are no-opped. They are used throughout `audio_source.h` with both plain string literals and `F("...")` (flash strings), making them incompatible with ESP-IDF's `EXT_LOG*` format macros. The ESP-IDF I2S driver logs its own errors at the system level, so these macros only suppress usermod-specific debug output.

```cpp
#define DEBUGSR_PRINT(x)    (void)0
#define DEBUGSR_PRINTLN(x)  (void)0
#define DEBUGSR_PRINTF(...) (void)0
#define ERRORSR_PRINT(x)    (void)0
#define ERRORSR_PRINTLN(x)  (void)0
#define ERRORSR_PRINTF(...) (void)0
#define USER_PRINT(x)       (void)0
#define USER_PRINTLN(x)     (void)0
#define USER_PRINTF(...)    (void)0
#define USER_FLUSH()        (void)0
#define DEBUG_PRINT(x)      (void)0
#define DEBUG_PRINTLN(x)    (void)0
#define DEBUG_PRINTF(...)   (void)0
#define PLOT_PRINT(x)       (void)0
#define PLOT_PRINTLN(x)     (void)0
#define PLOT_PRINTF(...)    (void)0
#define PLOT_FLUSH()        (void)0
```

The driver's own `startService()` / `stopService()` report status via `updateControl("status", ...)` and `EXT_LOG*`.

### 4b. PinManager stub

`audio_source.h` calls `PinManager::allocatePin()` / `deallocatePin()` / `joinWire()`. In MoonLight, ModuleIO owns pin tracking and I2C initialisation, so these calls are no-ops (for allocate/deallocate) or delegate to `Wire.begin()` (for joinWire, called from I2C codec types when `WLEDMM` is defined).

```cpp
enum class PinOwner { UM_Audioreactive = 40 };

namespace PinManager {
  inline bool allocatePin(int8_t, bool, PinOwner) { return true; }
  inline void deallocatePin(int8_t, PinOwner) {}
  inline bool joinWire(int8_t sda, int8_t scl) { Wire.begin(sda, scl); return true; }
}
```

`WLEDMM` is **not** defined, so the `#ifdef WLEDMM joinWire()` path in I2C codec initializers is skipped. I2C bus initialisation is handled by MoonLight's ModuleIO before the codec's `initialize()` is called (see §6).

### 4c. Global I2C pin variables

I2C codec types (ES7243, ES8388, WM8978, AC101, ES8311) read `i2c_sda` and `i2c_scl` global variables. These are declared as `inline` (C++17 ODR-safe) variables with a sentinel value of `-1`.

```cpp
inline int8_t i2c_sda = -1;
inline int8_t i2c_scl = -1;
```

The driver sets them from ModuleIO state immediately before calling an I2C codec's `initialize()`.

---

## 5. Node identity and controls

```
name()     → "MoonModules Audio"
dim()      → _NoD
tags()     → "☸️♫"
category() → "Driver"
```

### Controls

| Variable | UI type | Default | Range | Description |
|---|---|---|---|---|
| `micType` | select | 1 | 0–10 | Microphone / codec type (see §7) |
| `soundAgc` | select | 1 | 0–3 | AGC mode: Off / Normal / Vivid / Lazy |
| `squelch` | slider | 10 | 0–100 | Noise gate threshold |
| `gain` | slider | 60 | 0–255 | Manual gain (used when AGC is off) |
| `inputLevel` | slider | 128 | 0–255 | Input level scaling |
| `limiter` | checkbox | true | — | Attack/decay envelope on volume and FFT |
| `status` | text | "No pins" | — | Read-only status string |

`status` is updated by the driver (`updateControl("status", ...)`) to reflect lifecycle states: `No pins` → `Starting` → `Running` → `Stopped` → `Error: <reason>`.

---

## 6. Pin management

Follows the exact pattern of `D_FastLEDAudio.h`.

```cpp
uint8_t pinSD   = UINT8_MAX;
uint8_t pinWS   = UINT8_MAX;
uint8_t pinSCK  = UINT8_MAX;  // UINT8_MAX → PDM mode (2-pin)
uint8_t pinMCLK = UINT8_MAX;  // UINT8_MAX → no MCLK
```

In `setup()`, register a ModuleIO update handler:
```cpp
ioUpdateHandler = moduleIO->addUpdateHandler([this](const String&) { readPins(); });
readPins();  // explicit call: node added at runtime, no initial IO update received
```

In `readPins()`:
```cpp
bool changed  = moduleIO->updatePin(pinWS,   pin_I2S_WS);
     changed  = moduleIO->updatePin(pinSD,   pin_I2S_SD)   || changed;
     changed  = moduleIO->updatePin(pinSCK,  pin_I2S_SCK)  || changed;
     changed  = moduleIO->updatePin(pinMCLK, pin_I2S_MCLK) || changed;
if (changed) {
  stopService();
  if (pinSD != UINT8_MAX && pinWS != UINT8_MAX) startService();
  else updateControl("status", "No pins");
}
```

In `~MoonModulesAudioDriver()`:
```cpp
stopService();
moduleIO->removeUpdateHandler(ioUpdateHandler);
```

**I2C codec pins:** For mic types that need I2C (ES7243=2, ES8388=6, WM8978=7, AC101=8, ES8311=9), the driver reads the global I2C SDA/SCL from ModuleIO state before calling `initialize()`. It first checks that `I2CReady` is true (same pattern as `D_IMU.h`):
```cpp
moduleIO->read([&](ModuleState& state) {
  i2c_sda = (int8_t)state.data["sdaPin"].as<int>();
  i2c_scl = (int8_t)state.data["sclPin"].as<int>();
}, name());
if (i2c_sda < 0 || i2c_scl < 0) {
  updateControl("status", "Error: I2C not configured");
  return;
}
```

---

## 7. Mic type mapping

Mirrors `AudioReactive::createAudioSource()` from `audio_reactive.h`. The `micType` select control has values 0–10; the corresponding `AudioSource` subclass is instantiated in `startService()`.

| `micType` | Label | `AudioSource` class | Pins used | Notes |
|---|---|---|---|---|
| 0 | ADC Analog | `I2SAdcSource` | SD (as ADC pin) | Classic ESP32 only |
| 1 | Generic I2S | `I2SSource` | SD, WS, SCK | Default; INMP441, ICS43434 etc |
| 2 | ES7243 | `ES7243` | SD, WS, SCK, MCLK + I2C | Requires I2C |
| 3 | SPH0645 | `SPH0654` | SD, WS, SCK | |
| 4 | I2S + MCLK | `I2SSource(1/24)` | SD, WS, SCK, MCLK | |
| 5 | PDM | `I2SSource(1/4)` | SD, WS | No SCK |
| 6 | ES8388 | `ES8388Source` | SD, WS, SCK, MCLK + I2C | Requires I2C |
| 7 | WM8978 | `WM8978Source` | SD, WS, SCK, MCLK + I2C | Requires I2C |
| 8 | AC101 | `AC101Source` | SD, WS, SCK, MCLK + I2C | Requires I2C |
| 9 | ES8311 | `ES8311Source` | SD, WS, SCK, MCLK + I2C | Requires I2C; tested on ESP32-P4 |
| 10 | Legacy PDM | `I2SSource(1/1)` | SD, WS | Older PDM mics |

The SCK-pin-absent auto-detection from `audio_reactive.h` (if SCK == `I2S_PIN_NO_CHANGE` and mic type is 1 or 4, force to type 10) is replicated in `startService()` to preserve compatibility.

**Sample rate and block size:** `SAMPLE_RATE = 22050`, `BLOCK_SIZE = 128` (same constants as `audio_reactive.h` for S3 and classic ESP32).

---

## 8. Audio pipeline configuration

Called from `startService()` after `audioSource` is created. Mirrors `AudioReactive::configureAudioLibraries()`.

### AudioFilters

```cpp
AudioFilters::Config filterCfg;
filterCfg.filterMode = 2;  // DC blocker (default)
if (micType == 5 || micType == 10) filterCfg.filterMode = 1;  // PDM → bandpass
if (micType == 0)                  filterCfg.filterMode = 1;  // ADC → bandpass
filterCfg.micQuality = 1;
audioFilters.configure(filterCfg);
```

### AGCController

```cpp
AGCController::Config agcCfg;
agcCfg.preset     = (AGCController::Preset)((soundAgc > 0) ? soundAgc - 1 : 0);
agcCfg.squelch    = squelch;
agcCfg.sampleGain = gain;
agcCfg.inputLevel = inputLevel;
agcCfg.micQuality = 1;
agcController.configure(agcCfg);
agcController.setEnabled(soundAgc > 0);
```

### AudioProcessor

```cpp
AudioProcessor::Config procCfg;
procCfg.sampleRate    = SAMPLE_RATE;
procCfg.fftSize       = 512;
procCfg.numGEQChannels = 16;
procCfg.scalingMode   = 3;   // sqrt scaling (matches audio_reactive.h default FFTScalingMode=3)
procCfg.limiterOn     = limiter;
procCfg.attackTime    = 50;
procCfg.decayTime     = 300;
procCfg.useInputFilter = filterCfg.filterMode;
audioProcessor.configure(procCfg);
audioProcessor.setAudioSource(audioSource);
audioProcessor.setAudioFilters(&audioFilters);
audioProcessor.setAGCController(&agcController);
```

### Starting the FFT task

```cpp
if (!audioProcessor.initialize()) {
  updateControl("status", "Error: FFT init failed");
  return;
}
if (!audioProcessor.startTask(1 /*priority*/, 0 /*core*/)) {
  updateControl("status", "Error: FFT task failed");
  return;
}
```

The FFT task runs on Core 0 at priority 1. This matches `FFTTASK_PRIORITY=1` (non-FASTPATH default). Core 0 is also used by MoonLight's `effectTask`, but at a lower priority, so the FFT task gets CPU time as needed.

---

## 9. `loop()` — publishing to `sharedData`

Called every frame from the driver task (Core 1, every ~20 ms). The FFT task (Core 0) runs independently and writes results to `AudioProcessor`'s internal buffers; `loop()` only reads those buffers.

```cpp
void loop() override {
  if (!running) return;

  // 16-band FFT result (0–255 per band)
  memcpy(sharedData.bands, audioProcessor.getFFTResult(), sizeof(sharedData.bands));

  // Volume
  bool agcOn = (soundAgc > 0);
  float vol  = agcOn ? agcController.getSampleAGC() : agcController.getSampleAvg();
  if (limiter) audioProcessor.limitSampleDynamics(vol);
  sharedData.volume    = vol;
  sharedData.volumeRaw = agcOn
    ? (int16_t)agcController.getRawSampleAGC()
    : agcController.getSampleRaw();

  // Peak frequency and magnitude
  sharedData.majorPeak = audioProcessor.getMajorPeak();
  sharedData.magnitude = audioProcessor.getMagnitude();
}
```

`sharedData` fields **not** written by this driver (they remain as-is from whatever last set them):
- `sharedData.fps`, `connectedClients`, etc. (system, set by sveltekit layer)
- `sharedData.gravity` (set by `D_IMU.h`)
- `sharedData.fl_*` (set by `D_FastLEDAudio.h`)

---

## 10. `stopService()` — teardown

```cpp
void stopService() {
  if (!running) return;
  audioProcessor.stopTask();   // waits for FFT task to exit
  if (audioSource) {
    audioSource->deinitialize();
    delete audioSource;
    audioSource = nullptr;
  }
  running = false;
  updateControl("status", "Stopped");
}
```

Called from `readPins()` on pin change, from `onUpdate()` on `micType` change, and from the destructor.

---

## 11. `onUpdate()` — reacting to control changes

```cpp
void onUpdate(const JsonObject& ctrl) override {
  const char* ctrlName = ctrl["name"];
  if (strcmp(ctrlName, "micType") == 0) {
    // Full restart: different AudioSource class needed
    stopService();
    if (pinSD != UINT8_MAX && pinWS != UINT8_MAX) startService();
  } else if (running) {
    // Reconfigure in-place (no restart needed):
    // soundAgc, squelch, gain, inputLevel, limiter
    // Re-call configureAudioPipeline() which calls .configure() on AGC/filters/processor
    reconfigurePipeline();
  }
}
```

`reconfigurePipeline()` extracts the configure calls from `startService()` without re-creating `audioSource` or restarting the FFT task.

---

## 12. Node registration

### `src/MoonBase/Nodes.h`

Inside the `#if FT_MOONLIGHT` block, alongside the other driver includes:

```cpp
#include "MoonLight/Nodes/Drivers/D_MoonModulesAudio.h"
```

### `src/MoonLight/Modules/ModuleDrivers.h`

In `addNodes()` (builds the UI dropdown):
```cpp
#if FT_MOONLIGHT_AUDIO
addNodeValue<MoonModulesAudioDriver>(control);
#endif
```

In `addNode()` (instantiates a node by name, e.g. when loading saved state):
```cpp
#if FT_MOONLIGHT_AUDIO
if (!node) node = checkAndAlloc<MoonModulesAudioDriver>(name);
#endif
```

---

## 13. Interaction with other drivers

### `D_WLEDAudio.h`

Writes the same `sharedData` fields. Both can coexist in the node list, but their results will overwrite each other each frame (last writer wins). In practice users should add one or the other, not both.

### `D_FastLEDAudio.h`

Uses a different audio pipeline (FastLED's) and writes to `sharedData.fl_*` fields. Writes to different fields from `MoonModulesAudio`, so technically they can coexist. However, **both use `I2S_NUM_0`** — only one I2S driver can be installed at a time. If both are active simultaneously the second `i2s_driver_install()` call will fail; the driver that starts second will log an error and set its status to `Error: ...`. Users should add only one I2S audio driver.

---

## 14. Known limitations and future work

**Initial implementation:**
- `micType` select is exposed but mic types requiring I2C (2, 6, 7, 8, 9) require that ModuleIO has already initialised the I2C bus and that `sdaPin`/`sclPin` are available in ModuleIO state. If not configured, these types will report an error and not start.
- No audio sync transmit (UDP out). The UDP transmit path from `AudioReactive::loop()` is not ported. If needed, `AudioSync` from the library could be added as a future enhancement.
- `FFTScalingMode`, `pinkIndex`, `freqDist`, `fftWindow`, sliding FFT toggle are not exposed as controls. They use defaults matching `audio_reactive.h` (scalingMode=3, pinkIndex=0, freqDist=0, fftWindow=0, no sliding window). Can be added as "advanced" controls in a future iteration.
- The `AudioReactive` audio palette generation (`getCRGBForBand` / `getAudioPalette`) from `D_WLEDAudio.h` is not included in this driver. It was ported to `WLEDAudioDriver` and remains there.

**Future:**
- Expose `FFTScalingMode` and `pinkIndex` as advanced controls
- Add UDP audio transmit (share local audio to other devices on the network)
- Consider `loop20ms()` instead of `loop()` to reduce overhead (the FFT task already runs independently; `loop()` is just copying results)
- If `D_WLEDAudio.h`'s `loopAudioDriver()` stub is eventually removed, this driver is its replacement

---

## 15. File header

```cpp
/**
    @title     MoonLight
    @file      D_MoonModulesAudio.h
    @repo      https://github.com/MoonModules/MoonLight, submit changes to this file as PRs
    @Authors   https://github.com/MoonModules/MoonLight/commits/main
    @Doc       https://moonmodules.org/MoonLight/moonlight/overview/
    @Copyright © 2026 GitHub MoonLight Commit Authors
    @license   GNU GENERAL PUBLIC LICENSE Version 3, 29 June 2007
    @license   For non GPL-v3 usage, commercial licenses must be purchased. Contact us for more information.
**/
```
