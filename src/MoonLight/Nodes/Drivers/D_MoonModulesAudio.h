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

#pragma once

#if FT_MOONLIGHT_AUDIO

// audio_source_factory.h provides AudioSourceConfig, createAudioSource(), and
// normalizeDmType().  It transitively includes audio_source.h (all driver
// classes) and audio_processor.h (ISampleSource).
#include <audio_source_factory.h>
#include <audio_filters.h>
#include <agc_controller.h>
#include <audio_processor.h>

static constexpr uint16_t MMA_SAMPLE_RATE = 22050;
static constexpr int      MMA_BLOCK_SIZE  = 128;

/// Convert MoonLight's UINT8_MAX pin sentinel to I2S_PIN_NO_CHANGE (-1).
static inline int8_t toI2SPin(uint8_t p) {
  return (p == UINT8_MAX) ? I2S_PIN_NO_CHANGE : static_cast<int8_t>(p);
}

/// MoonModulesAudio driver — captures audio locally from an I2S microphone or codec
/// and processes it through the WLEDMM audio pipeline (filters → AGC → FFT → 16-band EQ),
/// writing results into sharedData (same fields as D_WLEDAudio.h).
///
/// Supported mic types (micType control, 0–10):
///   0 = ADC Analog   1 = Generic I2S   2 = ES7243 (I2C)   3 = SPH0645
///   4 = I2S + MCLK   5 = PDM           6 = ES8388 (I2C)   7 = WM8978 (I2C)
///   8 = AC101 (I2C)  9 = ES8311 (I2C)  10 = Legacy PDM
///
/// S3 and P4 only — gated by FT_MOONLIGHT_AUDIO.
class MoonModulesAudioDriver : public Node {
 public:
  static const char* name()     { return "MoonModules Audio"; }
  static uint8_t     dim()      { return _NoD; }
  static const char* tags()     { return "☸️♫"; }
  static const char* category() { return "Driver"; }

 private:
  // --- Pins (UINT8_MAX = not assigned) ---
  uint8_t pinSD   = UINT8_MAX;
  uint8_t pinWS   = UINT8_MAX;
  uint8_t pinSCK  = UINT8_MAX;   // UINT8_MAX → PDM mode (no SCK)
  uint8_t pinMCLK = UINT8_MAX;   // UINT8_MAX → no MCLK

  // --- Controls ---
  uint8_t  micType    = 1;    // 0–10, see class comment
  uint8_t  soundAgc   = 1;    // 0=off, 1=Normal, 2=Vivid, 3=Lazy
  uint8_t  squelch    = 10;
  uint8_t  gain       = 60;
  uint8_t  inputLevel = 128;
  bool     limiter    = true;
  Char<32> status     = "No pins";

  // --- Audio pipeline ---
  AudioSource*   audioSource   = nullptr;
  AudioFilters   audioFilters;
  AGCController  agcController;
  AudioProcessor audioProcessor;

  bool                running        = false;
  update_handler_id_t ioUpdateHandler;

 public:
  void setup() override {
    addControl(micType, "micType", "select", 0, 10);
    addControlValue("ADC Analog");
    addControlValue("Generic I2S");
    addControlValue("ES7243");
    addControlValue("SPH0645");
    addControlValue("I2S + MCLK");
    addControlValue("PDM");
    addControlValue("ES8388");
    addControlValue("WM8978");
    addControlValue("AC101");
    addControlValue("ES8311");
    addControlValue("Legacy PDM");

    addControl(soundAgc, "soundAgc", "select", 0, 3);
    addControlValue("Off");
    addControlValue("Normal");
    addControlValue("Vivid");
    addControlValue("Lazy");

    addControl(squelch,    "squelch",    "slider",   0, 100);
    addControl(gain,       "gain",       "slider",   0, 255);
    addControl(inputLevel, "inputLevel", "slider",   0, 255);
    addControl(limiter,    "limiter",    "checkbox");
    addControl(status,     "status",     "text",     0, 32, true);

    ioUpdateHandler = moduleIO->addUpdateHandler([this](const String&) { readPins(); });
    readPins();  // Node added at runtime: no initial IO update received; call explicitly.
  }

  void readPins() {
    if (safeModeMB) {
      EXT_LOGW(ML_TAG, "Safe mode enabled, not adding pins");
      return;
    }

    bool changed  = moduleIO->updatePin(pinWS,   pin_I2S_WS);
         changed  = moduleIO->updatePin(pinSD,   pin_I2S_SD)   || changed;
         changed  = moduleIO->updatePin(pinSCK,  pin_I2S_SCK)  || changed;
         changed  = moduleIO->updatePin(pinMCLK, pin_I2S_MCLK) || changed;

    if (changed) {
      stopService();
      if (pinSD != UINT8_MAX && pinWS != UINT8_MAX) startService();
      else updateControl("status", "No pins");
    }
  }

  void startService() {
    if (running) stopService();
    updateControl("status", "Starting");
    EXT_LOGI(ML_TAG, "MoonModulesAudio starting micType=%d WS:%d SD:%d SCK:%d MCLK:%d",
             micType, pinWS, pinSD, pinSCK, pinMCLK);

    // Pin conversions (UINT8_MAX → I2S_PIN_NO_CHANGE)
    const int8_t ws   = toI2SPin(pinWS);
    const int8_t sd   = toI2SPin(pinSD);
    const int8_t sck  = toI2SPin(pinSCK);
    const int8_t mclk = toI2SPin(pinMCLK);

    // Read I2C pins for codec types that need them.
    // micType values: 2=ES7243, 6=ES8388, 7=WM8978, 8=AC101, 9=ES8311
    int8_t sdaPin = -1, sclPin = -1;
    // Resolve effective type first so the I2C check uses the right value.
    // MoonLight uses 10 for Legacy PDM; the factory maps 10→51 internally,
    // but normalizeDmType also handles PDM auto-promotion (types 1/4, no SCK).
    const uint8_t eff = normalizeDmType(micType == 10 ? 51 : micType, sck);
    const bool needsI2C = (eff == 2 || eff == 6 || eff == 7 || eff == 8 || eff == 9);
    if (needsI2C) {
      moduleIO->read([&](ModuleState& state) {
        sdaPin = static_cast<int8_t>(state.data["sdaPin"].as<int>());
        sclPin = static_cast<int8_t>(state.data["sclPin"].as<int>());
      }, name());
      if (sdaPin < 0 || sclPin < 0) {
        updateControl("status", "Error: I2C not configured");
        EXT_LOGE(ML_TAG, "I2C pins not configured for micType %d", eff);
        return;
      }
    }

    // Instantiate and initialise the AudioSource via the shared factory.
    // nullptr allocator is safe — _allocatePin helpers skip when allocator is null.
    AudioSourceConfig cfg;
    cfg.dmType    = micType;   // factory handles the 10→51 alias internally
    cfg.sampleRate = MMA_SAMPLE_RATE;
    cfg.blockSize  = MMA_BLOCK_SIZE;
    cfg.i2swsPin   = ws;
    cfg.i2ssdPin   = sd;
    cfg.i2sckPin   = sck;
    cfg.mclkPin    = mclk;
    cfg.i2c_sda    = sdaPin;
    cfg.i2c_scl    = sclPin;

    audioSource = createAudioSource(cfg);  // allocator defaults to nullptr

    if (!audioSource) {
      updateControl("status", "Error: source alloc failed");
      return;
    }

    // Configure and start the audio pipeline
    configurePipeline();

    if (!audioProcessor.initialize()) {
      updateControl("status", "Error: FFT init failed");
      return;
    }
    if (!audioProcessor.startTask(1 /*priority*/, 0 /*core*/)) {
      updateControl("status", "Error: FFT task failed");
      return;
    }

    running = true;
    updateControl("status", "Running");
    EXT_LOGI(ML_TAG, "MoonModulesAudio running");
  }

  /// Configure AudioFilters, AGCController, and AudioProcessor.
  /// Called from startService() and reconfigurePipeline().
  void configurePipeline() {
    const uint8_t eff = ((micType == 1 || micType == 4) && pinSCK == UINT8_MAX) ? 10 : micType;

    // AudioFilters: PDM and ADC use bandpass; everything else uses DC blocker.
    AudioFilters::Config filterCfg;
    filterCfg.filterMode = (eff == 0 || eff == 5 || eff == 10) ? 1 : 2;
    filterCfg.micQuality = 1;
    audioFilters.configure(filterCfg);

    // AGCController
    AGCController::Config agcCfg;
    agcCfg.preset     = static_cast<AGCController::Preset>((soundAgc > 0) ? soundAgc - 1 : 0);
    agcCfg.squelch    = static_cast<float>(squelch);
    agcCfg.sampleGain = static_cast<float>(gain);
    agcCfg.inputLevel = inputLevel;
    agcCfg.micQuality = 1;
    agcController.configure(agcCfg);
    agcController.setEnabled(soundAgc > 0);

    // AudioProcessor
    AudioProcessor::Config procCfg;
    procCfg.sampleRate      = MMA_SAMPLE_RATE;
    procCfg.fftSize         = 512;
    procCfg.numGEQChannels  = 16;
    procCfg.scalingMode     = 3;   // sqrt scaling (matches audio_reactive.h FFTScalingMode=3)
    procCfg.limiterOn       = limiter;
    procCfg.attackTime      = 50;
    procCfg.decayTime       = 300;
    procCfg.useInputFilter  = filterCfg.filterMode;
    audioProcessor.configure(procCfg);
    audioProcessor.setAudioSource(audioSource);
    audioProcessor.setAudioFilters(&audioFilters);
    audioProcessor.setAGCController(&agcController);
  }

  /// Reconfigure AGC and filter in-place (no FFT task restart).
  /// Called from onUpdate() for soundAgc / squelch / gain / inputLevel / limiter changes.
  void reconfigurePipeline() {
    if (!running) return;

    const uint8_t eff = ((micType == 1 || micType == 4) && pinSCK == UINT8_MAX) ? 10 : micType;

    AudioFilters::Config filterCfg;
    filterCfg.filterMode = (eff == 0 || eff == 5 || eff == 10) ? 1 : 2;
    filterCfg.micQuality = 1;
    audioFilters.configure(filterCfg);

    AGCController::Config agcCfg;
    agcCfg.preset     = static_cast<AGCController::Preset>((soundAgc > 0) ? soundAgc - 1 : 0);
    agcCfg.squelch    = static_cast<float>(squelch);
    agcCfg.sampleGain = static_cast<float>(gain);
    agcCfg.inputLevel = inputLevel;
    agcCfg.micQuality = 1;
    agcController.configure(agcCfg);
    agcController.setEnabled(soundAgc > 0);
  }

  void stopService() {
    audioProcessor.stopTask();   // safe to call even if task was never started
    if (audioSource) {
      audioSource->deinitialize();
      delete audioSource;
      audioSource = nullptr;
    }
    if (running) {
      running = false;
      updateControl("status", "Stopped");
      EXT_LOGI(ML_TAG, "MoonModulesAudio stopped");
    }
  }

  /// Called every frame from the driver task (Core 1).
  /// The FFT task (Core 0) writes results to AudioProcessor's internal buffers;
  /// loop() only reads those and copies to sharedData.
  void loop() override {
    if (!running) return;

    // 16-band FFT result (0–255 per band)
    memcpy(sharedData.bands, audioProcessor.getFFTResult(), sizeof(sharedData.bands));

    // Volume
    const bool agcOn = (soundAgc > 0);
    float vol = agcOn ? agcController.getSampleAGC() : agcController.getSampleAvg();
    if (limiter) audioProcessor.limitSampleDynamics(vol);
    sharedData.volume    = vol;
    sharedData.volumeRaw = agcOn
        ? static_cast<int16_t>(agcController.getRawSampleAGC())
        : agcController.getSampleRaw();

    // Peak frequency and magnitude
    sharedData.majorPeak = audioProcessor.getMajorPeak();
    sharedData.magnitude = audioProcessor.getMagnitude();
  }

  void onUpdate(const JsonObject& control) override {
    if (control["name"] == "micType") {
      // Full restart: different AudioSource class required.
      stopService();
      if (pinSD != UINT8_MAX && pinWS != UINT8_MAX) startService();
    } else if (running) {
      // In-place reconfigure: soundAgc, squelch, gain, inputLevel, limiter.
      // (limiter change is also reflected immediately in loop() via the member variable.)
      reconfigurePipeline();
    }
  }

  ~MoonModulesAudioDriver() override {
    stopService();
    moduleIO->removeUpdateHandler(ioUpdateHandler);
  }
};

#endif  // FT_MOONLIGHT_AUDIO
