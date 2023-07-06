#include "simple-daisy.h"
#include "looper.h"

DaisyHardware pod;

// Setup pins (Daisy Pod)
static const int record_pin      = D(S42); // Switch 1. Pin 34 D27 S42
static const int clear_loop_pin  = D(S43); // Switch 2. Pin 35 D28 S43
static const int loop_start_pin  = A(S36); // Pot 1.    Pin 28 A6  S36
static const int loop_length_pin = A(S30); // Pot 2.    Pin 22 A0  S30

static const float kKnobMax = 1023;

// Allocate buffer in SDRAM 
static const uint32_t kBufferLengthSec = 10;
static const uint32_t kSampleRate = 48000;
static const size_t kBufferLengthSamples = kBufferLengthSec * kSampleRate;
static float DSY_SDRAM_BSS buffer_l[kBufferLengthSamples];
static float DSY_SDRAM_BSS buffer_r[kBufferLengthSamples];

// Create objects from classes
static synthux::Looper looper_l;
static synthux::Looper looper_r;
static PitchShifter pitch_shifter;
static Chorus chorus;
//static DelayLine<float, 48000> delay_line; // may delay time of 1 second
//static ReverbSc rev;

// Set the pitch val to be 0.5 i.e. no pitch transposition
float pitch_val_default = 0.0;
float pitch_val = pitch_val_default;
bool pitch_val_changed = false;
auto dryWet = 0;

float chorusLfoFreq_default = 0.0;
float chorusLfoFreq = chorusLfoFreq_default;
bool chorusLfoFreq_changed = false;

// Set the delay time in samples
//float delay_time = 6860;
//float wet_delay;

int buttonCurrentState;
bool recordingActive = false;

float dryAmplitude = 0.5;
float wetAmplitude = 0.5;

// Array to hold the ID of each effect control, plus the associated LED colour
float effects[3][3] = {
    {0.25f, 0.0f, 1.0f},  // Magenta: Pitch Shift
    {0.0f, 0.25f, 0.75f}, // Cyan: Chorus
    {1.0f, 0.33f, 0.0f}   // Yellow: Dry/Wet
    };
int numEffects = sizeof(effects) / sizeof(effects[0]);
int selectedEffect = 0;

// Use this to limit the possible encoder values
int dryWetEncoderMax = 21;


void AudioCallback(float **in, float **out, size_t size) {
  for (size_t i = 0; i < size; i++) {
    auto looper_out_l = looper_l.Process(in[0][i]);
    auto looper_out_r = looper_l.Process(in[1][i]);

    auto pitch_shifter_out_l = pitch_shifter.Process(looper_out_l);
    auto pitch_shifter_out_r = pitch_shifter.Process(looper_out_r);

    auto chorus_out_l = chorus.Process(pitch_shifter_out_l);
    auto chorus_out_r = chorus.Process(pitch_shifter_out_r);

    out[0][i] = (looper_out_l * dryAmplitude) + (chorus_out_l * wetAmplitude);
    out[1][i] = (looper_out_r * dryAmplitude) + (chorus_out_r * wetAmplitude);

    // Keep this in for now... I might use it later
    // Pitch shifting causes a slight delay and a leval drop
    // Therefore, we add delay to non pitch-shifted channel so both channels are in sync
    // ... then times the 0.65 to reduce the amplitude
    //wet_delay = delay_line.Read();
    //delay_line.Write(looper_out_r);
    //out[1][i] = wet_delay * 0.55;
    //out[0][i] = chorus.Process(pitch_shifter_out_l);

  }
}

void setup() {
  
  //  Init Daisy Pod
  pod = DAISY.init(DAISY_POD, AUDIO_SR_48K);
  
  pod.leds[0].Set(false, false, false);
  pod.leds[1].Set(false, false, false);

  float sample_rate = DAISY.get_samplerate();

  // Setup looper
  looper_l.Init(buffer_l, kBufferLengthSamples);
  looper_r.Init(buffer_r, kBufferLengthSamples);

  // Setup pitch shifter
  pitch_shifter.Init(sample_rate);

  // Setup Chorus
  chorus.Init(sample_rate);

  // Setup Reverb
  //rev.Init(sample_rate);

  // Setup Delay line
  //delay_line.Init();
  //delay_line.SetDelay(delay_time);

  // Setup pins
  pinMode(record_pin, INPUT);

  // Start comms with Daisy hardware
  DAISY.begin(AudioCallback);
}

void loop() {

  // Set loop parameters
  auto loop_start = fmap(analogRead(loop_start_pin) / kKnobMax, 0.f, 1.f);
  auto loop_length = fmap(analogRead(loop_length_pin) / kKnobMax, 0.f, 1.f, Mapping::EXP);
  //looper.SetLoop(loop_start, loop_length);
  looper_l.SetLoop(loop_start, loop_length);
  looper_r.SetLoop(loop_start, loop_length);

  // Read button states
  UpdateButtons();

  looper_l.SetRecording(recordingActive);
  looper_r.SetRecording(recordingActive);
  pod.leds[0].Set(recordingActive, 0, 0);

  //pod.leds[1].Set(0,0,1);
  pod.leds[1].Set(0, 0, 0);
  pod.leds[1].Set(effects[selectedEffect][0],effects[selectedEffect][1],effects[selectedEffect][2]);

  pitch_shifter.SetTransposition(pitch_val);
  chorus.SetLfoFreq(chorusLfoFreq);
  chorus.SetLfoDepth(chorusLfoFreq * 0.1, chorusLfoFreq * 0.05);
  //reverb parameters
  // rev.SetLpFreq(18000.0f);
  // rev.SetFeedback(0.85f);

}

  void UpdateButtons() {

    pod.DebounceControls();

    // Short button 1 press toggles recording on/off
    if (pod.buttons[0].RisingEdge()) {
      recordingActive = !recordingActive;
      buttonCurrentState = LOW;
    }

    // Long press (> 2s) clears the recording buffer
    if (pod.buttons[0].TimeHeldMs() >= 2000 && buttonCurrentState == LOW) {
      recordingActive = false;
      looper_l.ClearBuffer();
      looper_r.ClearBuffer();
      pod.leds[0].Set(0, 255, 0);
      System::Delay(100);
      pod.leds[0].Set(0, 0, 0);
      buttonCurrentState = HIGH;
    }

    // Button3 press cycles through effects
    if (pod.buttons[1].RisingEdge()) {
      selectedEffect += 1;
      if (selectedEffect == numEffects){
        selectedEffect = 0;
      }
    }

    if (selectedEffect == 0) {

      // adjust pitch shifter params
      if (pod.encoder.RisingEdge()) {
        pitch_val = pitch_val_default;
      }
      pitch_val += pod.encoder.Increment();
    
    } else if (selectedEffect == 1) {
      
      // adjust chorus params
      if (pod.encoder.RisingEdge()) {
        chorusLfoFreq = chorusLfoFreq_default;
      }
      chorusLfoFreq += (pod.encoder.Increment() * 0.5);

    } else if (selectedEffect == 2) {

      // hmmm, not sure yet. Maybe adjust the dry/wet?
      if (pod.encoder.RisingEdge()) {
        dryAmplitude = 0.5;
        wetAmplitude = 0.5;
      }
      
      // Dry wet will always be a value between 0 and encoderMax - 1
      auto previousDryWet = dryWet;
      dryWet += (pod.encoder.Increment());
      dryWet = ((dryWet % dryWetEncoderMax + dryWetEncoderMax) % dryWetEncoderMax);

      // Make sure the encoder does not roll over from max to min or min to max
      // In other words, try and make the encoder act like a pot!
      if (previousDryWet == dryWetEncoderMax - 1 && dryWet == 0) {
        dryWet = dryWetEncoderMax - 1;
      } else if (previousDryWet == 0 && dryWet == dryWetEncoderMax -1) {
        dryWet = 0;
      }

      if (previousDryWet != dryWet) {
        dryAmplitude = 1 - (1 * dryWet * 0.05);
        wetAmplitude = (1 * (dryWet * 0.05) * 2); // The wet needs a boost to avoid low levels when 100% wet
      }

    }

  }
