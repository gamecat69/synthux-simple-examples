
#include "simple-daisy.h"
#include "looper.h"

DaisyHardware hw;

// Setup pins (Synthux Board)
//static const int record_pin      = D(S30);
//static const int loop_start_pin  = A(S31);
//static const int loop_length_pin = A(S32);
//static const int pitch_pin       = A(S33);

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
static DelayLine<float, 48000> delay_line; // may delay time of 1 second

// Set the pitch val to be 0.5 i.e. no pitch transposition
auto pitch_val = 0.5;

// Set the delay time in samples
float delay_time = 6860;
float wet_delay;

void AudioCallback(float **in, float **out, size_t size) {
  for (size_t i = 0; i < size; i++) {
    auto looper_out_l = looper_l.Process(in[0][i]);
    auto looper_out_r = looper_l.Process(in[1][i]);
    // Apply a pitch shift and chorus effect to only the left channel
    // This either creates a wild stereo effect, or a clean and effected channel for mixing together
    auto pitch_shifter_out_l = pitch_shifter.Process(looper_out_l);    

    // Pitch shifting causes a slight delay and a leval drop
    // Therefore, we add delay to non pitch-shifted channel so both channels are in sync
    // ... then times the 0.65 to reduce the amplitude
    wet_delay = delay_line.Read();
    delay_line.Write(looper_out_r);
    out[1][i] = wet_delay * 0.55;
    out[0][i] = chorus.Process(pitch_shifter_out_l);

  }
}

void setup() {
  //  Init Daisy Seed
  //DAISY.init(DAISY_SEED, AUDIO_SR_48K);
  
  //  Init Daisy Pod
  hw = DAISY.init(DAISY_POD, AUDIO_SR_48K);
  hw.leds[0].Set(false, false, false);
  hw.leds[1].Set(false, false, false);

  float sample_rate = DAISY.get_samplerate();

  // Setup looper
  looper_l.Init(buffer_l, kBufferLengthSamples);
  looper_r.Init(buffer_r, kBufferLengthSamples);

  // Setup pitch shifter
  pitch_shifter.Init(sample_rate);

  // Setup Chorus
  chorus.Init(sample_rate);

  // Setup Delay line
  delay_line.Init();
  //  Set the delay time based on the pitch_shifter processing delay
  delay_line.SetDelay(delay_time);

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

  // Toggle record - Press and hold to record. Recording stops when button is released
  // Note for the Daisy Pod the button is the opposite, therefore the value is !record_on
  hw.DebounceControls();
  // Synthux Board:
  // auto record_on = digitalRead(record_pin);
  // Daisy Pod:
  auto record_on = !digitalRead(record_pin);
  //looper.SetRecording(record_on);
  looper_l.SetRecording(record_on);
  looper_r.SetRecording(record_on);
  // Daisy Pod only, turn on LED when recording
  hw.leds[0].Set(record_on, 0, 0);
  
  // Set pitch
  //  Original code:
  //auto pitch_val = fmap(analogRead(pitch_pin) / kKnobMax, 0.f, 1.f);
  // Updated for Daisy Pod Encoder:
  // Reset if encoder pressed
  if (hw.encoder.RisingEdge()) {
    pitch_val = 0.5;
  }
  //  Adjust pitch when Daisy Pod encoder if used
  pitch_val += hw.encoder.Increment();
  //pitch_val += (hw.encoder.Increment() * 0.1);
  //set_pitch(pitch_val);
  pitch_shifter.SetTransposition(pitch_val);

  // Clear loop if button 2 is pressed
  if (digitalRead(clear_loop_pin) == LOW) {
    memset(buffer_l, 0, sizeof(float) * kBufferLengthSamples);
    memset(buffer_r, 0, sizeof(float) * kBufferLengthSamples);
    hw.leds[1].Set(0, 255, 0);
    System::Delay(100);
    hw.leds[1].Set(0, 0, 0);
  }
  
}

// void set_pitch(float pitch_val) {
//   int pitch = pitch_val;
//   // Allow some gap in the middle of the knob turn so 
//   // it's easy to cacth zero position
//   // if (pitch_val < 0.45 || pitch_val > 0.55) {
//   //   pitch = 12.0 * (pitch_val - 0.5);
//   // }
//   pitch_shifter.SetTransposition(pitch);
// }