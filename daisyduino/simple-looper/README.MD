# Simple Looper

Simple looper for the Daisy platform.

This looper was adapted for the Daisy pod based on the original code by VLADYSLAV LYTVYNENKO here: https://github.com/Synthux-Academy/simple-examples/tree/main/daisyduino/simple-looper.

## Adaptations:

1. Updated control mappings for Daisy pod
2. Added chorus effect
3. Added dry/wet control for chorus and pitch-shift
4. It's now stereo!
5. Added reverse playback option (Click encoder in Yellow mode)

## Controls:

- Button 1: Short-press to toggle recording on and off
- Button 1: Long-press (> 2s) clears the recording buffer
- Button 2: Cycle through encoder functions
	- LED 2: Magenta: Pitch Shift (rotate: pitch, click: return to 0)
	- LED 2: Cyan: Chorus (rotate: lfoFreq + lfoDepth, click: reset to 0)
	- LED 2: Yellow: Dry/Wet, Reverse playback (rotate: dry/wet balance, click: toggle reverse playback)
- Pot 1: Loop start position
- Pot 2: Loop length
- Encoder Press: Reset effect control

## Potential future enhancements

- Tweak chorus effect
- (If on-board memory will permit) Add another effect (maybe a filter?)
- Save to / Load from SD Card
