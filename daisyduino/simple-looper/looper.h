#pragma once

namespace synthux {

class Looper {
  public:

    static const long kBufferLengthSec = 5;
    static const long kSampleRate = 48000;
    static const long kBufferLenghtSamples = kBufferLengthSec * kSampleRate;

    void Init(float sample_rate, float *buf, long length) {
      _buffer = buf;
      _buffer_length = length;
      // Reset buffer contents to zero
      memset(_buffer, 0, sizeof(float) * _buffer_length);
    }

    void SetRecording(bool is_rec_on) {
        //Initialize recording head position on start
        if (_rec_ramp_pos_inc <= 0 && is_rec_on) {
            _rec_head = (_loop_start + _play_head) % _buffer_length; 
            _is_empty = false;
        }
        // When record switch changes state it effectively
        // sets ramp to rising/falling, providing a
        // fade in/out in the beginning and at the end of 
        // the recorded region.
        _rec_ramp_pos_inc = is_rec_on ? 1 : -1;
    }

    void SetLoop(const float loop_start, const float loop_length) {
      _pending_loop_start = static_cast<long>(loop_start * (_buffer_length - 1));
      if (_loop_start == -1) _loop_start = _pending_loop_start;
      _loop_length = max(kMinLoopLength, static_cast<long>(loop_length * _buffer_length));
    }
  
    float Process(float in) {
      // Calculate iterator position on the ramp.
      if (_rec_ramp_pos_inc > 0 && _rec_ramp_pos < kFadeLength
       || _rec_ramp_pos_inc < 0 && _rec_ramp_pos > 0) {
          _rec_ramp_pos += _rec_ramp_pos_inc;
      }
      // If we're in the middle of the ramp - record to the buffer.
      if (_rec_ramp_pos > 0) {
        // Calculate fade in/out
        float rec_attenuation = static_cast<float>(_rec_ramp_pos) / static_cast<float>(kFadeLength);
        auto rec_pos = _rec_head % _buffer_length;
        _buffer[rec_pos] = in * rec_attenuation + _buffer[rec_pos] * (1.f - rec_attenuation);
        _rec_head ++;
      }
      
      if (_is_empty) {
        return 0;
      }

      // Playback from the buffer
      float attenuation = 1;
      float output = 0;
      //Calculate fade in/out
      if (_play_head < kFadeLength) {
        attenuation = static_cast<float>(_play_head) / static_cast<float>(kFadeLength);
      }
      else if (_play_head >= _loop_length - kFadeLength) {
        attenuation = static_cast<float>(_loop_length - _play_head) / static_cast<float>(kFadeLength);
      }
      
      // Read from the buffer
      auto play_pos = (_loop_start + _play_head) % _buffer_length;
      output = _buffer[play_pos] * attenuation;

      // Advance playhead
      if (++_play_head >= _loop_length) {
        _loop_start = _pending_loop_start;
        _play_head = 0;
      }
      
      return output * attenuation;
    }

  private:
    const long kFadeLength = 600;
    const long kMinLoopLength = 2 * kFadeLength;

    float* _buffer;
    
    long _buffer_length       = 0;
    long _loop_length         = 0;
    long _loop_start          = -1;
    long _pending_loop_start  = 0;

    long _play_head = 0;
    long _rec_head  = 0;

    long _rec_ramp_pos      = 0;
    long _rec_ramp_pos_inc  = 0;
    bool _is_empty  = true;
};
};
