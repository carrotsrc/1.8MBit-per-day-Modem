#include <math.h>
#include <fftw3.h>
#include <sstream>
#include "common.hpp"
#include "jack_client.hpp"
#include "sync.hpp"

#define THRESHOLD 20
#define CONTROL_THRESHOLD 10
#define CONTROL_RESET 80
#define BOOSTER 1


#define FS 44100.0f


enum FREQ {
  CONTROL = 0,
  ZEROA = 1,
  ZEROB = 2,
  ONEA = 3,  
  ONEB = 4
};

void design_bpf(unsigned int num_taps, float* taps, float phi, float lambda) {
    
    for(auto n = 0u; n < num_taps; n++) {
        auto mm = n - ( static_cast<float>(num_taps) - 1.0f ) / 2.0f;
        if(mm == 0.0) {
            taps[n] = (phi - lambda) / M_PI; 
        } else {
            taps[n] = ( sinf(mm * phi) - sinf(mm * lambda) ) / (mm*M_PI);
        }
    }
}

float* bpf(unsigned int num_taps, float fs, float fl, float fu) {
    auto lambda = M_PI * fl / (fs/2.0f);
    auto phi = M_PI * fu /  (fs/2.0f);
    
    auto taps = new float[num_taps];
    design_bpf(num_taps, taps, phi, lambda);
    return taps;
}

float* create_buffer(unsigned int num_taps) {
    return new float[num_taps];
}

float filter(float sample, float* taps, float* buffer, unsigned int num_taps) {
    
    auto result = float(0.0);
    for(auto i = num_taps-1; i >=1; i--) {
        buffer[i] = buffer[i-1];
    }
    
    buffer[0] = sample;
    
    for(auto  i = 0u; i < num_taps; i++) {
        result += buffer[i] * taps[i];
    }
    
    return result;
}

struct payload {
  enum status {
    waiting, control_start, writing, control_end
  };
  
  sfsw::jack::client* client;
  float* sample_buffer;
  float* taps;
  float* output_buffer;
  std::uint8_t* data_buffer;
  FILE* fp;
  struct {
    fftw_plan plan;
    double* in;
    fftw_complex* out;
    unsigned int bins[5];
    float avg;
  } fftw;
  
  unsigned int state;
  unsigned int reset;
  unsigned int bit_period;
  
  status cstate;
  
  char ctl_marker;
  int bit_num;
  
};

#define NUM_TAPS 51


struct magnitudes {
  float fc, f0a, f0b, f1a, f1b;
};

void handle_zero(magnitudes& mag, payload& pl) {
  if(mag.f0a > THRESHOLD) {
    if(pl.ctl_marker != 0) {
      pl.data_buffer[pl.bit_num] = 0;
      pl.bit_num++;
    }
    pl.ctl_marker = 0;
  } else if(mag.f0b > THRESHOLD) {
    if(pl.ctl_marker != '0') {
     pl.data_buffer[pl.bit_num] = 0;
     pl.bit_num++;
    }
    pl.ctl_marker = '0';
  } 
}

void handle_one(magnitudes& mag, payload& pl) {
  if(mag.f1a > THRESHOLD) {
    if(pl.ctl_marker != 1) {
      pl.data_buffer[pl.bit_num] = 1;
      pl.bit_num++;
    }
    pl.ctl_marker = 1;
  } else if(mag.f1b > THRESHOLD) {
    if(pl.ctl_marker != '1') {
      pl.data_buffer[pl.bit_num] = 1;
      pl.bit_num++;
    }
    pl.ctl_marker = '1';
  }
}

std::string decode_bin(std::uint8_t* bin, unsigned int num_bits) {
  int bp = 0;
  char ch = 0x00;
  std::stringstream s;
  for(auto b = 0; b < num_bits; b++) {
    ch |= bin[b] << bp;
    if(++bp == 8) {
      bp = 0;
      s << static_cast<char>(ch);
      ch = 0x00;
    }
  }
  
  return s.str();
}

void resolve_state(magnitudes& mag, payload& pl) {

  if(mag.f0a > THRESHOLD | mag.f0b > THRESHOLD) {
    handle_zero(mag, pl);
  } else if(mag.f1a > THRESHOLD || mag.f1b > THRESHOLD) {
    handle_one(mag, pl);
  } else if(mag.fc > CONTROL_THRESHOLD) {
    pl.ctl_marker = 'c';
    if(pl.bit_num > 0) {
      std::stringstream s;
      for(auto b = 0u; b < pl.bit_num; b++) {
        s << std::to_string(pl.data_buffer[b]);
      }
      
      std::cout << "read " << pl.bit_num << " bits\n";
      std::cout << "\n" << s.str() << "\n";
      std::cout << "\n\n" << decode_bin(pl.data_buffer, pl.bit_num) << "\n\n\n\n\n\n" << std::endl;
    }
    pl.bit_num = 0;
  } 
}

inline float window_hanning(unsigned int n, unsigned int N) {
  constexpr float pi2 = 2 * M_PI;
  return 0.5f - (0.5f * cosf( pi2 * n / N));
}

void print_magnitudes(magnitudes& mag) {
    std::cout << "\e[A\e[A\e[A\e[A\e[A" 
      <<               "fc:      " << mag.fc 
      << "            \nf0a:      " << mag.f0a
      << "            \nf0b:      " << mag.f0b
      << "            \nf1a:      " << mag.f1a
      << "            \nf1b:      " << mag.f1b
      << "            \n" << std::flush;
  };
int process(jack_nframes_t nframes, void* data) {
  auto pl = static_cast<payload*>(data);
  

  auto port_in = pl->client->ports().audio_input("in", sfsw::audio::stereo_channel::left);
  auto port_out = pl->client->ports().audio_output("out", sfsw::audio::stereo_channel::left);
  
  if(port_in == nullptr || port_out == nullptr){ return 1; }
  
  auto in_buffer = static_cast<float*>(jack_port_get_buffer(port_in, nframes));
  auto out_buffer = static_cast<float*>(jack_port_get_buffer(port_out, nframes));
  if(pl->output_buffer == nullptr) {
    pl->output_buffer = new float[nframes];
    pl->fftw.in = new double[nframes];
    pl->fftw.out = static_cast<fftw_complex*>(fftw_malloc( sizeof(fftw_complex) * nframes ));
    pl->fftw.plan = fftw_plan_dft_r2c_1d(nframes, pl->fftw.in, pl->fftw.out, FFTW_ESTIMATE);
  }

  for(auto i = 0u; i < nframes; i++) {
    auto filtered = filter(in_buffer[i], pl->taps, pl->sample_buffer, NUM_TAPS);
    filtered *= window_hanning(i, nframes);
    pl->output_buffer[i] =  filtered;
    pl->fftw.in[i] = filtered;
  }
  fftw_execute(pl->fftw.plan);
  
  magnitudes mag;  
  
  auto f0a = pl->fftw.out[ pl->fftw.bins[FREQ::ZEROA] ][0];
  auto f0b = pl->fftw.out[ pl->fftw.bins[FREQ::ZEROB] ][0];
  auto fc  = pl->fftw.out[ pl->fftw.bins[FREQ::CONTROL] ][0];
  auto f1a = pl->fftw.out[ pl->fftw.bins[FREQ::ONEA] ][0];
  auto f1b = pl->fftw.out[ pl->fftw.bins[FREQ::ONEB] ][0];
  mag.f0a = (f0a < 0.0 ? f0a * -1.0 : f0a) * BOOSTER;
  mag.f0b = (f0b < 0.0 ? f0b * -1.0 : f0b) * BOOSTER;
  mag.f1a = (f1a < 0.0 ? f1a * -1.0 : f1a) * BOOSTER;
  mag.f1b = (f1b < 0.0 ? f1b * -1.0 : f1b) * BOOSTER;
  mag.fc  = fc < 0.0 ? fc * -1.0 : fc;
  
  resolve_state(mag, *pl);

  print_magnitudes(mag); 

  return 0;
}



int main(int argc, char** argv) {
    sfsw::jack::client client("wivi-demod");
   
    payload pl;
    pl.client = &client;
    pl.cstate = payload::waiting;
    pl.ctl_marker = 'w';
    pl.data_buffer = new std::uint8_t[1024];
    
    constexpr int binwidth = FS / 2048;
        
    pl.fftw.bins[FREQ::CONTROL] = static_cast<unsigned int>( FC0 / binwidth);
    pl.fftw.bins[FREQ::ZEROA] = static_cast<unsigned int>( F0A / binwidth);
    pl.fftw.bins[FREQ::ONEA] = static_cast<unsigned int>( F1A / binwidth);
    pl.fftw.bins[FREQ::ZEROB] = static_cast<unsigned int>( F0B / binwidth);
    pl.fftw.bins[FREQ::ONEB] = static_cast<unsigned int>( F1B / binwidth);
    pl.taps = bpf(NUM_TAPS,44.1,0.3,0.9);
    pl.sample_buffer = create_buffer(NUM_TAPS);

    std::cout << "fc : " << FC0 << "Hz\t" << "(" << pl.fftw.bins[FREQ::CONTROL] << ")\n";
    std::cout << "f0a: " << F0A << "Hz\t" << "(" << pl.fftw.bins[FREQ::ZEROA] << ")\n";
    std::cout << "f0b: " << F0B << "Hz\t" << "(" << pl.fftw.bins[FREQ::ZEROB] << ")\n";
    std::cout << "f1a: " << F1A << "Hz\t" << "(" << pl.fftw.bins[FREQ::ONEA] << ")\n";
    std::cout << "f1b: " << F1B << "Hz\t" << "(" << pl.fftw.bins[FREQ::ONEB] << ")\n";
    std::cout << "Waiting for signal...\n\n\n\n\n" << std::endl;    


    pl.output_buffer = nullptr;
    
    pl.fftw.in = nullptr;
    pl.fftw.out = nullptr;
    
    client.set_process_data(static_cast<void*>(&pl));
    client.open(process, nullptr);
    client.ports().register_audio_inputs("in");
    client.ports().register_audio_outputs("out");
    client.activate();
    
    std::this_thread::sleep_until(std::chrono::time_point<std::chrono::system_clock>::max());
    return (EXIT_SUCCESS);
}

