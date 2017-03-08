#include <cstdlib>
#include <iostream>
#include <sstream>
#include <cmath>
#include "jack_client.hpp"
#include "sync.hpp"

#define CONTROL_TIME 20
#define DATA_LOOP 16
std::string char_to_bits(char ch) {
  std::stringstream bits;
  for(auto bit = 0u; bit < 8; bit++) {
    bits << ( (ch >> bit) & 0x1);
  }
  return bits.str();
}

struct info {
  enum status {
    waiting, header, data_cue, data, footer
  };
  info(float fc, float f0a, float f0b, float f1a, float f1b, std::string input)
    : fc(fc), phi_delta_fc( (2.0f * M_PI * fc) / 44100.0f )
    , f0a(f0a), phi_delta_f0a( (2.0f * M_PI * f0a) / 44100.0f )
    , f0b(f0b), phi_delta_f0b( (2.0f * M_PI * f0b) / 44100.0f )
    , f1a(f1a), phi_delta_f1a( (2.0f * M_PI * f1a) / 44100.0f )
    , f1b(f1b), phi_delta_f1b( (2.0f * M_PI * f1b) / 44100.0f )
    , phi_delta(0), phi(0.0f)
    , period_count(0), bit(0)
    , client(nullptr)
    , cstate(status::waiting)
    , data_loop(0)
    , input(input)
    , data_len(input.length())
  { }
  float fc, phi_delta_fc;
  float f0a, phi_delta_f0a;
  float f0b, phi_delta_f0b;
  float f1a, phi_delta_f1a;
  float f1b, phi_delta_f1b;
  float phi_delta;
  float phi;
  unsigned int period_count, bit;
  sfsw::jack::client* client;
  status cstate;
  unsigned int data_loop;
  std::string input;
  std::string char_bits;
  unsigned int data_len;
};

int process(jack_nframes_t nframes, void* data) {

 static float pi2 = 2 * M_PI;
  
  auto sys = static_cast<info*>(data);
  
  auto port_l = sys->client->ports().audio_output("output", sfsw::audio::stereo_channel::left);
  auto port_r = sys->client->ports().audio_output("output", sfsw::audio::stereo_channel::right);
  if(port_l == nullptr || port_r == nullptr) { return 1; }
  
  auto bufl = static_cast<float*>(jack_port_get_buffer(port_l, nframes));
  auto bufr = static_cast<float*>(jack_port_get_buffer(port_r, nframes));
  
  if(sys->cstate == info::waiting) {
    sys->phi_delta = 0.0f;
  } else if(sys->cstate == info::header) {
    sys->phi_delta = sys->phi_delta_fc;
    if(sys->period_count == CONTROL_TIME) {
      sys->cstate = info::data_cue;
      sys->period_count = 0;
    } else {
      sys->period_count++;
    }
  } else if(sys->cstate == info::footer) {
    sys->phi_delta = sys->phi_delta_fc;
    if(sys->period_count == CONTROL_TIME) {
      sys->cstate = info::waiting;
      sys->period_count = 0;
    } else {
      sys->period_count++;
    }
  } else if(sys->cstate == info::data_cue) {
    sys->cstate = info::data;
    sys->phi_delta = 0;
    sys->data_loop = 0;
  }


  if(sys->data_loop == sys->data_len && sys->cstate == info::data) {
    sys->cstate = info::footer;
    sys->period_count = 0;
    std::cout << "\nModulated " << sys->data_len << " bits\n";
  }
  
  if(sys->cstate == info::data) {
    if(sys->period_count == BIT_WIDTH) {
      auto ch = sys->input.at(sys->data_loop);
      
      if(ch == '0') {
        if(sys->phi_delta == sys->phi_delta_f0a) {
          sys->phi_delta = sys->phi_delta_f0b;
          std::cout << "0";
        } else {  
          sys->phi_delta = sys->phi_delta_f0a;
          std::cout << "0"; 
        }
        
      } else if(ch == '1') {
        if(sys->phi_delta == sys->phi_delta_f1a) {
          sys->phi_delta = sys->phi_delta_f1b;
          std::cout << "1";
        } else {       
          sys->phi_delta = sys->phi_delta_f1a;
          std::cout << "1";
        }
      }
      sys->data_loop++;
      sys->period_count = 0;
    } else {
      sys->period_count++;
    }
  }

  for(auto i =0u; i < nframes; i++) {
    auto y = sinf( sys->phi );
    bufl[i] = y;
    bufr[i] = y;
    sys->phi += sys->phi_delta;
    
    // Completed 1 cycle = 2pi radians
    if(sys->phi > pi2) {
      sys->phi -= pi2;
    }
  }
 
  return 0;
}

int main(int argc, char** argv) {

  std::string input;
  std::getline(std::cin, input);
  std::string bits = "";
  std::cout << "Running modulator...\n";
  auto client = sfsw::jack::client("wivi-mod");

  for(auto& ch : input) {
    bits.append( char_to_bits(ch) );
  }
  info i(FC0, F0A,F0B, F1A, F1B, bits);
  i.client = &client;

  client.set_process_data(static_cast<void*>(&i));
  client.open(process,nullptr);
  client.ports().register_audio_outputs("output");
  i.cstate = info::waiting;
  client.activate();
  while(jack_port_connected_to(client.ports().audio_output("output", sfsw::audio::stereo_channel::left), "wivi-demod:in_l") == 0) {
    continue;
  }
  

  std::cout << "Connected" << std::endl;
  i.cstate = info::header;
  while(i.cstate != info::waiting){ continue; }
  return (EXIT_SUCCESS);
}

