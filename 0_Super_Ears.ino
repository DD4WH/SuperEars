/*
 * Super Ears
 * 
 * Compensating age-related high frequency loss for bird and Orthoptera field work
 * optional: automatic heterodyne detector for bats
 * 
 * Pitch shifting in the time domain for high pitched birds & Orthoptera
 * & auto heterodyne mode for ultrasound, eg. bats & Tettigoniidae
 * 
 * implements pitch shifting in the time domain and is based on an idea by Lang Elliott & Herb Susmann for the "Hear birds again"-project,
 * specifically for the now deprecated SongFinder pitch shifter units. 
 * The algorithm can shift the audio down by a factor of two (one octave), three (1.5 octaves) or four (two octaves). 
 * Find a graph showing the implementation for the case of downshifting by 4 here: https://github.com/DD4WH/BirdSongPitchShifter#readme
 *  * Many thanks go to Harold Mills & Lang Elliott for explaining this algorithm to me and answering my questions ! :-) 
 * https://hearbirdsagain.org/
 * 
 * Many thanks to Jean-Do Vrignault for the Teensy Recorder code!
 * 
 * (c) Frank DD4WH 2022-10-05
 *  this version from 2024-07-23
 *  latest version from 2025-09-19
 * 
 * TODO:
 * - use FIR filters instead of IIR to preserve phase information for stereo locating (is this important???)
 * - use FFT results to search for highest peak frequency and tune the internal oscillator for auto-heterodyne mode
 * 
 * 
 * uses Teensy 4.1 and external ADC / DAC connected on perf board with ground plane
 * 
 *  PCM5102A DAC module
    VCC = Vin
    3.3v = NC
    GND = GND
    FLT = GND
    SCL = 23 / MCLK via series 100 Ohm
    BCK = BCLK (21)
    DIN = TX (7)
    LCK = LRCLK (20)
    FMT = GND
    XMT = 3.3V (HIGH)
    
    PCM1808 ADC module:    
    FMT = GND
    MD1 = GND
    MD0 = GND
    GND = GND
    3.3V = 3.3V --> ADC needs both: 5V AND 3V3
    5V = VIN
    BCK = BCLK (21) via series 100 Ohm
    OUT = RX (8)
    LRC = LRCLK (20) via series 100 Ohm
    SCK = MCLK (23) via series 100 Ohm
    GND = GND
    3.3V = 3.3V
 *  
 *  OLED display SSH1106
 *  
 *  Vcc = Vcc 3.3V
 *  GND = GND
 *  SCL 19
 *  SDA 18
 *  SCL & SDA each with a pull-up 4K7 resistor to 3V3 
 *  
 *  Potentiometer  pin 17
 *  Buttons
 *  UP      30 
 *  DOWN    32
 *  PUSH    31
 *    
 *  
 *  
 * MIT license
 */

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <Bounce.h>
#include <arm_math.h>
#include <arm_const_structs.h> // in the Teensy 4.0 audio library, the ARM CMSIS DSP lib is already a newer version
#include <utility/imxrt_hw.h>
#include <U8g2lib.h>

#define DEBUG
#define PIH 1.5707963267948966192313216916398f
#define FOURPI  (2.0 * TWO_PI)
#define SIXPI   (3.0 * TWO_PI)

// Screen manager
U8G2 *Display = NULL;

extern "C" volatile uint32_t set_arm_clock(uint32_t frequency);

int32_t sum = 0;
int32_t elapsed_micros_idx_t = 0;
int32_t elapsed_micros_mean = 0;
int32_t elapsed_micros_sum = 0;
float32_t mean = 0.0f;
bool adjust_LO = false;

int16_t *sp_L;
int16_t *sp_R;
//double SAMPLE_RATE = 44100;
double SAMPLE_RATE = 96000; // standard sample rate
//double SAMPLE_RATE = 192000;
float32_t vol_knob_gain_dB = 0.0;
float32_t audio_gain = 2.0f; 

// DD4WH: 3 buttons used
#define PINPUSH  31 //DD4WH
#define PINUP    30 //DD4WH
#define PINDOWN  32 //DD4WH
#define POTI     17 //DD4WH 
Bounce push_button = Bounce(PINPUSH, 15);
Bounce up_button = Bounce(PINUP, 15);
Bounce down_button = Bounce(PINDOWN, 15);

// GUItool: begin automatically generated code
AudioInputI2S            i2sIN;          //xy=111.19999694824219,376
AudioAnalyzeFFT256       FFT256;       //xy=137.1999969482422,269
AudioFilterBiquad        biquadR;        //xy=320.1999969482422,343
AudioFilterBiquad        biquadL;        //xy=322.1999969482422,394
AudioRecordQueue         PitchShiftINR;  //xy=365.1999969482422,185
AudioSynthWaveformSine   SineL;          //xy=372.1999969482422,479
AudioRecordQueue         PitchShiftINL;  //xy=372.1999969482422,535
AudioSynthWaveformSine   SineR;          //xy=377.1999969482422,248
AudioPlayQueue           PitchShiftOUTR; //xy=520.2000122070312,185
AudioPlayQueue           PitchShiftOUTL; //xy=526.2000122070312,536
AudioEffectMultiply      multiplyR;      //xy=543.1999969482422,325
AudioEffectMultiply      multiplyL;      //xy=547.1999969482422,405
AudioMixer4              mixerR;         //xy=660.2000122070312,254
AudioMixer4              mixerL;         //xy=671.2000122070312,469
AudioAmplifier           gainR;           //xy=784.2000122070312,255.20001220703125
AudioAmplifier           gainL;           //xy=799.2000122070312,469.20001220703125
AudioFilterBiquad        biquadOUTL;        //xy=830.2000122070312,407.20001220703125
AudioFilterBiquad        biquadOUTR;        //xy=832.2000122070312,329.20001220703125
AudioOutputI2S           i2sOUT;         //xy=976.2000122070312,367
AudioConnection          patchCord1(i2sIN, 0, PitchShiftINR, 0);
AudioConnection          patchCord2(i2sIN, 0, biquadR, 0);
AudioConnection          patchCord3(i2sIN, 1, PitchShiftINL, 0);
AudioConnection          patchCord4(i2sIN, 1, biquadL, 0);
AudioConnection          patchCord5(biquadR, 0, multiplyR, 1);
//AudioConnection          patchCord6(biquadR, FFT256);
AudioConnection          patchCord6(i2sIN, 1, FFT256, 0);
AudioConnection          patchCord7(biquadL, 0, multiplyL, 0);
AudioConnection          patchCord8(SineL, 0, multiplyL, 1);
AudioConnection          patchCord9(SineR, 0, multiplyR, 0);
AudioConnection          patchCord10(PitchShiftOUTR, 0, mixerR, 0);
AudioConnection          patchCord11(PitchShiftOUTL, 0, mixerL, 0);
AudioConnection          patchCord12(multiplyR, 0, mixerR, 1);
AudioConnection          patchCord13(multiplyL, 0, mixerL, 1);
AudioConnection          patchCord14(mixerR, gainR);
AudioConnection          patchCord15(mixerL, gainL);
AudioConnection          patchCord16(gainR, biquadOUTR);
AudioConnection          patchCord17(gainL, biquadOUTL);
AudioConnection          patchCord18(biquadOUTL, 0, i2sOUT, 1);
AudioConnection          patchCord19(biquadOUTR, 0, i2sOUT, 0);

int shift = 2; // 1 = pass-thru, 5 = heterodyne, 2-4 = shift by one (2), one-and-a-half (3) or two octaves (4)
int shift_new = 2;
int lowest_shift = 1; // to allow pass-thru (beware of feedback !), set this to 1, otherwise set this to 2
#define BLOCK_SIZE 128
const int N_BLOCKS = 6; //9; // 6 blocks á 128 samples  = 768 samples. No. of samples has to be dividable by 2 AND by 3 AND by 4 // 6 blocks of 128 samples == 768 samples = 17.4ms of delay
const int WINDOW_LENGTH = N_BLOCKS * BLOCK_SIZE;
const int WINDOW_LENGTH_D_2 = WINDOW_LENGTH / 2;
const int IN_BUFFER_SIZE = WINDOW_LENGTH;
#define HOPSIZE_4 WINDOW_LENGTH/4
#define HOPSIZE_3 WINDOW_LENGTH/3
float32_t in_buffer_L[4][IN_BUFFER_SIZE];
float32_t in_buffer_R[4][IN_BUFFER_SIZE];
float32_t hp_buffer_R[IN_BUFFER_SIZE];
float32_t hp_buffer_L[IN_BUFFER_SIZE];
float32_t out_buffer_L[IN_BUFFER_SIZE];
float32_t out_buffer_R[IN_BUFFER_SIZE];
float32_t add_buffer_L[IN_BUFFER_SIZE / 2];
float32_t add_buffer_R[IN_BUFFER_SIZE / 2];
float32_t window[N_BLOCKS * BLOCK_SIZE];
int buffer_idx = 0;

int het_freq = 22000;
int last_LO_frequency=22000;
int FFT_max_freq = last_LO_frequency;
float32_t FFT_bin [128];
float32_t FFT_max = 0.0f;
float32_t FFT_mean = 0.0f;
uint32_t FFT_max_idx = 0; 
int16_t FFT_max1 = 0;
uint32_t FFT_max_bin1 = 0;
int16_t FFT_mean1 = 0;
int16_t FFT_max2 = 0;
uint32_t FFT_max_bin2 = 0;
int16_t FFT_mean2 = 0;
//int16_t FFT_threshold = 0;
int16_t FFT_bat [3]; // max of 3 frequencies are being displayed
int16_t index_FFT;
int l_limit;
int u_limit;
int index_l_limit;
int index_u_limit;
const uint16_t FFT_points = 256;
int barm [512];


const float32_t n_att = 70.0; // desired stopband attenuation, 80dB
const int num_taps = 72; // can be divided by 2, 3 and 4 
// interpolation-by-N
// num_taps has to be a multiple integer of interpolation factor L
// pState is of length (numTaps/L)+blockSize-1 words where blockSize is the number of input samples processed by each call
// phaseLength=numTaps/L
// state array of size blockSize + phaseLength - 1
arm_fir_interpolate_instance_f32 interpolation_R;
float32_t DMAMEM interpolation_R_state [num_taps / 2 + N_BLOCKS * BLOCK_SIZE / 2]; 
arm_fir_interpolate_instance_f32 interpolation_L;
float32_t DMAMEM interpolation_L_state [num_taps / 2 + N_BLOCKS * BLOCK_SIZE / 2]; 
float32_t DMAMEM interpolation_coeffs[num_taps];

const int num_taps_hp = 128; // 
arm_fir_instance_f32 highpass_R;
// Note that the length of the state buffer exceeds the length of the coefficient array by blockSize-1
float32_t DMAMEM highpass_R_state [num_taps_hp + N_BLOCKS * BLOCK_SIZE - 1]; 
arm_fir_instance_f32 highpass_L;
float32_t DMAMEM highpass_L_state [num_taps_hp + N_BLOCKS * BLOCK_SIZE - 1]; 
float32_t DMAMEM highpass_coeffs[num_taps_hp];

int hop0 = 0;
int hop1 = 1;
int hop2 = 2;
int hop3 = 3;

void setup() {
  Serial.begin(115200);
  delay(100);
  set_arm_clock(396000000);
  Serial.printf("Teensy 4.1 F_CPU %d, F_CPU_ACTUAL %d, F_BUS_ACTUAL %d\n", F_CPU, F_CPU_ACTUAL, F_BUS_ACTUAL);
  pinMode(PINPUSH, INPUT_PULLUP);
  pinMode(PINUP, INPUT_PULLUP);
  pinMode(PINDOWN, INPUT_PULLUP);

  AudioMemory(200); // must be high enough to deliver 16 stereo blocks = 32 * 128 samples at any point in time! 
  delay(100);

  /****************************************************************************************
     Audio Shield Setup
  ****************************************************************************************/
  setI2SFreq(SAMPLE_RATE);
  
  mixerL.gain(0, 1.0); // 0 == pitch shift, 1 == heterodyne
  mixerR.gain(0, 1.0); // 0 == pitch shift, 1 == heterodyne
  mixerL.gain(1, 0.0); // 0 == pitch shift, 1 == heterodyne
  mixerR.gain(1, 0.0); // 0 == pitch shift, 1 == heterodyne
  gainR.gain(1.0);
  gainL.gain(1.0);

  
  /****************************************************************************************
     display stuff
  ****************************************************************************************/

  Display = new U8G2_SH1106_128X64_NONAME_F_HW_I2C(U8G2_R0, U8X8_PIN_NONE);
  // Initialisation
  Display->begin();
  Display->setFontPosTop();
  Display->clearBuffer();
  Display->setFont(u8g2_font_6x10_mf);  // Hauteur 7
  Display->sendBuffer();
  Display->setContrast(255);

  Display->setFont(u8g2_font_6x13B_mf);  // Hauteur 13
  Display->setCursor(3,25);
  // Reading the recorder type
  Display->println("Super Ears by");
  Display->setCursor(3,37);
  Display->println("Frank DD4WH");
  Display->setFont(u8g2_font_micro_mr);  // Hauteur 5
  Display->setCursor( 3, 2);
  Display->print( "SH1106");
 
  // Print CPU speed
  char temp[20];
  sprintf(temp, "CPU %dMHz", (int)F_CPU_ACTUAL/1000000);
  Display->setCursor( 3, 9);
  Display->print( temp);
  // If problem, print SDFat
  if (MAINTAIN_FREE_CLUSTER_COUNT != 1)
  {
    Display->setCursor( 3, 16);
    Display->print( "SDFat cluster !");
  }
  Display->sendBuffer();
  delay(1000);
  
  Serial.println("DD4WH time domain pitch shifter following Hear Birds Again by Harolds Mills");
  display_mode();
  
  /****************************************************************************************
     prepare audio processing
  ****************************************************************************************/

  for(unsigned idx=0; idx < WINDOW_LENGTH; idx++)
  { // von Hann window
     window[idx] = 0.5f * (1.0f - cosf(TWO_PI * (float)idx / ((float)(WINDOW_LENGTH - 1))));  
    // Blackman-Nuttall
    //window[idx] = 0.3635819f - 0.4891775f*cosf(2.0*M_PI*(float)idx/((float)(WINDOW_LENGTH-1))) + 0.1365995*cosf(FOURPI*(float)idx/((float)(WINDOW_LENGTH-1))) - 0.0106411f*cosf(SIXPI*(float)idx/((float)(WINDOW_LENGTH-1)));  
  }

  init_FIR_filters();
  init_IIR_filters();
  //FFT256.windowFunction(AudioWindowBlackmanHarris256);
  //FFT256.averageTogether(16);
  
  delay(100);

  
  /****************************************************************************************
     start queues audio
  ****************************************************************************************/
 
  PitchShiftINL.begin(); // start the input queues for pitch shift 
  PitchShiftINR.begin();

} // END OF SETUP



void init_FIR_filters(void)
{
      // Interpolation filter
    // the interpolation low pass filter is AFTER the upsampling, so it has to be in the target sample rate!
    calc_FIR_coeffs (interpolation_coeffs, num_taps, (float32_t)5000, n_att, 0, 0.0, SAMPLE_RATE);
    init_interpolation_filter();

    // Highpass filter
    // void calc_FIR_coeffs (float * coeffs_I, int numCoeffs, float32_t fc, float32_t Astop, int type, float dfc, float Fsamprate)
    calc_FIR_coeffs (highpass_coeffs, num_taps_hp, (float32_t)5000, n_att, 1, 0.0, SAMPLE_RATE);
    arm_fir_init_f32(&highpass_R, num_taps, highpass_coeffs, highpass_R_state, BLOCK_SIZE * N_BLOCKS); 
    arm_fir_init_f32(&highpass_L, num_taps, highpass_coeffs, highpass_L_state, BLOCK_SIZE * N_BLOCKS); 
}

void init_IIR_filters(void)
{
    // output IIR filter -> lowpass
    // lowpass f == 5000Hz
    int fakeF = 8000 / (SAMPLE_RATE/AUDIO_SAMPLE_RATE_EXACT);
    // Linkwitz-Riley filter, 48 dB/octave
    biquadOUTR.setLowpass(0, fakeF, 0.54);
    biquadOUTR.setLowpass(1, fakeF, 1.3);
    biquadOUTR.setLowpass(2, fakeF, 0.54);
    biquadOUTR.setLowpass(3, fakeF, 1.3);
    biquadOUTL.setLowpass(0, fakeF, 0.54);
    biquadOUTL.setLowpass(1, fakeF, 1.3);
    biquadOUTL.setLowpass(2, fakeF, 0.54);
    biquadOUTL.setLowpass(3, fakeF, 1.3);  
    
    // input IIR filter -> highpass
    fakeF = 12000 / (SAMPLE_RATE/AUDIO_SAMPLE_RATE_EXACT);
    biquadR.setHighpass(0, fakeF, 0.54);
    biquadR.setHighpass(1, fakeF, 1.3);
    biquadR.setHighpass(2, fakeF, 0.54);
    biquadR.setHighpass(3, fakeF, 1.3);
    biquadL.setHighpass(0, fakeF, 0.54);
    biquadL.setHighpass(1, fakeF, 1.3);
    biquadL.setHighpass(2, fakeF, 0.54);
    biquadL.setHighpass(3, fakeF, 1.3);  
}

void init_heterodyne(void) {
  SAMPLE_RATE = 96000;
  //SAMPLE_RATE = 192000; // noise is very high
  setI2SFreq(SAMPLE_RATE);
  init_IIR_filters();
  PitchShiftINL.end(); // end the input queues for pitch shift 
  PitchShiftINR.end();
  PitchShiftINL.clear(); // end the input queues for pitch shift 
  PitchShiftINR.clear();
  mixerL.gain(1, 1.0); // 0 == pitch shift, 1 == heterodyne
  mixerR.gain(1, 1.0); // 0 == pitch shift, 1 == heterodyne
  mixerL.gain(0, 0.0); // 0 == pitch shift, 1 == heterodyne
  mixerR.gain(0, 0.0); // 0 == pitch shift, 1 == heterodyne
  // start oscillators
  SineL.frequency(het_freq/(SAMPLE_RATE/AUDIO_SAMPLE_RATE_EXACT));  
  SineR.frequency(het_freq/(SAMPLE_RATE/AUDIO_SAMPLE_RATE_EXACT));
  SineL.amplitude(0.9f);
  SineR.amplitude(0.9f);
  display_het_freq();
 }

void init_pitch_shift(void) {
  buffer_idx = 0;
  SAMPLE_RATE = 96000;
  setI2SFreq(SAMPLE_RATE);
  init_IIR_filters();
  init_FIR_filters(); // new
  init_interpolation_filter();
  PitchShiftINL.begin(); // start the input queues for pitch shift 
  PitchShiftINR.begin();
  mixerL.gain(0, 1.0); // 0 == pitch shift, 1 == heterodyne
  mixerR.gain(0, 1.0); // 0 == pitch shift, 1 == heterodyne
  mixerL.gain(1, 0.0); // 0 == pitch shift, 1 == heterodyne
  mixerR.gain(1, 0.0); // 0 == pitch shift, 1 == heterodyne
  SineL.amplitude(0.0f);
  SineR.amplitude(0.0f);
}

elapsedMillis memorytimer = 0;

void loop() {
elapsedMicros usec = 0;

  //check the potentiometer
//    servicePotentiometer(millis(),100); //service the potentiometer every 100 msec
    servicePotentiometer(millis(), 20); //service the potentiometer every 20 msec

    if(shift_new != shift) 
    {
      shift = shift_new;
      if (shift == 5)
      {
        init_heterodyne();
      }
      else 
      {
        init_pitch_shift();
      }
      Serial.print("Shift real is: "); Serial.println(shift);
      Serial.print("Sample Rate: "); Serial.println(SAMPLE_RATE);
    }
  
  // are there at least N_BLOCKS buffers in each channel available ?
    if (PitchShiftINL.available() > N_BLOCKS + 0 && PitchShiftINR.available() > N_BLOCKS + 0 && shift < 5)
    {
      usec = 0;
      // get audio samples from the audio  buffers and convert them to float
      for (unsigned i = 0; i < N_BLOCKS; i++)
      {
        sp_L = PitchShiftINL.readBuffer();
        sp_R = PitchShiftINR.readBuffer();

        // convert to float one buffer_size
        // float_buffer samples are now standardized from > -1.0 to < 1.0
        arm_q15_to_float (sp_L, &in_buffer_L[buffer_idx][BLOCK_SIZE * i], BLOCK_SIZE); // convert int_buffer to float 32bit
        arm_q15_to_float (sp_R, &in_buffer_R[buffer_idx][BLOCK_SIZE * i], BLOCK_SIZE); // convert int_buffer to float 32bit
        PitchShiftINL.freeBuffer();
        PitchShiftINR.freeBuffer();
      }
 
      /*********************************************************************************************************************
          Highpass Filter to supress low frequency noise and supress down-pitching sounds that can be heard with the ear 
       *********************************************************************************************************************/

       if(shift != 1)
       { 
          arm_fir_f32(&highpass_R, &in_buffer_R[buffer_idx][0], hp_buffer_R, BLOCK_SIZE * N_BLOCKS);
          arm_fir_f32(&highpass_L, &in_buffer_L[buffer_idx][0], hp_buffer_L, BLOCK_SIZE * N_BLOCKS);
          arm_copy_f32(hp_buffer_L,&in_buffer_L[buffer_idx][0], BLOCK_SIZE * N_BLOCKS);  
          arm_copy_f32(hp_buffer_R,&in_buffer_R[buffer_idx][0], BLOCK_SIZE * N_BLOCKS);  
       }
      /**********************************************************************************
          Time Domain pitch shift algorithm "OLA" by Harald Mills "Hear birds again" 
       **********************************************************************************/

      /**********************************************************************************
          1 Windowing 
       **********************************************************************************/

//      we apply the second half of the window to the first half of the input buffer
//      works for N==2 
        if(shift == 2)
        {
            for (unsigned i = 0; i < WINDOW_LENGTH_D_2; i++)
            {
              in_buffer_L[buffer_idx][i] = in_buffer_L[buffer_idx][i] * window[i + WINDOW_LENGTH_D_2];
              in_buffer_R[buffer_idx][i] = in_buffer_R[buffer_idx][i] * window[i + WINDOW_LENGTH_D_2];
            }
//      we apply the first half of the window to the second half of the input buffer
    
            for (unsigned i = 0; i < WINDOW_LENGTH_D_2; i++)
            {
              in_buffer_L[buffer_idx][i + WINDOW_LENGTH_D_2] = in_buffer_L[buffer_idx][i + WINDOW_LENGTH_D_2] * window[i];
              in_buffer_R[buffer_idx][i + WINDOW_LENGTH_D_2] = in_buffer_R[buffer_idx][i + WINDOW_LENGTH_D_2] * window[i];
            }
        }
        else if(shift == 3)
        {
           for (unsigned i = 0; i < WINDOW_LENGTH; i++)
           {
              in_buffer_L[buffer_idx][i] = in_buffer_L[buffer_idx][i] * window[i];
              in_buffer_R[buffer_idx][i] = in_buffer_R[buffer_idx][i] * window[i];
           }          
        }
        else if(shift == 4)
        {
           for (unsigned i = 0; i < WINDOW_LENGTH; i++)
           {
              in_buffer_L[buffer_idx][i] = in_buffer_L[buffer_idx][i] * window[i];
              in_buffer_R[buffer_idx][i] = in_buffer_R[buffer_idx][i] * window[i];
           }
        } // end shift == 4

    
      /**********************************************************************************
          2 Overlap & Add 
       **********************************************************************************/
        if(shift == 2)
        {
            for (unsigned i = 0; i < WINDOW_LENGTH_D_2; i++)
            {
              add_buffer_L[i] = in_buffer_L[buffer_idx][i] + in_buffer_L[buffer_idx][i + WINDOW_LENGTH_D_2];
              add_buffer_R[i] = in_buffer_R[buffer_idx][i] + in_buffer_R[buffer_idx][i + WINDOW_LENGTH_D_2];
            }
        }
        else if(shift == 3)
        {   // index of in_buffer    [0][x]  [1][x]  [2][x]  
            if(buffer_idx==2)       {hop0=2, hop1=1, hop2=0;}
            else if(buffer_idx==1)  {hop0=1, hop1=0, hop2=2;}
            else if(buffer_idx==0)  {hop0=0, hop1=2, hop2=1;}
            hop0=hop0*HOPSIZE_3;
            hop1=hop1*HOPSIZE_3;
            hop2=hop2*HOPSIZE_3;
            for (unsigned i = 0; i < HOPSIZE_3; i++)
            {
                add_buffer_L[i] = in_buffer_L[0][i + hop0] + in_buffer_L[1][i + hop1] + in_buffer_L[2][i + hop2]; 
                add_buffer_R[i] = in_buffer_R[0][i + hop0] + in_buffer_R[1][i + hop1] + in_buffer_R[2][i + hop2]; 
            }

            buffer_idx = buffer_idx + 1;  // increment
            if (buffer_idx >=3) {buffer_idx = 0;} // flip-over           
        }
        else if(shift == 4)
        {   // index of in_buffer    [0][x]  [1][x]  [2][x]  [3][x]
            if(buffer_idx == 3)     {hop0=3, hop1=2, hop2=1, hop3=0;}
            else if(buffer_idx==2)  {hop0=2, hop1=1, hop2=0, hop3=3;}
            else if(buffer_idx==1)  {hop0=1, hop1=0, hop2=3, hop3=2;}
            else if(buffer_idx==0)  {hop0=0, hop1=3, hop2=2, hop3=1;}
            hop0=hop0*HOPSIZE_4;
            hop1=hop1*HOPSIZE_4;
            hop2=hop2*HOPSIZE_4;
            hop3=hop3*HOPSIZE_4;
            for (unsigned i = 0; i < HOPSIZE_4; i++)
            {
                add_buffer_L[i] = in_buffer_L[0][i + hop0] + in_buffer_L[1][i + hop1] + in_buffer_L[2][i + hop2] + in_buffer_L[3][i + hop3]; 
                add_buffer_R[i] = in_buffer_R[0][i + hop0] + in_buffer_R[1][i + hop1] + in_buffer_R[2][i + hop2] + in_buffer_R[3][i + hop3]; 
            }

            buffer_idx = buffer_idx + 1;  // increment
            if (buffer_idx >=4) {buffer_idx = 0;} // flip-over  
        }

      /**********************************************************************************
          3 Interpolate 
       **********************************************************************************/

      // interpolation-in-place does not work
      // blocksize is BEFORE zero stuffing
      if(shift == 2 || shift == 3 || shift == 4)
      {
          arm_fir_interpolate_f32(&interpolation_L, add_buffer_L, out_buffer_L, BLOCK_SIZE * N_BLOCKS / (uint32_t)(shift));
          arm_fir_interpolate_f32(&interpolation_R, add_buffer_R, out_buffer_R, BLOCK_SIZE * N_BLOCKS / (uint32_t)(shift));
      }

      /**********************************************************************************
           
       **********************************************************************************/

      /**********************************************************************************
          Just copy input into output buffer for testing/pass-thru
       **********************************************************************************/
      if(shift == 1)
      {
        for (unsigned i = 0; i < BLOCK_SIZE * N_BLOCKS; i++)
        {
          out_buffer_L[i] = in_buffer_L[buffer_idx][i]; //hp_buffer_L[i];
          out_buffer_R[i] = in_buffer_R[buffer_idx][i]; //hp_buffer_R[i];
        } 
      }
       /**********************************************************************
          CONVERT TO INTEGER AND PLAY AUDIO - Push audio into I2S audio chain
       **********************************************************************/
      for (unsigned i = 0; i < N_BLOCKS; i++)
        {
          sp_L = PitchShiftOUTL.getBuffer();    
          sp_R = PitchShiftOUTR.getBuffer();
          arm_float_to_q15 (&out_buffer_L[BLOCK_SIZE * i], sp_L, BLOCK_SIZE); 
          arm_float_to_q15 (&out_buffer_R[BLOCK_SIZE * i], sp_R, BLOCK_SIZE);
          PitchShiftOUTL.playBuffer(); // play it !  
          PitchShiftOUTR.playBuffer(); // play it !
        }

      elapsed_micros_sum = elapsed_micros_sum + usec;
      elapsed_micros_idx_t++;
    } // end of pitch shift audio process loop

    if(shift == 5)
    {
        if (FFT256.available())
        {
            search_bats();

                /*//   this is for testing, whether the FFT sends data
                Serial.print("FFT: ");
                for (int i=0; i<30; i++) 
                {  // 0-25  -->  DC to 1.25 kHz
                 float n = FFT256.read(i);
                  printNumber(n);
                }
                Serial.println();
                */
        }
        if(adjust_LO)
        {
            //check_LO();
            Serial.println("Checking LO");
        }
    }

    
    if((memorytimer > 500) & (shift == 5))
    {
        //print_audio_lib_usage();
        //display_found_bats();
        memorytimer = 0;
    }


       /**********************************************************************************
          PRINT ROUTINE FOR ELAPSED MICROSECONDS
       **********************************************************************************/
#ifdef DEBUG

      if (elapsed_micros_idx_t > 100) 
      {
          elapsed_micros_mean = elapsed_micros_sum / elapsed_micros_idx_t;
          // one audio block is 128 samples
          // the time for this in seconds is:
          double block_time = 128.0 / (double)SAMPLE_RATE; 
          // block_time is equivalent to 100% processor load for ONE block processing
          // if we have more blocks to process:
          block_time = block_time * N_BLOCKS;
          block_time *= 1000000.0; // now in µseconds
          // take audio processing time and divide by block_time, convert to %
          double processor_load = elapsed_micros_mean / block_time * 100;       

        if (processor_load < 100.0)
        {
          Serial.print("processor load:  ");
          Serial.print (processor_load);
          Serial.println("%");
        }
        else
        {
          Serial.println("100%");
        }
        Serial.print (elapsed_micros_mean);
        Serial.print (" microsec for ");
        Serial.print (N_BLOCKS);
        Serial.println ("  stereo blocks    ");
        //Serial.print("FFT-length = "); Serial.print(FFT_length);
        //Serial.print(";   FIR filter length = "); Serial.println(m_NumTaps);
        elapsed_micros_idx_t = 0;
        elapsed_micros_sum = 0;
        elapsed_micros_mean = 0;        
      }
#endif   

    
      /**********************************************************************************
          Add button check etc. here
       **********************************************************************************/

} // end loop

void calc_FIR_coeffs (float * coeffs_I, int numCoeffs, float32_t fc, float32_t Astop, int type, float dfc, float Fsamprate)
// pointer to coefficients variable, no. of coefficients to calculate, frequency where it happens, stopband attenuation in dB,
// filter type, half-filter bandwidth (only for bandpass and notch)
{ // modified by WMXZ and DD4WH after
  // Wheatley, M. (2011): CuteSDR Technical Manual. www.metronix.com, pages 118 - 120, FIR with Kaiser-Bessel Window
  // assess required number of coefficients by
  //     numCoeffs = (Astop - 8.0) / (2.285 * TPI * normFtrans);
  // selecting high-pass, numCoeffs is forced to an even number for better frequency response

  float32_t Beta;
  float32_t izb;
  float fcf = fc;
  int nc = numCoeffs;
  fc = fc / Fsamprate;
  dfc = dfc / Fsamprate;
  // calculate Kaiser-Bessel window shape factor beta from stop-band attenuation
  if (Astop < 20.96)
    Beta = 0.0;
  else if (Astop >= 50.0)
    Beta = 0.1102 * (Astop - 8.71);
  else
    Beta = 0.5842 * powf((Astop - 20.96), 0.4) + 0.07886 * (Astop - 20.96);

  for (int i = 0; i < numCoeffs; i++) //zero pad entire coefficient buffer, important for variables from DMAMEM
  {
    coeffs_I[i] = 0.0;
  }

  izb = Izero (Beta);
  if (type == 0) // low pass filter
    //     {  fcf = fc;
  { fcf = fc * 2.0;
    nc =  numCoeffs;
  }
  else if (type == 1) // high-pass filter
  { fcf = -fc;
    nc =  2 * (numCoeffs / 2);
  }
  else if ((type == 2) || (type == 3)) // band-pass filter
  {
    fcf = dfc;
    nc =  2 * (numCoeffs / 2); // maybe not needed
  }
  else if (type == 4) // Hilbert transform
  {
    nc =  2 * (numCoeffs / 2);
    // clear coefficients
    for (int ii = 0; ii < 2 * (nc - 1); ii++) coeffs_I[ii] = 0;
    // set real delay
    coeffs_I[nc] = 1;

    // set imaginary Hilbert coefficients
    for (int ii = 1; ii < (nc + 1); ii += 2)
    {
      if (2 * ii == nc) continue;
      float x = (float)(2 * ii - nc) / (float)nc;
      float w = Izero(Beta * sqrtf(1.0f - x * x)) / izb; // Kaiser window
      coeffs_I[2 * ii + 1] = 1.0f / (PIH * (float)(ii - nc / 2)) * w ;
    }
    return;
  }

  for (int ii = - nc, jj = 0; ii < nc; ii += 2, jj++)
  {
    float x = (float)ii / (float)nc;
    float w = Izero(Beta * sqrtf(1.0f - x * x)) / izb; // Kaiser window
    coeffs_I[jj] = fcf * m_sinc(ii, fcf) * w;

  }

  if (type == 1)
  {
    coeffs_I[nc / 2] += 1;
  }
  else if (type == 2)
  {
    for (int jj = 0; jj < nc + 1; jj++) coeffs_I[jj] *= 2.0f * cosf(PIH * (2 * jj - nc) * fc);
  }
  else if (type == 3)
  {
    for (int jj = 0; jj < nc + 1; jj++) coeffs_I[jj] *= -2.0f * cosf(PIH * (2 * jj - nc) * fc);
    coeffs_I[nc / 2] += 1;
  }

} // END calc_FIR_coeffs

float32_t Izero (float32_t x)
{
  float32_t x2 = x / 2.0;
  float32_t summe = 1.0;
  float32_t ds = 1.0;
  float32_t di = 1.0;
  float32_t errorlimit = 1e-9;
  float32_t tmp;
  do
  {
    tmp = x2 / di;
    tmp *= tmp;
    ds *= tmp;
    summe += ds;
    di += 1.0;
  }   while (ds >= errorlimit * summe);
  return (summe);
}  // END Izero

float m_sinc(int m, float fc)
{ // fc is f_cut/(Fsamp/2)
  // m is between -M and M step 2
  //
  float x = m * PIH;
  if (m == 0)
    return 1.0f;
  else
    return sinf(x * fc) / (fc * x);
}


      //servicePotentiometer: listens to the potentiometer and sends the new pot value
      //  to the audio processing algorithm as a control parameter
      void servicePotentiometer(unsigned long curTime_millis, unsigned long updatePeriod_millis) {
          static unsigned long lastUpdate_millis = 0;
          static float prev_val = -1.0;
      
        //has enough time passed to update everything?
          if (curTime_millis < lastUpdate_millis) lastUpdate_millis = 0; //handle wrap-around of the clock
          if ((curTime_millis - lastUpdate_millis) > updatePeriod_millis) { //is it time to update the user interface?
      
          // read buttons
          push_button.update();
          up_button.update();
          down_button.update();
          if (push_button.fallingEdge()) 
          {
              Serial.println("Push-Button Press");
              shift_new = shift_new + 1;
              if(shift_new > 5)
              {
                  audio_gain *= 0.5f;
                  shift_new = lowest_shift;
              }
              Serial.print("New Shift is: "); Serial.println(shift_new);
              display_mode();
              
          } 
          // add other buttons here
      
          
          //read potentiometer
          static float old_val = 0; 
          float val = (1023.0f - (float32_t)analogRead(POTI)) / 1023.0; //0.0 to 1.0
          val = 0.05f * val + 0.95f * old_val; // exponential averager DD4WH
          old_val = val;
          val = (0.025f) * (float)((int)(40.0 * val + 0.125)); //quantize so that it doesn't chatter...0 to 1.0
          //Serial.print("Poti-Wert (0 bis 1): "); Serial.println(val);
      
          //send the potentiometer value to your algorithm as a control parameter
          //    if (abs(val - prev_val) > 0.05) { //is it different than before?
          if (val != prev_val) { //is it different than before?
            prev_val = val;  //save the value for comparison for the next time around
      
            //choose the desired gain value based on the knob setting
            const float min_gain_dB = -10.0, max_gain_dB = 25.0; //set desired gain range
            vol_knob_gain_dB = min_gain_dB + (max_gain_dB - min_gain_dB)*val; //computed desired gain value in dB
            //command the new gain setting
            if(vol_knob_gain_dB == 0.0) vol_knob_gain_dB = 0.01;
            audio_gain = powf(10.0, vol_knob_gain_dB / 20.0);
            if(shift == 5) audio_gain *=2.0f;
            gainR.gain(audio_gain);
            gainL.gain(audio_gain);
            //Serial.print("Poti-Wert (0 bis 1): "); Serial.println(val);
            //Serial.print("servicePotentiometer: Digital Gain dB = "); Serial.println(vol_knob_gain_dB); //print text to Serial port for debugging
          }
          lastUpdate_millis = curTime_millis;
        } // end if
      } //end servicePotentiometer();
      
      void init_interpolation_filter(void) {
        AudioNoInterrupts(); 
        if (arm_fir_interpolate_init_f32(&interpolation_R, (uint8_t)shift, num_taps, interpolation_coeffs, interpolation_R_state, (uint32_t) (BLOCK_SIZE * N_BLOCKS) / (uint32_t)shift)) 
        {
          Serial.println("Init of interpolation failed");
          while(1);
        }
        if (arm_fir_interpolate_init_f32(&interpolation_L, (uint8_t)shift, num_taps , interpolation_coeffs, interpolation_L_state, (uint32_t) (BLOCK_SIZE * N_BLOCKS) / (uint32_t)shift)) 
        {
          Serial.println("Init of interpolation failed");
          while(1);
        }
        buffer_idx=0; // this is important
        delay(10);
        AudioInterrupts();
        }

        void setI2SFreq(int freq) {
        // PLL between 27*24 = 648MHz und 54*24=1296MHz
        int n1 = 4; //SAI prescaler 4 => (n1*n2) = multiple of 4
        int n2 = 1 + (24000000 * 27) / (freq * 256 * n1);
        double C = ((double)freq * 256 * n1 * n2) / 24000000;
        int c0 = C;
        int c2 = 10000;
        int c1 = C * c2 - (c0 * c2);
        set_audioClock(c0, c1, c2, true);
        CCM_CS1CDR = (CCM_CS1CDR & ~(CCM_CS1CDR_SAI1_CLK_PRED_MASK | CCM_CS1CDR_SAI1_CLK_PODF_MASK))
             | CCM_CS1CDR_SAI1_CLK_PRED(n1-1) // &0x07
             | CCM_CS1CDR_SAI1_CLK_PODF(n2-1); // &0x3f 
      //Serial.printf("SetI2SFreq(%d)\n",freq);
      }
/*
void Print_processor_load(long msec) {
      static int32_t sum = 0;
      static int idx_t = 0;
      float32_t mean = 0.0f;
      sum = sum + msec;
      idx_t++;
      if (idx_t > 150) {
        mean = sum / idx_t;
        if (mean / 29.00 / N_BLOCKS * SAMPLE_RATE / AUDIO_SAMPLE_RATE_EXACT < 100.0)
        {
          Serial.print("processor load:  ");
          Serial.print (mean / 29.00 / N_BLOCKS * SAMPLE_RATE / AUDIO_SAMPLE_RATE_EXACT);
          Serial.println("%");
        }
        else
        {
          Serial.println("100%");
        }
        Serial.print (mean);
        Serial.print (" microsec for ");
        Serial.print (N_BLOCKS);
        Serial.print ("  stereo blocks    ");
        //Serial.print("FFT-length = "); Serial.print(FFT_length);
        //Serial.print(";   FIR filter length = "); Serial.println(m_NumTaps);

        idx_t = 0;
        sum = 0;
      }
}
*/

      void print_audio_lib_usage()
      {
            // print a summary of the current & maximum usage
          Serial.print("CPU: ");
          Serial.print("fft=");
          Serial.print(FFT256.processorUsage());
          Serial.print("%, ");
          Serial.print(FFT256.processorUsageMax());
          Serial.print("%  ");
          Serial.print("all=");
          Serial.print(AudioProcessorUsage());
          Serial.print("%, ");
          Serial.print(AudioProcessorUsageMax());
          Serial.print("%    ");
          Serial.print("Memory: ");
          Serial.print(AudioMemoryUsage());
          Serial.print(", ");
          Serial.print(AudioMemoryUsageMax());
          Serial.print("    ");
          Serial.println();
      }

       void spectrum() { // spectrum analyser code by rheslip - modified
           if (FFT256.available()) {
           int scale = 5;
        for (int16_t x = 2; x < 128; x++) {
             FFT_bin[x] = abs(FFT256.output[x]); 
             int bar = (FFT_bin[x] * scale);
             //if (bar >175) bar=175;
             // this is a very simple first order IIR filter to smooth the reaction of the bars
             bar = 0.05 * bar + 0.95 * barm[x]; 
      
             // tft.drawPixel(x*2+10, 210-barm[x], ILI9341_BLACK);
             // tft.drawPixel(x*2+10, 210-bar, ILI9341_WHITE);
      
             barm[x] = bar;
        }
        search_bats();     
        } //end if
      }

      void  search_bats() {
          // the array FFT_bin contains the results of the 256 point FFT --> 127 magnitude values
          // we look for bins that have a high amplitude compared to the mean noise, indicating the presence of ultrasound
          // 1. only search in those parts of the array > 14kHz and not around +-10kHz of the LO freq -->
          //    thus it is best, if I search in two parts --> 14kHz to freq_real-10k AND freq_real+10k to sample_rate/2
          // 2. determine mean and max in both parts of the array
          // 3. if we find a bin that is much larger than the mean (take care of situations where mean is zero!) --> identify the no. of the bin
          // 4. determine frequency of that bin (depends on sample_rate_real)
          //    a.) by simply multiplying bin# with bin width
          //    b.) by using an interpolator (not (yet) implemented)
          // 5. display frequency in bold and RED for 1-2 sec. (TODO: also display possible bat species ;-)) and then delete
          // goto 1.    
          float32_t lower_ultrasound_limit = 14000.0f;
          int lower_ultrasound_limit_idx = (int)(lower_ultrasound_limit / 375.0f);
          for (unsigned x=0; x < lower_ultrasound_limit_idx; x++)
          {
              FFT_bin[x] = 0; // zero out DC part and low freqs
          }
          for (unsigned x=lower_ultrasound_limit_idx; x < 128; x++)
          {
              FFT_bin[x] = FFT256.read(x); // read in float 0 to 1.0
          }
          
          arm_max_f32(FFT_bin, 128, &FFT_max, &FFT_max_idx);
          arm_mean_f32(FFT_bin, 128, &FFT_mean);
          Serial.print("Mean: "); Serial.print(FFT_mean, 6); Serial.print("              MAX:  "); Serial.print(FFT_max,4); Serial.print("               MAX/MEAN = "); Serial.println(FFT_max/FFT_mean, 2); 
          // only adjust frequency 
          if(FFT_max > FFT_mean * 4.0f && FFT_max > 0.002f)
          {
              FFT_max_freq = FFT_max_freq * 0.5 + (FFT_max_idx * SAMPLE_RATE / FFT_points) * 0.5f;
              
              int rounded_freq = int((FFT_max_freq / 100)) * 100;
              //if (fabsf(last_LO_frequency - FFT_max_freq) > 50)                          //only adjust when necessary
              {
                  SineL.frequency(rounded_freq/(SAMPLE_RATE/AUDIO_SAMPLE_RATE_EXACT));  
                  SineR.frequency(rounded_freq/(SAMPLE_RATE/AUDIO_SAMPLE_RATE_EXACT));
                  SineL.amplitude(0.9f);
                  SineR.amplitude(0.9f);
                  last_LO_frequency = rounded_freq;
                  het_freq = (int)rounded_freq;
                  display_het_freq();
              }
          }     
 /*         
          for (unsigned x=0; x < 4; x++)
          {
              FFT_bin[x] = 0; // zero out DC part and low freqs
          }
          for (uint16_t x=4; x < 128; x++)
          {
              FFT_bin[x] = FFT256.output[x];
          }
          arm_max_q15(FFT_bin, 128, &FFT_max1, &FFT_max_bin1);
          arm_mean_q15(FFT_bin, 128, &FFT_mean1);
          
          //if((FFT_max1 > 1.1 * FFT_mean1) && (FFT_mean1 > 0))
          if(1)
          {
              //FFT_max_freq = (int)((float)FFT_max_bin1 * (float)SAMPLE_RATE / (float)FFT_points);

              FFT_max_freq = (int)(((float)FFT_max_freq * 0.95f) + ((float)FFT_max_bin1 * (float)SAMPLE_RATE / (float)FFT_points * 0.05f));    
              check_LO();
              Serial.print("FFT max freq = "); Serial.println(FFT_max_freq);
          }
          else
          {
               
          }
                
    */      

    /*  
          // search array in two parts: 
          //  1.)  14k to (freq_real - 10k)
          // upper and lower limits for maximum search
           l_limit = 14000;
           u_limit = het_freq - 10000;
           index_l_limit =  (l_limit * FFT_points / SAMPLE_RATE);  // 1024 !
           index_u_limit =  (u_limit * FFT_points / SAMPLE_RATE);  // 1024 !
      //     Serial.print(index_l_limit); Serial.print ("  "); Serial.println(index_u_limit);
      
           if (index_u_limit > index_l_limit) { 
              arm_max_q15(&FFT_bin[index_l_limit], index_u_limit - index_l_limit, &FFT_max1, &FFT_max_bin1);
                  // this is the efficient CMSIS function to calculate the mean
              arm_mean_q15(&FFT_bin[index_l_limit], index_u_limit - index_l_limit, &FFT_mean1);
                  // shift bin_max because we have not searched through ALL the 256 FFT bins
      //     Serial.print(index_l_limit); Serial.print ("  "); Serial.println(index_u_limit);
              FFT_max_bin1 = FFT_max_bin1 + index_l_limit;
           }
      //     Serial.print(FFT_max1); Serial.print ("  "); Serial.println(FFT_mean1);
      
          //  2.)  (freq_real + 10k) to 256 
          // upper and lower limits for maximum search
           l_limit = het_freq + 10000;
           if (l_limit < 14000) {
            l_limit = 14000;
           }
           index_l_limit = (l_limit * FFT_points / SAMPLE_RATE); 
           index_u_limit = (FFT_points / 2) - 1; 
      //     Serial.print(index_l_limit); Serial.print ("  "); Serial.println(index_u_limit);
           if (index_u_limit > index_l_limit) { 
              arm_max_q15(&FFT_bin[index_l_limit], index_u_limit - index_l_limit, &FFT_max2, &FFT_max_bin2);
                  // this is the efficient CMSIS function to calculate the mean
              arm_mean_q15(&FFT_bin[index_l_limit], index_u_limit - index_l_limit, &FFT_mean2);
                  // shift bin_max because we have not searched through ALL the 128 FFT bins
              FFT_max_bin2 = FFT_max_bin2 + index_l_limit;
           }
      //         Serial.print(FFT_max2); Serial.print ("  "); Serial.println(FFT_mean2);
      */
          //display_found_bats();

      }  // END function search_bats()

      void check_LO() 
      {
          //int frequency = int((FFT_max_bin1 * (SAMPLE_RATE / FFT_points) / 500)) * 500 + 1000; //round to nearest 500hz and shift 1000hz up to make the signal audible
          int frequency = FFT_max_freq;
          if (fabsf(last_LO_frequency - frequency) > 50)                          //only adjust when necessary
          {
          het_freq = frequency;
          SineL.frequency(het_freq/(SAMPLE_RATE/AUDIO_SAMPLE_RATE_EXACT));  
          SineR.frequency(het_freq/(SAMPLE_RATE/AUDIO_SAMPLE_RATE_EXACT));
          SineL.amplitude(0.9f);
          SineR.amplitude(0.9f);
          last_LO_frequency = het_freq;
          display_het_freq();
          }               
      }

      void display_mode(void)
      {
              Display->clearBuffer();
              Display->setFont(u8g2_font_6x10_mf);  // Hauteur 7
              Display->setCursor(3,25);
              if (shift_new != 5)
              {
                  Display->println("Pitch Shift: ");
              }
              else
              {
                  Display->println("Heterodyne Mode");
              }
              Display->setCursor(3,37);
              switch(shift_new)
              {
                  case 1: Display->printf("zero! "); break;
                  case 2: Display->printf("one "); break;
                  case 3: Display->printf("one & a half "); break;
                  case 4: Display->printf("two "); break;
                  default: break;
              }
              if (shift_new != 5) Display->println("oct.");      
              Display->sendBuffer();
      }

      void display_het_freq(void)
      {
              Display->clearBuffer();
              Display->setFont(u8g2_font_6x10_mf);  // Hauteur 7
              Display->setCursor(3,25);
              Display->println("Heterodyne Mode");
              Display->setCursor(3,37);
              Display->println(het_freq);
              Display->sendBuffer();

      }

      
void printNumber(float n) {
  
  if (n >= 0.004) {
    Serial.print(n, 3);
    Serial.print(" ");
  } else {
    Serial.print("   -  "); // don't print "0.00"
  }
  
  /*
  if (n > 0.25) {
    Serial.print("***** ");
  } else if (n > 0.18) {
    Serial.print(" ***  ");
  } else if (n > 0.06) {
    Serial.print("  *   ");
  } else if (n > 0.005) {
    Serial.print("  .   ");
  }
  */
}
