  Super Ears
  
  Compensating age-related high frequency loss for bird and Orthoptera field work
  and automatic heterodyne detector for bats
  
  Pitch shifting in the time domain for high pitched birds & Orthoptera
  & auto heterodyne mode for ultrasound, eg. bats & Tettigoniidae
  
  implements pitch shifting in the time domain and is based on an idea by Lang Elliott & Herb Susmann for the "Hear birds again"-project,
  specifically for the now deprecated SongFinder pitch shifter units. 
  The algorithm can shift the audio down by a factor of two (one octave), three (1.5 octaves) or four (two octaves). 
  Find a graph showing the implementation for the case of downshifting by 4 here: https://github.com/DD4WH/BirdSongPitchShifter#readme
   * Many thanks go to Harold Mills & Lang Elliott for explaining this algorithm to me and answering my questions ! :-) 
  https://hearbirdsagain.org/
  
  Many thanks to Jean-Do Vrignault for the Teensy Recorder code!

 Hardware needed:
  * uses Teensy 4.1 and external ADC / DAC connected on perf board with ground plane
  
   PCM5102A DAC module
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
   
   OLED display SSH1106
   
   Vcc = Vcc 3.3V
   GND = GND
   SCL 19
   SDA 18
   SCL & SDA each with a pull-up 4K7 resistor to 3V3 
   
   Potentiometer  pin 17
   Buttons
   UP      30 
   DOWN    32
   PUSH    31
