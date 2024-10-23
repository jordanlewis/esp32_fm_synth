/*
   Copyright (c) 2022 Marcel Licence

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.

   Dieses Programm ist Freie Software: Sie können es unter den Bedingungen
   der GNU General Public License, wie von der Free Software Foundation,
   Version 3 der Lizenz oder (nach Ihrer Wahl) jeder neueren
   veröffentlichten Version, weiter verteilen und/oder modifizieren.

   Dieses Programm wird in der Hoffnung bereitgestellt, dass es nützlich sein wird, jedoch
   OHNE JEDE GEWÄHR,; sogar ohne die implizite
   Gewähr der MARKTFÄHIGKEIT oder EIGNUNG FÜR EINEN BESTIMMTEN ZWECK.
   Siehe die GNU General Public License für weitere Einzelheiten.

   Sie sollten eine Kopie der GNU General Public License zusammen mit diesem
   Programm erhalten haben. Wenn nicht, siehe <https://www.gnu.org/licenses/>.
*/

/**
   @file ml_synth_basic_example.ino
   @author Marcel Licence

   this file should be opened with arduino, this is the main project file

   shown in: https://youtu.be/rGTw05GKwvU
*/


#ifdef __CDT_PARSER__
#include <cdt.h>
#endif

/*
   global project configuration
   stays on top of multi-file-compilation
*/
#include "config.h"


#include <Arduino.h>
#include <arduino-timer.h>
#include <FS.h>
#include <SD_MMC.h>
#include <WiFi.h>

#include <FastLED.h>

extern void Status_ValueChangedFloat(const char *descr, float value);

/* requires the ML_SynthTools library: https://github.com/marcel-licence/ML_SynthTools */
#include <ml_arp.h>
#include <ml_delay.h>
#include <ml_midi_ctrl.h>
#include <ml_reverb.h>
#ifdef OLED_OSC_DISP_ENABLED
#include <ml_scope.h>
#endif

#include <ml_types.h>

#define ML_SYNTH_INLINE_DECLARATION
#include <ml_inline.h>
#undef ML_SYNTH_INLINE_DECLARATION

/* to avoid the high click when turning on the microphone */
static float click_supp_gain = 0.0f;

#define N_CHORDS 105
static int chords[N_CHORDS][4] = {
  {45, 48, 52, 0},
  {57, 48, 52, 0},
  {57, 60, 52, 0},
  {45, 48, 52, 55},
  {57, 48, 52, 55},
  {57, 60, 52, 55},
  {57, 60, 64, 55},
  {48, 52, 55, 0},
  {60, 52, 55, 0},
  {60, 64, 55, 0},
  {48, 52, 55, 59},
  {60, 52, 55, 59},
  {60, 64, 55, 59},
  {60, 64, 67, 59},
  {50, 53, 57, 0},
  {62, 53, 57, 0},
  {62, 65, 57, 0},
  {50, 53, 57, 60},
  {62, 53, 57, 60},
  {62, 65, 57, 60},
  {62, 65, 69, 60},
  {52, 55, 59, 0},
  {64, 55, 59, 0},
  {64, 67, 59, 0},
  {52, 55, 59, 62},
  {64, 55, 59, 62},
  {64, 67, 59, 62},
  {64, 67, 71, 62},
  {53, 57, 60, 0},
  {65, 57, 60, 0},
  {65, 69, 60, 0},
  {53, 57, 60, 64},
  {65, 57, 60, 64},
  {65, 69, 60, 64},
  {65, 69, 72, 64},
  {55, 59, 62, 0},
  {67, 59, 62, 0},
  {67, 71, 62, 0},
  {55, 59, 62, 65},
  {67, 59, 62, 65},
  {67, 71, 62, 65},
  {67, 71, 74, 65},
  {57, 60, 64, 0},
  {69, 60, 64, 0},
  {69, 72, 64, 0},
  {57, 60, 64, 67},
  {69, 60, 64, 67},
  {69, 72, 64, 67},
  {69, 72, 76, 67},
  {60, 64, 67, 0},
  {72, 64, 67, 0},
  {72, 76, 67, 0},
  {60, 64, 67, 71},
  {72, 64, 67, 71},
  {72, 76, 67, 71},
  {72, 76, 79, 71},
  {62, 65, 69, 0},
  {74, 65, 69, 0},
  {74, 77, 69, 0},
  {62, 65, 69, 72},
  {74, 65, 69, 72},
  {74, 77, 69, 72},
  {74, 77, 81, 72},
  {64, 67, 71, 0},
  {76, 67, 71, 0},
  {76, 79, 71, 0},
  {64, 67, 71, 74},
  {76, 67, 71, 74},
  {76, 79, 71, 74},
  {76, 79, 83, 74},
  {65, 69, 72, 0},
  {77, 69, 72, 0},
  {77, 81, 72, 0},
  {65, 69, 72, 76},
  {77, 69, 72, 76},
  {77, 81, 72, 76},
  {77, 81, 84, 76},
  {67, 71, 74, 0},
  {79, 71, 74, 0},
  {79, 83, 74, 0},
  {67, 71, 74, 77},
  {79, 71, 74, 77},
  {79, 83, 74, 77},
  {79, 83, 86, 77},
  {69, 72, 76, 0},
  {81, 72, 76, 0},
  {81, 84, 76, 0},
  {69, 72, 76, 79},
  {81, 72, 76, 79},
  {81, 84, 76, 79},
  {81, 84, 88, 79},
  {72, 76, 79, 0},
  {84, 76, 79, 0},
  {84, 88, 79, 0},
  {72, 76, 79, 83},
  {84, 76, 79, 83},
  {84, 88, 79, 83},
  {84, 88, 91, 83},
  {74, 77, 81, 0},
  {86, 77, 81, 0},
  {86, 89, 81, 0},
  {74, 77, 81, 84},
  {86, 77, 81, 84},
  {86, 89, 81, 84},
  {86, 89, 93, 84}
};

// LED gamma correction: https://learn.adafruit.com/led-tricks-gamma-correction/the-quick-fix
// We don't use PROGMEM because it's slower and we are not RAM limited.
const uint8_t gamma8[] = {
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,
  1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,
  2,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,  4,  4,  5,  5,  5,
  5,  6,  6,  6,  6,  7,  7,  7,  7,  8,  8,  8,  9,  9,  9, 10,
  10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16,
  17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25,
  25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36,
  37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50,
  51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66, 67, 68,
  69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89,
  90, 92, 93, 95, 96, 98, 99, 101, 102, 104, 105, 107, 109, 110, 112, 114,
  115, 117, 119, 120, 122, 124, 126, 127, 129, 131, 133, 135, 137, 138, 140, 142,
  144, 146, 148, 150, 152, 154, 156, 158, 160, 162, 164, 167, 169, 171, 173, 175,
  177, 180, 182, 184, 186, 189, 191, 193, 196, 198, 200, 203, 205, 208, 210, 213,
  215, 218, 220, 223, 225, 228, 231, 233, 236, 239, 241, 244, 247, 249, 252, 255
};

// The LED strips have 160 LEDs per meter.
#define NUM_L0 242
#define NUM_L1 234
#define NUM_L2 46
// NUM_L3 is the total length of the fourth strip, which is split into 2 segments.
#define NUM_L3 146
// NUM_L3 is the length of the first segment of the fourth strip.
#define NUM_L3_P1 80
// The longest strip happens to be the second segment of the 4th strips, since it's connected
// at a high offset to one of the branches.
#define L3_P2_OFFSET 197
#define NUM_L5 110
#define MAX_NUM_LEDS (L3_P2_OFFSET + NUM_L3-NUM_L3_P1)
CRGB leds0[NUM_L0];
CRGB leds1[NUM_L1];
CRGB leds2[NUM_L2];
CRGB leds3[NUM_L3];

// The 5th LED strip is simply the second half of the 4th LED strip,
// so we use a single long array for it, and divide that long array
// in 2.
CRGB *leds4 = leds3 + 80;
CRGB leds5[NUM_L5];

static CRGB *ledss[] = {leds0, leds1, leds2, leds3, leds4, leds5};
int ledSizes[] = {NUM_L0, NUM_L1, NUM_L2, NUM_L3_P1, NUM_L3-NUM_L3_P1};
int ledOffsets[] = {0, 0, 0, 168, L3_P2_OFFSET, 0};

unsigned int chord_counter = 0;
#define LED_COLOR_ORDER GRB

// A value of N indicates the LED train will be 1/N of the strip length
#define LED_TRAIN_LENGTH_FRACTION 5

int led_train_length;
int led_train_head = 0;

void button1() {
  Serial.println("one");
}
void button2() {
  Serial.println("two");
}
void button3() {
  Serial.println("three");
}
void button4() {
  Serial.println("four");
}

auto timerLeds = timer_create_default();
auto timer1sec = timer_create_default();
auto timer8sec = timer_create_default();
auto timerRestartChord = timer_create_default();
auto bleepSequenceTimer = timer_create_default();
auto bleepTimer = timer_create_default();

#define KEY1 36
#define KEY2 13
#define KEY3 19
#define KEY4 23
#define KEY5 18
#define KEY6 5

static int curChannel = 0;
static int lastChannel = 0;

static int maxBleepWaitMs = 7000;

// Mutex for reading/writing the lastChord variable
SemaphoreHandle_t lastChordMutex = NULL;
static int *lastChord = NULL;

/* this application starts here */
void setup()
{
  // put your setup code here, to run once:
  delay(250);
  Serial.begin(SERIAL_BAUDRATE);
  Serial.println();

  pinMode(KEY1, INPUT_PULLUP);
  pinMode(KEY2, INPUT_PULLUP);
  pinMode(KEY3, INPUT_PULLUP);
  pinMode(KEY4, INPUT_PULLUP);
  pinMode(KEY5, INPUT_PULLUP);
  pinMode(KEY6, INPUT_PULLUP);

  heap_caps_print_heap_info(MALLOC_CAP_8BIT);

#ifdef ESP32
  Serial.printf("ESP.getFreeHeap() %d\n", ESP.getFreeHeap());
  Serial.printf("ESP.getMinFreeHeap() %d\n", ESP.getMinFreeHeap());
  Serial.printf("ESP.getHeapSize() %d\n", ESP.getHeapSize());
  Serial.printf("ESP.getMaxAllocHeap() %d\n", ESP.getMaxAllocHeap());

  Serial.printf("Total heap: %d\n", ESP.getHeapSize());
  Serial.printf("Free heap: %d\n", ESP.getFreeHeap());

  /* PSRAM will be fully used by the looper */
  Serial.printf("Total PSRAM: %d\n", ESP.getPsramSize());
  Serial.printf("Free PSRAM: %d\n", ESP.getFreePsram());
#endif

  int chordIdx = random(N_CHORDS);
  lastChord = chords[chordIdx];
  setupLeds();
  lastChordMutex = xSemaphoreCreateMutex();
  Core0TaskInit(); // audio
  Core1TaskInit(); // LEDs
}

void setupLeds()
{
  FastLED.addLeds<WS2812B, 5, LED_COLOR_ORDER>(leds0, NUM_L0);
  FastLED.addLeds<WS2812B, 18, LED_COLOR_ORDER>(leds1, NUM_L1);
  FastLED.addLeds<WS2812B, 23, LED_COLOR_ORDER>(leds2, NUM_L2);

  // Adding this strip caused the CRGB::Purple background (faded) color to be a no-op (black / blank ) 
  // at 100 brightness, but only on the 0th LED strip. The same color works on all of the other strips. 
  // Setting the brightness to 127 (and possible 126 -untested) makes the purple show up again. But 
  // the same purple looks slightly different on the first strip... 
  FastLED.addLeds<WS2812B, 22, LED_COLOR_ORDER>(leds3, NUM_L3);
  FastLED.addLeds<WS2812B, 21, LED_COLOR_ORDER>(leds5, NUM_L5);

  // FastLED.setCorrection(TypicalLEDStrip);
  // FastLED.setTemperature(Tungsten100W);
  FastLED.setBrightness(255); // 0-255
  FastLED.setDither(1);
  // FastLED.setMaxRefreshRate(0);

  FastLED.clear();
  FastLED.show();
  lightTest();

  led_train_length = 50; //divRoundClosest(MAX_NUM_LEDS, LED_TRAIN_LENGTH_FRACTION);

  // TODO: this timer interval isn't 100% accurate. I'm currently seeing log lines like this:
  // https://gist.github.com/dasl-/c8967188b8ee15a4b60d195d8c776e98
  //
  // It means the timer is executing every 44-45 ms, whereas I want it to execute every 41 ms
  // Maybe this is because `FastLED.show()` takes ~4 ms, so it gets that much delayed? Then we keep
  // getting more and more off track. It means the LED train doesn't make it to the end of the strip
  // by the time the chord changes.
  //
  // Maybe we need to implement a custom timer using the millis() function that will correct itself if
  // things get off??
  int led_timer_ms = 7550 / (MAX_NUM_LEDS + led_train_length);
  Serial.printf("Led Timer MS target: %d\n", led_timer_ms);
  timerLeds.every(10, doLedTimer);
}

void loopLeds()
{
  int loop_s = millis();
  timerLeds.tick();

  // https://github.com/FastLED/FastLED/wiki/FastLED-Temporal-Dithering
  // > The more often your code calls FastLED.show(), or FastLED.delay(), the higher-quality the dithering will be
  int show_s = millis();
  FastLED.show();
  int end = millis();
  int show_elapsed = end - show_s;
  int loop_elapsed = end - loop_s; 
  // Serial.printf("Show took: %d ms\n", show_elapsed);
  // Serial.printf("loop took: %d ms\n", loop_elapsed);
}

void setupAudio()
{
  Serial.printf("Loading data\n");
  click_supp_gain = 0.0f;

#ifdef BLINK_LED_PIN
  Blink_Setup();
#endif

#if 0
  Status_Setup();
#endif

  Audio_Setup();

#ifdef ESP32_AUDIO_KIT
  button_setup();
#endif
  Sine_Init();

  /*
     Initialize reverb
     The buffer shall be static to ensure that
     the memory will be exclusive available for the reverb module
  */
  //static float revBuffer[REV_BUFF_SIZE];
  static float *revBuffer = (float *)malloc(sizeof(float) * REV_BUFF_SIZE);
  Reverb_Setup(revBuffer);

  /*
     Prepare a buffer which can be used for the delay
  */
  static int16_t *delBuffer = (int16_t *)malloc(sizeof(int16_t) * MAX_DELAY);
  Delay_Init(delBuffer, MAX_DELAY);

  /*
     setup midi module / rx port
  */
  Midi_Setup();

#ifdef ARP_MODULE_ENABLED
  Arp_Init(24 * 4); /* slowest tempo one step per bar */
#endif

  FmSynth_Init();

  Serial.printf("Firmware started successfully\n");

#ifdef NOTE_ON_STARTUP /* activate this line to get a tone on startup to test the DAC */
  FmSynth_NoteOn(0, 64, 1.0f);
#endif

#ifdef MIDI_STREAM_PLAYER_ENABLED
  MidiStreamPlayer_Init();

  char midiFile[] = "/fm_demo2_fm0.mid";
  MidiStreamPlayer_PlayMidiFile_fromLittleFS(midiFile, 3);
#endif

  FmSynth_NoteOn(curChannel, 1, 0.0f);
  FmSynth_NoteOff(curChannel, 1);
  delay(50);

  FmSynth_ModulationWheel(1, .039);
  FmSynth_ModulationSpeed(1, .196);
  Reverb_SetLevel(1, .5);
  Delay_SetInputLevel(1, 0.74f);
  Delay_SetLength(1, 0.717f);
  Delay_SetOutputLevel(1, 0.5f);
  Delay_SetFeedback(1, 0.5f);
  FmSynth_TremoloWheel(1, .094);
  FmSynth_TremoloSpeed(1, .157);

  delay(125);

  timer1sec.every(1000, loop_1Hz);
  timer8sec.every(8000, loop_8sec);

  loop_8sec(NULL);
  bleepSequenceTimer.in(random(maxBleepWaitMs), doBleepSequence);
}

#ifdef ESP32
/*
   Core 0
*/
/* this is used to add a task to core 0 */
TaskHandle_t Core0TaskHnd;
TaskHandle_t Core1TaskHnd;
inline
void Core0TaskInit()
{
  /* we need a second task for the terminal output */
  xTaskCreatePinnedToCore(Core0Task, "Core0Task", 8192, NULL, 999, &Core0TaskHnd, 0);
}

void Core0Task(void *parameter)
{
  setupAudio();
  while (true)
  {
    loopAudio();

    /* this seems necessary to trigger the watchdog */
    delay(1);
    yield();
  }
}
inline
void Core1TaskInit()
{
  /* we need a second task for the terminal output */
  xTaskCreatePinnedToCore(Core1Task, "Core1Task", 8192, NULL, 999, &Core1TaskHnd, 1);
}

void Core1Task(void *parameter)
{
  while (true) {
    loopLeds();
  }
}
#endif /* ESP32 */


float master_output_gain = 0.15f;

#ifdef ESP32_AUDIO_KIT
/* little enum to make switching more clear */
enum acSource
{
  acSrcLine,
  acSrcMic
};

/* line in is used by default, so it should not be changed here */
enum acSource selSource = acSrcLine;

/* be carefull when calling this function, microphones can cause very bad feedback!!! */
void App_ToggleSource(uint8_t channel, float value)
{
  if (value > 0)
  {
    switch (selSource)
    {
      case acSrcLine:
        click_supp_gain = 0.0f;
#ifdef AC101_ENABLED
        ac101_setSourceMic();
#endif
        selSource = acSrcMic;
        // Status_TestMsg("Input: Microphone");
        break;
      case acSrcMic:
        click_supp_gain = 0.0f;
#ifdef AC101_ENABLED
        ac101_setSourceLine();
#endif
        selSource = acSrcLine;
        // Status_TestMsg("Input: LineIn");
        break;
    }
  }
}
#endif

void App_SetOutputLevel(uint8_t not_used, float value)
{
  master_output_gain = value;
}

static uint32_t midiSyncCount = 0;

void Midi_SyncRecvd()
{
  midiSyncCount += 1;
}

void Synth_RealTimeMsg(uint8_t msg)
{
#ifndef MIDI_SYNC_MASTER
  switch (msg)
  {
    case 0xfa: /* start */
      Arp_Reset();
      break;
    case 0xf8: /* Timing Clock */
      Midi_SyncRecvd();
      break;
  }
#endif
}

#ifdef MIDI_SYNC_MASTER

#define MIDI_PPQ    24
#define SAMPLES_PER_MIN  (SAMPLE_RATE*60)

static float midi_tempo = 120.0f;

void MidiSyncMasterLoop(void)
{
  static float midiDiv = 0;
  midiDiv += SAMPLE_BUFFER_SIZE;
  if (midiDiv >= (SAMPLES_PER_MIN) / (MIDI_PPQ * midi_tempo))
  {
    midiDiv -= (SAMPLES_PER_MIN) / (MIDI_PPQ * midi_tempo);
    Midi_SyncRecvd();
  }
}

void Synth_SetMidiMasterTempo(uint8_t unused, float val)
{
  midi_tempo = 60.0f + val * (240.0f - 60.0f);
}

#endif

void Synth_SongPosition(uint16_t pos)
{
  Serial.printf("Songpos: %d\n", pos);
  if (pos == 0)
  {
    Arp_Reset();
  }
}

void Synth_SongPosReset(uint8_t unused, float var)
{
  if (var > 0)
  {
    Synth_SongPosition(0);
  }
}

/*
   this should avoid having a constant offset on our signal
   I am not sure if that is required, but in case it can avoid early clipping
*/
static float fl_offset = 0.0f;
static float fr_offset = 0.0f;


static float fl_sample[SAMPLE_BUFFER_SIZE];
static float fr_sample[SAMPLE_BUFFER_SIZE];
static float m1_sample[SAMPLE_BUFFER_SIZE];

#ifndef absf
#define absf(a) ((a>=0.0f)?(a):(-a))
#endif

/*
   the main audio task
*/
inline void audio_task()
{
#ifdef AUDIO_PASS_THROUGH
  memset(fl_sample, 0, sizeof(fl_sample));
  memset(fr_sample, 0, sizeof(fr_sample));
  memset(m1_sample, 0, sizeof(m1_sample));
#ifdef ESP32_AUDIO_KIT
  //Audio_Input(fl_sample, fr_sample);
#endif
#else
  memset(fl_sample, 0, sizeof(fl_sample));
  memset(fr_sample, 0, sizeof(fr_sample));
  memset(m1_sample, 0, sizeof(m1_sample));
#endif

  //    for (int n = 0; n < SAMPLE_BUFFER_SIZE; n++)
  //    {
  //        /*
  //         * this avoids the high peak coming over the mic input when switching to it
  //         */
  //        fl_sample[n] *= click_supp_gain;
  //        fr_sample[n] *= click_supp_gain;
  //
  //        if (click_supp_gain < 1.0f)
  //        {
  //            click_supp_gain += 0.00001f;
  //        }
  //        else
  //        {
  //            click_supp_gain = 1.0f;
  //        }
  //
  //        /* make it a bit quieter */
  //        fl_sample[n] *= 0.5f;
  //        fr_sample[n] *= 0.5f;
  //
  //        /*
  //         * this removes dc from signal
  //         */
  //        fl_offset = fl_offset * 0.99 + fl_sample[n] * 0.01;
  //        fr_offset = fr_offset * 0.99 + fr_sample[n] * 0.01;
  //
  //        fl_sample[n] -= fl_offset;
  //        fr_sample[n] -= fr_offset;
  //    }

  /*
     main loop core
  */
  FmSynth_Process(m1_sample, SAMPLE_BUFFER_SIZE);

  /*
     apply master output gain
  */
  for (int n = 0; n < SAMPLE_BUFFER_SIZE; n++)
  {
    /* apply master_output_gain */
    m1_sample[n] *= master_output_gain;
  }

  /*
     little simple delay effect
  */
  Delay_Process_Buff(m1_sample, SAMPLE_BUFFER_SIZE);

  /*
     add some mono reverb
  */
  Reverb_Process(m1_sample, SAMPLE_BUFFER_SIZE);

  /*
     mix mono signal to stereo out
  */
  for (int n = 0; n < SAMPLE_BUFFER_SIZE; n++)
  {
    m1_sample[n] *= 2.125f;
    //fl_sample[n] += m1_sample[n];
    //fr_sample[n] += m1_sample[n];
  }

  Audio_Output(m1_sample, m1_sample);

#ifdef OLED_OSC_DISP_ENABLED
  ScopeOled_AddSamples(fl_sample, fr_sample, SAMPLE_BUFFER_SIZE);
#endif

  Status_Process_Sample(SAMPLE_BUFFER_SIZE);
}

/*
   this function will be called once a second
   call can be delayed when one operation needs more time (> 1/44100s)
*/
bool loop_1Hz(void *)
{
#ifdef ESP32_AUDIO_KIT
  button_loop();
#endif
#ifdef BLINK_LED_PIN
  Blink_Process();
#endif

  return true;
}

static bool enableChords = true;


int lastStartChordTime = 0;
bool startChord(void *)
{
  if (lastStartChordTime != 0) {
    int now = millis();
    // Serial.printf("Time since last startChord: %d \n", now - lastStartChordTime);
    lastStartChordTime = now;
  } else {
    lastStartChordTime = millis();
  }

  Serial.printf("Playing chord on channel %d\n", curChannel);
  if (xSemaphoreTake(lastChordMutex, 0xFFFFFFFF) != pdTRUE) {
    return false;
  }

  // We acquired the last chord mutex
  int *chord = lastChord;
  if (lastChord == NULL) {
    int chordIdx = random(N_CHORDS);
    chord = chords[chordIdx];
    lastChord = chord;
  } else {
    // Find all chords that have all but one note in common with the previous chord.
    int chordArray[N_CHORDS];
    int nChords = 0;
    for (int i = 0; i < N_CHORDS; i++) {   // For each of the top level chords
      int nMatches = 0;
      for (int oldIdx = 0; oldIdx < 4; oldIdx++) { // For each of the notes in the last chord
        for (int newIdx = 0; newIdx < 4; newIdx++) {  // For each of the notes in the new chord
          if (lastChord[oldIdx] == chords[i][newIdx]) {
            // Found a match!
            //Serial.printf("Match: %d %d\n", lastChord[oldIdx], chords[i][newIdx]);
            nMatches += 1;
            break;
          }
        }
      }
      if (nMatches == 3) {
        //Serial.printf("Found match:");
        //for (int j = 0; j < 4; j++) {
        //  Serial.printf(" %d", chords[i][j]);
        //}
        //Serial.println("");
        chordArray[nChords] = i;
        nChords += 1;
      }
    }

    int chordIdx = random(nChords);
    chord = chords[chordArray[chordIdx]];
    lastChord = chord;
    //Serial.printf("Decided on:");
    //for (int i = 0; i < 4; i++) {
    //  Serial.printf(" %d", lastChord[i]);
    //}
    //Serial.println("");
  }


  Serial.printf("Playing chord");
  for (int i = 0; i < 4; i++) {
    Serial.printf(" %d", lastChord[i]);
    if (chord[i] != 0) {
      FmSynth_NoteOn(curChannel, lastChord[i], 1.0f);
    }
  }
  FmSynth_NoteOn(lastChannel, lastChord[0] % 12 + 48, 1.0f);

  Serial.println();
  xSemaphoreGive(lastChordMutex);

  return true;
}

bool loop_8sec(void *)
{
  chord_counter++;
  int *chord = lastChord;
  if (chord != NULL) {
    Serial.printf("Have last chord");
    for (int i = 0; i < 4; i++) {
      Serial.printf(" %d", chord[i]);
      if (chord[i] != 0) {
        FmSynth_NoteOff(lastChannel, chord[i]);
      }
    }
    FmSynth_NoteOff(lastChannel, chord[0] % 12 + 36);
    Serial.println("");
  }
  lastChannel = curChannel;
  if (!enableChords) {
    return true;
  }
  timerRestartChord.in(20, startChord);
  return true;
}

static int bleepCount;
static int bleepChan;
static int bleepNote;
static bool bleepPlaying = false;
static int bleepStrip = 0;
static bool enableBleeps = true;
static int bleepDur = 0;

bool doBleepSequence(void *) {
  if (lastChord == NULL || !enableBleeps) {
    bleepSequenceTimer.in(random(maxBleepWaitMs), doBleepSequence);
    return true;
  }
  bleepDur = random(400) + 20;
  bleepChan = 13;

  // Pick random note, if it's a triad ignore the 4th note (0).
  if (lastChord[3] == 0) {
    bleepNote = lastChord[random(3)];
  } else {
    bleepNote = lastChord[random(4)];
  }
  bleepNote += 12;

  if (bleepDur < 150) {
    bleepCount = random(10) + 2;
  } else {
    bleepCount = 1;
  }
  Serial.printf("Beginning a bleep sequence. count=%d, dur=%d, note=%d, chan=%d\n", bleepCount, bleepDur, bleepNote, bleepChan);
  bleepTimer.every(bleepDur, doBleep);
  return true;
}

bool doBleep(void *) {
  if (bleepPlaying) {
    FmSynth_NoteOff(bleepChan, bleepNote);
  }
  bleepPlaying = (bleepCount > 0);
  if (!bleepPlaying) {
    // Schedule a new bleep sequence.
    bleepNote = 0;
    bleepStrip = random(sizeof(ledSizes) / sizeof(ledSizes[0]));
    bleepSequenceTimer.in(random(maxBleepWaitMs), doBleepSequence);
    return false;
  }

  Serial.printf("bleeping. count=%d, note=%d, chan=%d, strip=%d\n", bleepCount, bleepNote, bleepChan, bleepStrip);

  FmSynth_NoteOn(bleepChan, bleepNote, .5);
  bleepCount--;
  return true;
}

// 0 - 255, higher values are fainter.
// Ex: 192 means 75% faded
#define BACKGROUND_FADE_AMOUNT 200
static int *lastLastChord = NULL;
int bleepBeginTime = 0;
int bleepLit = 0;
int chordBeginTime = 0;
bool doLedTimer(void *)
{
  int now = millis();

  if (xSemaphoreTake(lastChordMutex, 0xFFFFFF) != pdTRUE) {
    Serial.printf("failed to acquire lastChordMutex in doLedTimer\n");
    return false;
  }
  // We acquired the last chord mutex

  if (lastLastChord != lastChord) {
    // The chord just changed, i.e. it's been 8 seconds
    lastLastChord = lastChord;
    led_train_head = 0;
    chordBeginTime = now;
    // Serial.printf("chord changed\n");
  }

  CRGB root_color = getColorForNote(lastChord[0]);
  CRGB third_color = getColorForNote(lastChord[1]);
  CRGB fifth_color = getColorForNote(lastChord[2]);
  CRGB seventh_color = NULL;
  if (lastChord[3] != 0) {
    CRGB seventh_color = getColorForNote(lastChord[3]);
  }
  
  xSemaphoreGive(lastChordMutex);
  double fractionThroughCycle = 1-((double)((double)8000 - (now-chordBeginTime))/(double)8000);
  led_train_head = fractionThroughCycle * (MAX_NUM_LEDS + led_train_length);
  //Serial.printf("Fraction through cycle: %f: %d\n", fractionThroughCycle, led_train_head);

  int led_train_tail = led_train_head - led_train_length + 1;
  int train_mid_pt;
  int distance_to_mid_pt;

  // This formula works regardless of whether the train length is even or odd. In the odd case,
  // we take advantage of integer truncation.
  int steps_to_fade_train = (led_train_length + 1) / 2; 
  int fade_amount_per_step = divRoundClosest(BACKGROUND_FADE_AMOUNT, steps_to_fade_train);
  bool is_odd = led_train_length % 2 == 1;

  CRGB root_background_color = getBackgroundColor(root_color);
  CRGB third_background_color = getBackgroundColor(third_color);
  CRGB fifth_background_color = getBackgroundColor(fifth_color);
  CRGB seventh_background_color = NULL;
  if (seventh_color != NULL) {
    seventh_background_color = getBackgroundColor(seventh_color);
  }
  
  // Why do we iterate backwards? When iterating forwards, we had a weird "color bleed" problem. The 1st LED of
  // the 1st strip would be the color of the last LED in the 2nd strip. Likewise, the 1st LED of the 2nd strip 
  // would be the color of the last LED in the 3rd strip. I assume this is due to some data line  interference.
  for (int i = MAX_NUM_LEDS - 1; i >= 0; i--)
  {
    if (led_train_tail <= i && i <= led_train_head) {
      // Hop aboard the train
      train_mid_pt = is_odd ? led_train_tail + steps_to_fade_train - 1 : led_train_tail + steps_to_fade_train;
      if (is_odd) {
        // train has single mid-point
        distance_to_mid_pt = abs(train_mid_pt - i);
      } else {
        // train has two mid-points
        if (i == train_mid_pt || i == (train_mid_pt - 1)) {
          distance_to_mid_pt = 0;
        } else if (i < train_mid_pt) {
          distance_to_mid_pt = (train_mid_pt - 1) - i;
        } else {
          distance_to_mid_pt = i - train_mid_pt;
        }
      }

      CRGB root_train_color = getColorForTrainPosition(root_color, fade_amount_per_step, distance_to_mid_pt);
      CRGB third_train_color = getColorForTrainPosition(third_color, fade_amount_per_step, distance_to_mid_pt);
      CRGB fifth_train_color = getColorForTrainPosition(fifth_color, fade_amount_per_step, distance_to_mid_pt);
      CRGB seventh_train_color = seventh_color == NULL ? NULL : getColorForTrainPosition(seventh_color, fade_amount_per_step, distance_to_mid_pt);
      setLeds(i, 0, root_train_color);
      setLeds(i, 1, root_train_color);
      setLeds(i, 2, third_train_color);
      setLeds(i, 3, seventh_train_color == NULL ? fifth_train_color : seventh_train_color);
      setLeds(i, 4, seventh_train_color == NULL ? fifth_train_color : seventh_train_color);
      setLeds(i, 5, fifth_train_color);
    } else {
      setLeds(i, 0, root_background_color);
      setLeds(i, 1, root_background_color);
      setLeds(i, 2, third_background_color);
      setLeds(i, 3, seventh_background_color == NULL ? fifth_background_color : seventh_background_color);
      setLeds(i, 4, seventh_background_color == NULL ? fifth_background_color : seventh_background_color);
      setLeds(i, 5, fifth_background_color);
    }
  }

  //led_train_head += 1;

  // Now compute bleeps - we're going to light up the ends of the strips when a bleep is active.
  if (bleepLit == 0 && bleepNote != 0 && bleepPlaying) {
    // Begin bleep.
    bleepBeginTime = now;
    bleepLit = 1;
  };
  if (bleepNote == 0) {
    bleepLit = 0;
  }

  int durSinceBleepBegin = now-bleepBeginTime;
  if (bleepLit && durSinceBleepBegin < bleepDur) {
    // We are in the middle of a bleep.
    // Serial.printf("activating bleep for note %d on strip %d\n", bleepNote, bleepStrip);
    int stripEnd = ledSizes[bleepStrip] + ledOffsets[bleepStrip] - 1;
    CRGB bleepColor = getColorForNote(bleepNote);
    // This commented out code attempted temporal fading which is kind of pointless because the bleeps are short.
    //CRGB bleepColor = getColorForTrainPosition(getColorForNote(bleepNote), fade_amount_per_step, (1-((bleepDur - durSinceBleepBegin) / bleepDur)) * steps_to_fade_train);
    for (int i = stripEnd; i >= stripEnd - 25; i--) {
      setLeds(i, bleepStrip, getColorForTrainPosition(bleepColor, fade_amount_per_step, stripEnd-i));
    }
  } else {
    // Disable the bleep light.
    bleepLit = 0;
  }
  
  
  return true;
}

void setLeds(int ledIndex, int stripIndex, CRGB color) {
  ledIndex -= ledOffsets[stripIndex];
  if (0 <= ledIndex && ledIndex < ledSizes[stripIndex] + ledOffsets[stripIndex]) {
    ledss[stripIndex][ledIndex] = color;
  }
}

const CRGB colors[] = {
  CRGB::LightCoral,
  CRGB::FireBrick,
  CRGB::Gold,
  CRGB::HotPink,
  CRGB::Tomato,
  CRGB::SaddleBrown,
  CRGB::Orange

// CRGB(255, 255, 0), // yellow
// CRGB(255, 176, 0), // orange
// CRGB(255, 0, 0), // red
// CRGB(255, 52, 222), // pink
// CRGB(177, 0, 255), // purple
// CRGB(72, 219, 203), // teal
// CRGB(0, 255, 0) // green
};
CRGB getColorForNote(int midi_note)
{
  if (midi_note % 12 == 0) return colors[0]; // C
  if (midi_note % 12 == 2) return colors[1]; // D
  if (midi_note % 12 == 4) return colors[2]; // E
  if (midi_note % 12 == 5) return colors[3]; // F
  if (midi_note % 12 == 7) return colors[4]; // G
  if (midi_note % 12 == 9) return colors[5]; // A
  if (midi_note % 12 == 11) return colors[6]; // B
  else return CRGB::White; // this should never happen
}

CRGB getBackgroundColor(CRGB color) {
  // https://github.com/FastLED/FastLED/wiki/Pixel-reference#dimming-and-brightening-colors
  color.fadeLightBy(BACKGROUND_FADE_AMOUNT);
  return scaleGamma(color);
}

CRGB getColorForTrainPosition(CRGB color, int fade_amount_per_step, int distance_to_mid_pt) {
  CRGB ret_color = color;
  ret_color.fadeLightBy(fade_amount_per_step * distance_to_mid_pt);
  return scaleGamma(ret_color);
}

CRGB scaleGamma(CRGB color)
{
  color.r = gamma8[color.r];
  color.g = gamma8[color.g];
  color.b = gamma8[color.b];
  return color;
}

void lightTest()
{
  Serial.printf("Starting light test\n");
  FastLED.delay(500);
  for (int i = 0; i < 7; i++)
  {
    CRGB led_background_color = colors[i];
    led_background_color.fadeLightBy(BACKGROUND_FADE_AMOUNT);
    led_background_color = scaleGamma(led_background_color);
    for (int j = 0; j < NUM_L1; j++)
    {
      leds0[j] = led_background_color;     
    }
    FastLED.delay(500);
  }
  Serial.printf("Finished light test\n");
}

// Returns a % b.
// C modulo operator (%) doesn't work as expected with negative numbers, so implement our own:
// https://stackoverflow.com/questions/11720656/modulo-operation-with-negative-numbers
int mod(int a, int b)
{
  int r = a % b;
  return r < 0 ? r + b : r;
}

// Returns `n / d` rounded to the closest integer
// https://stackoverflow.com/a/18067292
int divRoundClosest(const int n, const int d)
{
  return ((n < 0) == (d < 0)) ? ((n + d / 2) / d) : ((n - d / 2) / d);
}

void loopAudio()
{
  timer1sec.tick();
  timer8sec.tick();
  timerRestartChord.tick();
  bleepSequenceTimer.tick();
  bleepTimer.tick();

  Midi_Process();

#ifdef MIDI_STREAM_PLAYER_ENABLED
  MidiStreamPlayer_Tick(SAMPLE_BUFFER_SIZE);
#endif

#ifdef MIDI_VIA_USB_ENABLED
  UsbMidi_ProcessSync();
#endif

#ifdef MIDI_SYNC_MASTER
  MidiSyncMasterLoop();
#endif

#ifdef ARP_MODULE_ENABLED
  Arp_Process(midiSyncCount);
  midiSyncCount = 0;
#endif

  audio_task();
}


/*
   this is the main loop
*/
void loop()
{
  static int key1val = 1;
  static int key2val = 1;
  static int key3val = 1;

  int newVal = digitalRead(KEY1);
  if (key1val == 1 && newVal == 0) {
    enableBleeps = !enableBleeps;
    Serial.printf("enableBleeps = %d\n", enableBleeps);
    key1val = newVal;
  } else if (newVal == 1) {
    key1val = newVal;
  }

  newVal = digitalRead(KEY2);
  if (key2val == 1 && newVal == 0) {
    enableChords = !enableChords;
    Serial.printf("enableChords = %d\n", enableChords);
    key2val = newVal;
  } else if (newVal == 1) {
    key2val = newVal;
  }

  delay(1);
  yield();
}

/*
   Callbacks
*/
void MidiCtrl_Cb_NoteOn(uint8_t ch, uint8_t note, float vel)
{
  Arp_NoteOn(ch, note, vel);
}

void MidiCtrl_Cb_NoteOff(uint8_t ch, uint8_t note)
{
  Arp_NoteOff(ch, note);
}

void MidiCtrl_Status_ValueChangedIntArr(const char *descr, int value, int index)
{
  Status_ValueChangedIntArr(descr, value, index);
}

void Arp_Cb_NoteOn(uint8_t ch, uint8_t note, float vel)
{
  FmSynth_NoteOn(ch, note, vel);
}

void Arp_Cb_NoteOff(uint8_t ch, uint8_t note)
{
  FmSynth_NoteOff(ch, note);
}

void Arp_Status_ValueChangedInt(const char *msg, int value)
{
  Status_ValueChangedInt(msg, value);
}

void Arp_Status_LogMessage(const char *msg)
{
  Status_LogMessage(msg);
}

void Arp_Status_ValueChangedFloat(const char *msg, float value)
{
  Status_ValueChangedFloat(msg, value);
}

void Arp_Cb_Step(uint8_t step)
{
  /* ignore */
}
