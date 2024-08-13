/*
 * Copyright (c) 2022 Marcel Licence
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Dieses Programm ist Freie Software: Sie können es unter den Bedingungen
 * der GNU General Public License, wie von der Free Software Foundation,
 * Version 3 der Lizenz oder (nach Ihrer Wahl) jeder neueren
 * veröffentlichten Version, weiter verteilen und/oder modifizieren.
 *
 * Dieses Programm wird in der Hoffnung bereitgestellt, dass es nützlich sein wird, jedoch
 * OHNE JEDE GEWÄHR,; sogar ohne die implizite
 * Gewähr der MARKTFÄHIGKEIT oder EIGNUNG FÜR EINEN BESTIMMTEN ZWECK.
 * Siehe die GNU General Public License für weitere Einzelheiten.
 *
 * Sie sollten eine Kopie der GNU General Public License zusammen mit diesem
 * Programm erhalten haben. Wenn nicht, siehe <https://www.gnu.org/licenses/>.
 */

/**
 * @file ml_synth_basic_example.ino
 * @author Marcel Licence
 *
 * this file should be opened with arduino, this is the main project file
 *
 * shown in: https://youtu.be/rGTw05GKwvU
 */


#ifdef __CDT_PARSER__
#include <cdt.h>
#endif

/*
 * global project configuration
 * stays on top of multi-file-compilation
 */
#include "config.h"


#include <Arduino.h>
#include <arduino-timer.h>
#include <FS.h>
#include <SD_MMC.h>
#include <WiFi.h>


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
    {45,48,52,0},
    {57,48,52,0},
    {57,60,52,0},
    {45,48,52,55},
    {57,48,52,55},
    {57,60,52,55},
    {57,60,64,55},
    {48,52,55,0},
    {60,52,55,0},
    {60,64,55,0},
    {48,52,55,59},
    {60,52,55,59},
    {60,64,55,59},
    {60,64,67,59},
    {50,53,57,0},
    {62,53,57,0},
    {62,65,57,0},
    {50,53,57,60},
    {62,53,57,60},
    {62,65,57,60},
    {62,65,69,60},
    {52,55,59,0},
    {64,55,59,0},
    {64,67,59,0},
    {52,55,59,62},
    {64,55,59,62},
    {64,67,59,62},
    {64,67,71,62},
    {53,57,60,0},
    {65,57,60,0},
    {65,69,60,0},
    {53,57,60,64},
    {65,57,60,64},
    {65,69,60,64},
    {65,69,72,64},
    {55,59,62,0},
    {67,59,62,0},
    {67,71,62,0},
    {55,59,62,65},
    {67,59,62,65},
    {67,71,62,65},
    {67,71,74,65},
    {57,60,64,0},
    {69,60,64,0},
    {69,72,64,0},
    {57,60,64,67},
    {69,60,64,67},
    {69,72,64,67},
    {69,72,76,67},
    {60,64,67,0},
    {72,64,67,0},
    {72,76,67,0},
    {60,64,67,71},
    {72,64,67,71},
    {72,76,67,71},
    {72,76,79,71},
    {62,65,69,0},
    {74,65,69,0},
    {74,77,69,0},
    {62,65,69,72},
    {74,65,69,72},
    {74,77,69,72},
    {74,77,81,72},
    {64,67,71,0},
    {76,67,71,0},
    {76,79,71,0},
    {64,67,71,74},
    {76,67,71,74},
    {76,79,71,74},
    {76,79,83,74},
    {65,69,72,0},
    {77,69,72,0},
    {77,81,72,0},
    {65,69,72,76},
    {77,69,72,76},
    {77,81,72,76},
    {77,81,84,76},
    {67,71,74,0},
    {79,71,74,0},
    {79,83,74,0},
    {67,71,74,77},
    {79,71,74,77},
    {79,83,74,77},
    {79,83,86,77},
    {69,72,76,0},
    {81,72,76,0},
    {81,84,76,0},
    {69,72,76,79},
    {81,72,76,79},
    {81,84,76,79},
    {81,84,88,79},
    {72,76,79,0},
    {84,76,79,0},
    {84,88,79,0},
    {72,76,79,83},
    {84,76,79,83},
    {84,88,79,83},
    {84,88,91,83},
    {74,77,81,0},
    {86,77,81,0},
    {86,89,81,0},
    {74,77,81,84},
    {86,77,81,84},
    {86,89,81,84},
    {86,89,93,84}
};

void button1(){
  Serial.println("one"); 
}
void button2(){
  Serial.println("two"); 
}
void button3(){
  Serial.println("three"); 
}
void button4(){
  Serial.println("four"); 
}


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

static int maxBleepWaitMs = 12000;

/* this application starts here */
void setup()
{
    // put your setup code here, to run once:
    delay(250);

    timer1sec.every(1000, loop_1Hz);
    timer8sec.every(8000, loop_8sec);

    pinMode(KEY1,INPUT_PULLUP);
    pinMode(KEY2,INPUT_PULLUP);
    pinMode(KEY3,INPUT_PULLUP);
    pinMode(KEY4,INPUT_PULLUP);
    pinMode(KEY5,INPUT_PULLUP);
    pinMode(KEY6,INPUT_PULLUP);


    heap_caps_print_heap_info(MALLOC_CAP_8BIT);

    Serial.begin(SERIAL_BAUDRATE);

    Serial.println();

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
     * Initialize reverb
     * The buffer shall be static to ensure that
     * the memory will be exclusive available for the reverb module
     */
    //static float revBuffer[REV_BUFF_SIZE];
    static float *revBuffer = (float *)malloc(sizeof(float) * REV_BUFF_SIZE);
    Reverb_Setup(revBuffer);

    /*
     * Prepare a buffer which can be used for the delay
     */
    static int16_t *delBuffer = (int16_t *)malloc(sizeof(int16_t) * MAX_DELAY);
    Delay_Init(delBuffer, MAX_DELAY);

    /*
     * setup midi module / rx port
     */
    Midi_Setup();

#ifdef ARP_MODULE_ENABLED
    Arp_Init(24 * 4); /* slowest tempo one step per bar */
#endif

    FmSynth_Init();

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

    Serial.printf("Firmware started successfully\n");

#ifdef NOTE_ON_STARTUP /* activate this line to get a tone on startup to test the DAC */
    FmSynth_NoteOn(0, 64, 1.0f);
#endif

#ifdef MIDI_STREAM_PLAYER_ENABLED
    MidiStreamPlayer_Init();

    char midiFile[] = "/fm_demo2_fm0.mid";
    MidiStreamPlayer_PlayMidiFile_fromLittleFS(midiFile, 3);
#endif

    Core0TaskInit();




    FmSynth_NoteOn(curChannel, 1, 0.0f);
    FmSynth_NoteOff(curChannel, 1);
    delay(50);

    FmSynth_ModulationWheel(1, .039);
    FmSynth_ModulationSpeed(1, .196);
    Reverb_SetLevel(1, .8);
    Delay_SetInputLevel(1, 0.74f);
    Delay_SetLength(1, 0.717f);
    Delay_SetOutputLevel(1, 0.5f);
    Delay_SetFeedback(1, 0.5f);
    FmSynth_TremoloWheel(1, .094);
    FmSynth_TremoloSpeed(1, .157);

    delay(125);
    
    loop_8sec(NULL);
    bleepSequenceTimer.in(random(maxBleepWaitMs), doBleepSequence);
}

#ifdef ESP32
/*
 * Core 0
 */
/* this is used to add a task to core 0 */
TaskHandle_t Core0TaskHnd;

inline
void Core0TaskInit()
{
    /* we need a second task for the terminal output */
    xTaskCreatePinnedToCore(Core0Task, "CoreTask0", 8000, NULL, 999, &Core0TaskHnd, 0);
}

inline
void Core0TaskSetup()
{
    /*
     * init your stuff for core0 here
     */

#ifdef OLED_OSC_DISP_ENABLED
    ScopeOled_Setup();
#endif

    Status_Setup();

#ifdef MIDI_VIA_USB_ENABLED
    usb_setup();
    MIDI_setShortMsgHandler(HandleShortMsg);
#endif

#ifdef PRESSURE_SENSOR_ENABLED
    PressureSetup();
#endif
}

void Core0TaskLoop()
{
    /*
     * put your loop stuff for core0 here
     */
    Status_Process();
#ifdef MIDI_VIA_USB_ENABLED
    usb_loop();
#endif

#ifdef OLED_OSC_DISP_ENABLED
    ScopeOled_Process();
#endif

#ifdef PRESSURE_SENSOR_ENABLED
    PressureLoop();
#endif
}

void Core0Task(void *parameter)
{
    Core0TaskSetup();

    while (true)
    {
        Core0TaskLoop();

        /* this seems necessary to trigger the watchdog */
        delay(1);
        yield();
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
 * this should avoid having a constant offset on our signal
 * I am not sure if that is required, but in case it can avoid early clipping
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
 * the main audio task
 */
inline void audio_task()
{
#ifdef AUDIO_PASS_THROUGH
    memset(fl_sample, 0, sizeof(fl_sample));
    memset(fr_sample, 0, sizeof(fr_sample));
    memset(m1_sample, 0, sizeof(m1_sample));
#ifdef ESP32_AUDIO_KIT
    Audio_Input(fl_sample, fr_sample);
#endif
#else
    memset(fl_sample, 0, sizeof(fl_sample));
    memset(fr_sample, 0, sizeof(fr_sample));
    memset(m1_sample, 0, sizeof(m1_sample));
#endif

    for (int n = 0; n < SAMPLE_BUFFER_SIZE; n++)
    {
        /*
         * this avoids the high peak coming over the mic input when switching to it
         */
        fl_sample[n] *= click_supp_gain;
        fr_sample[n] *= click_supp_gain;

        if (click_supp_gain < 1.0f)
        {
            click_supp_gain += 0.00001f;
        }
        else
        {
            click_supp_gain = 1.0f;
        }

        /* make it a bit quieter */
        fl_sample[n] *= 0.5f;
        fr_sample[n] *= 0.5f;

        /*
         * this removes dc from signal
         */
        fl_offset = fl_offset * 0.99 + fl_sample[n] * 0.01;
        fr_offset = fr_offset * 0.99 + fr_sample[n] * 0.01;

        fl_sample[n] -= fl_offset;
        fr_sample[n] -= fr_offset;
    }

    /*
     * main loop core
     */
    FmSynth_Process(m1_sample, SAMPLE_BUFFER_SIZE);

    /*
     * apply master output gain
     */
    for (int n = 0; n < SAMPLE_BUFFER_SIZE; n++)
    {
        /* apply master_output_gain */
        m1_sample[n] *= master_output_gain;
    }

    /*
     * little simple delay effect
     */
    Delay_Process_Buff(m1_sample, SAMPLE_BUFFER_SIZE);

    /*
     * add some mono reverb
     */
    Reverb_Process(m1_sample, SAMPLE_BUFFER_SIZE);

    /*
     * mix mono signal to stereo out
     */
    for (int n = 0; n < SAMPLE_BUFFER_SIZE; n++)
    {
        m1_sample[n] *= 2.125f;
        fl_sample[n] += m1_sample[n];
        fr_sample[n] += m1_sample[n];
    }

    Audio_Output(fl_sample, fr_sample);

#ifdef OLED_OSC_DISP_ENABLED
    ScopeOled_AddSamples(fl_sample, fr_sample, SAMPLE_BUFFER_SIZE);
#endif

    Status_Process_Sample(SAMPLE_BUFFER_SIZE);
}

/*
 * this function will be called once a second
 * call can be delayed when one operation needs more time (> 1/44100s)
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

static int *lastChord = NULL;
static bool enableChords = true;


bool startChord(void *)
{
        Serial.printf("Playing chord on channel %d\n", curChannel);
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
                    Serial.printf("Found match:");
                    for (int j = 0; j < 4; j++) {
                        Serial.printf(" %d", chords[i][j]); 
                    }
                    Serial.println("");
                    chordArray[nChords] = i;
                    nChords += 1;
                }
            }
            
            int chordIdx = random(nChords);
            chord = chords[chordArray[chordIdx]];
            lastChord = chord;
        }

        Serial.printf("Playing chord");
        for (int i = 0; i < 4; i++) {
            Serial.printf(" %d", chord[i]);
            if (chord[i]!=0) {
                 FmSynth_NoteOn(curChannel, chord[i], 1.0f);
            }
        }
        FmSynth_NoteOn(lastChannel,chord[0]%12 + 48,1.0f);

        Serial.println();

    return true;
}

bool loop_8sec(void *)
{
        int *chord = lastChord;
        if (chord != NULL) {
            Serial.printf("Have last chord");
            for (int i = 0; i < 4; i++) {
                Serial.printf(" %d", chord[i]);
                if (chord[i] != 0) {
                     FmSynth_NoteOff(lastChannel, chord[i]);
                }
            }
            FmSynth_NoteOff(lastChannel,chord[0]%12 + 36);
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
static bool enableBleeps = true;

bool doBleepSequence(void *) {
    if (lastChord == NULL || !enableBleeps) {
      bleepSequenceTimer.in(random(maxBleepWaitMs), doBleepSequence);
      return true;
    }
    int bleepDur = random(400) + 20;
    bleepChan = 13;

    // Pick random note, if it's a triad ignore the 4th note (0).
    if (lastChord[3] == 0) {
        bleepNote = lastChord[random(3)];
    } else {
        bleepNote = lastChord[random(4)];
    }
    bleepNote += 12;
    
    if (bleepDur < 150) {
        bleepCount = random(10)+2;
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
        bleepSequenceTimer.in(random(maxBleepWaitMs), doBleepSequence);
        return false;
    }

    Serial.printf("bleeping. count=%d, note=%d, chan=%d\n", bleepCount, bleepNote, bleepChan);

    FmSynth_NoteOn(bleepChan, bleepNote, .5);
    bleepCount--;
    return true;
}

/*
 * this is the main loop
 */
void loop()
{
    timer1sec.tick();
    timer8sec.tick();
    timerRestartChord.tick();
    bleepSequenceTimer.tick();
    bleepTimer.tick();

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
 * Callbacks
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
