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
 * @file z_config.ino
 * @author Marcel Licence
 * @date 12.05.2021
 *
 * @brief This file contains the mapping configuration
 * Put all your project configuration here (no defines etc)
 * This file will be included at the and can access all
 * declarations and type definitions
 *
 * @see ESP32 Arduino DIY Synthesizer Projects - Little startup guide to get your MIDI synth working - https://youtu.be/ZNxGCB-d68g
 */


#ifdef __CDT_PARSER__
#include <cdt.h>
#endif


/*
 * this mapping is used for the edirol pcr-800
 * this should be changed when using another controller
 */
struct midiControllerMapping edirolMapping[] =
{
    /* transport buttons */
    { 0x8, 0x52, "back", NULL, NULL, 0},
    { 0xD, 0x52, "stop", NULL, NULL, 0},
    { 0xe, 0x52, "start", NULL, NULL, 0},
    { 0xe, 0x52, "start", NULL, NULL, 0},
    { 0xa, 0x52, "rec", NULL, NULL, 0},

    /* upper row of buttons */
    { 0x0, 0x50, "A1", NULL, FmSynth_ToggleMono, 0},
    { 0x1, 0x50, "A2", NULL, FmSynth_ToggleLegato, 1},
    { 0x2, 0x50, "A3", NULL, FmSynth_ChannelSettingDump, 2},
    { 0x3, 0x50, "A4", NULL, FmSynth_ChannelSettingInit, 3},

    { 0x4, 0x50, "A5", NULL, FmSynth_SelectOp, 0},
    { 0x5, 0x50, "A6", NULL, FmSynth_SelectOp, 1},
    { 0x6, 0x50, "A7", NULL, FmSynth_SelectOp, 2},
    { 0x7, 0x50, "A8", NULL, FmSynth_SelectOp, 3},

    { 0x0, 0x53, "A9", NULL, NULL, 0},

    /* lower row of buttons */
#ifdef ARP_MODULE_ENABLED
    { 0x0, 0x51, "B1", NULL, Arp_SelectSequence, 0},
    { 0x1, 0x51, "B2", NULL, Arp_SelectSequence, 1},
    { 0x2, 0x51, "B3", NULL, Arp_SelectSequence, 2},
    { 0x3, 0x51, "B4", NULL, Arp_SelectSequence, 3},

    { 0x4, 0x51, "B5", NULL, Arp_SelectSequence, 4},
    { 0x5, 0x51, "B6", NULL, Arp_SelectSequence, 5},
    { 0x6, 0x51, "B7", NULL, Arp_SelectSequence, 6},
    { 0x7, 0x51, "B8", NULL, Arp_SelectSequence, 7},
#else
    { 0x0, 0x51, "B1", NULL, FmSynth_SetAlgorithm, 0},
    { 0x1, 0x51, "B2", NULL, FmSynth_SetAlgorithm, 1},
    { 0x2, 0x51, "B3", NULL, FmSynth_SetAlgorithm, 2},
    { 0x3, 0x51, "B4", NULL, FmSynth_SetAlgorithm, 3},

    { 0x4, 0x51, "B5", NULL, FmSynth_SetAlgorithm, 4},
    { 0x5, 0x51, "B6", NULL, FmSynth_SetAlgorithm, 5},
    { 0x6, 0x51, "B7", NULL, FmSynth_SetAlgorithm, 6},
    { 0x7, 0x51, "B8", NULL, FmSynth_SetAlgorithm, 7},
#endif

    { 0x1, 0x53, "B9", NULL, FmSynth_SetAlgorithm, 8},

    /* pedal */
    { 0x0, 0x0b, "VolumePedal", NULL, NULL, 0},

    /* slider */
    { 0x0, 0x11, "S1", NULL, FmSynth_ChangeParam, 0},
    { 0x1, 0x11, "S2", NULL, FmSynth_ChangeParam, 1},
    { 0x2, 0x11, "S3", NULL, FmSynth_ChangeParam, 2},
    { 0x3, 0x11, "S4", NULL, FmSynth_ChangeParam, 3},

    { 0x4, 0x11, "S5", NULL, FmSynth_Attack, 4},
    { 0x5, 0x11, "S6", NULL, FmSynth_Decay1, 5},
    { 0x6, 0x11, "S7", NULL, FmSynth_DecayL, 6},
    { 0x7, 0x11, "S8", NULL, FmSynth_Decay2, 7},

    { 0x1, 0x12, "S9", NULL, FmSynth_Release, 8},

     /* CIRCUIT MAPPINGS */
    /* Synth 1 1-8 */
    { 0x1, 80, "R1", NULL, FmSynth_VelToLev, 0},
    { 0x1, 81, "R2", NULL, FmSynth_LfoAM, 1},
    { 0x1, 82, "R3", NULL, FmSynth_LfoFM, 2},
    { 0x1, 83, "R4", NULL, FmSynth_Feedback, 3},
    { 0x1, 84, "R5", NULL, Delay_SetInputLevel, 0},
    { 0x1, 85, "R6", NULL, Delay_SetFeedback, 0},
    { 0x1, 86, "R7", NULL, Delay_SetLength, 0},
    { 0x1, 87, "R8", NULL, Delay_SetOutputLevel, 0},

    /* Synth 2 1-8 */

    { 0x1, 80, "R1", NULL, FmSynth_ChangeParam, 0},
    { 0x1, 81, "R2", NULL, FmSynth_ChangeParam, 1},
    { 0x1, 82, "R3", NULL, FmSynth_ChangeParam, 2},
    { 0x1, 83, "R4", NULL, FmSynth_ChangeParam, 3},

    { 0x1, 84, "R5", NULL, FmSynth_Attack, 4},
    { 0x1, 85, "R6", NULL, FmSynth_Decay1, 5},
    { 0x1, 86, "R7", NULL, FmSynth_DecayL, 6},
    { 0x1, 87, "R8", NULL, FmSynth_Decay2, 7},

    { 0x0, 0x12, "R9", NULL, Reverb_SetLevel, 0},

    /* drum 1/2 */
    { 0x9, 14, "A1", NULL, FmSynth_ToggleMono, 0},
    { 0x9, 34, "A2", NULL, FmSynth_ToggleLegato, 1},
    { 0x9, 15, "A3", NULL, FmSynth_ChannelSettingDump, 2},
    { 0x9, 40, "A4", NULL, FmSynth_ChannelSettingInit, 3},
    { 0x9, 16, "A5", NULL, FmSynth_SelectOp, 0},
    { 0x9, 42, "A6", NULL, FmSynth_SelectOp, 1},
    { 0x9, 17, "A7", NULL, FmSynth_SelectOp, 2},
    { 0x9, 43, "A8", NULL, FmSynth_SelectOp, 3},



    /* drum 3/4 */

    { 0x9, 46, "A1", NULL, FmSynth_ToggleMono, 0},
    { 0x9, 55, "A2", NULL, FmSynth_ToggleLegato, 1},
    { 0x9, 47, "A3", NULL, FmSynth_ChannelSettingDump, 2},
    { 0x9, 57, "A4", NULL, FmSynth_ChannelSettingInit, 3},
    { 0x9, 48, "A5", NULL, FmSynth_SelectOp, 0},
    { 0x9, 61, "A6", NULL, FmSynth_SelectOp, 1},
    { 0x9, 49, "A7", NULL, FmSynth_SelectOp, 2},
    { 0x9, 76, "A8", NULL, FmSynth_SelectOp, 3},

    /* mixer */

    { 15, 12, "A1", NULL, FmSynth_ToggleMono, 0},
    { 15, 14, "A2", NULL, FmSynth_ToggleLegato, 1},
    { 0x9, 12, "A3", NULL, FmSynth_ChannelSettingDump, 2},
    { 0x9, 23, "A4", NULL, FmSynth_ChannelSettingInit, 3},
    { 0x9, 45, "A5", NULL, FmSynth_SelectOp, 0},
    { 0x9, 53, "A6", NULL, FmSynth_SelectOp, 1},

    /* fx 1 */

    { 15, 111, "A1", NULL, FmSynth_ToggleMono, 0},
    { 15, 112, "A2", NULL, FmSynth_ToggleLegato, 1},
    { 15, 113, "A3", NULL, FmSynth_ChannelSettingDump, 2},
    { 15, 114, "A4", NULL, FmSynth_ChannelSettingInit, 3},
    { 15, 115, "A5", NULL, FmSynth_SelectOp, 0},
    { 15, 116, "A6", NULL, FmSynth_SelectOp, 1},


    /* fx 2 */

    { 15, 88, "A1", NULL, FmSynth_ToggleMono, 0},
    { 15, 89, "A2", NULL, FmSynth_ToggleLegato, 1},
    { 15, 90, "A3", NULL, FmSynth_ChannelSettingDump, 2},
    { 15, 106, "A4", NULL, FmSynth_ChannelSettingInit, 3},
    { 15, 109, "A5", NULL, FmSynth_SelectOp, 0},
    { 15, 110, "A6", NULL, FmSynth_SelectOp, 1},

    /* Filter  */
    //{ 15, 74, "H1", NULL, Synth_SetMidiMasterTempo, 0},

};

struct midiControllerMapping circuitMapping[] =
{
    { 0x0, 80, "R1", NULL, FmSynth_VelToLev, 0},
    { 0x0, 81, "R2", NULL, FmSynth_LfoAM, 1},
    { 0x0, 82, "R3", NULL, FmSynth_LfoFM, 2},
    { 0x0, 83, "R4", NULL, FmSynth_Feedback, 3},
    { 0x0, 84, "R5", NULL, Delay_SetInputLevel, 0},
    { 0x0, 85, "R6", NULL, Delay_SetFeedback, 0},
    { 0x0, 86, "R7", NULL, Delay_SetLength, 0},
    { 0x0, 87, "R8", NULL, Delay_SetOutputLevel, 0},

    /* Synth 2 1-8 */

    { 0x1, 80, "R1", NULL, FmSynth_ChangeParam, 0},
    { 0x1, 81, "R2", NULL, FmSynth_ChangeParam, 1},
    { 0x1, 82, "R3", NULL, FmSynth_ChangeParam, 2},
    { 0x1, 83, "R4", NULL, FmSynth_ChangeParam, 3},
    { 0x1, 84, "R5", NULL, FmSynth_Attack, 4},
    { 0x1, 85, "R6", NULL, FmSynth_Decay1, 5},
    { 0x1, 86, "R7", NULL, FmSynth_DecayL, 6},
    { 0x1, 87, "R8", NULL, FmSynth_Decay2, 7},

    /* drum 1/2 */
    { 0x9, 14, "A1", NULL, FmSynth_ToggleMono, 0},
    { 0x9, 34, "A2", NULL, FmSynth_ToggleLegato, 1},
    { 0x9, 15, "A3", NULL, FmSynth_ChannelSettingDump, 2},
    { 0x9, 40, "A4", NULL, FmSynth_ChannelSettingInit, 3},
    { 0x9, 16, "A5", NULL, FmSynth_SelectOp, 0},
    { 0x9, 42, "A6", NULL, FmSynth_SelectOp, 1},
    { 0x9, 17, "A7", NULL, FmSynth_SelectOp, 2},
    { 0x9, 43, "A8", NULL, FmSynth_SelectOp, 3},

    /* drum 3/4 */

    { 0x9, 46, "A1", NULL, FmSynth_SetAlgorithm, 0},
    { 0x9, 55, "A2", NULL, FmSynth_SetAlgorithm, 1},
    { 0x9, 47, "A3", NULL, FmSynth_SetAlgorithm, 2},
    { 0x9, 57, "A4", NULL, FmSynth_SetAlgorithm, 3},
    { 0x9, 48, "A5", NULL, FmSynth_SetAlgorithm, 4},
    { 0x9, 61, "A6", NULL, FmSynth_SetAlgorithm, 5},
    { 0x9, 49, "A7", NULL, FmSynth_SetAlgorithm, 6},
    { 0x9, 76, "A8", NULL, FmSynth_SetAlgorithm, 7},

    /* mixer */

    { 15, 12, "A1", NULL, FmSynth_ModulationWheel, 0},
    { 15, 14, "A2", NULL, FmSynth_ModulationSpeed, 1},
    { 0x9, 12, "A3", NULL, FmSynth_TremoloWheel, 2},
    { 0x9, 23, "A4", NULL, FmSynth_TremoloSpeed, 3},
    { 0x9, 45, "A5", NULL, FmSynth_SelectOp, 0},
    { 0x9, 53, "A6", NULL, FmSynth_SelectOp, 1},

    /* fx 1 */

    { 15, 111, "A1", NULL, LEDChange, 0},
    { 15, 112, "A2", NULL, LEDChange, 1},
    { 15, 113, "A3", NULL, LEDChange, 2},
    { 15, 114, "A4", NULL, LEDStrip, 0},
    { 15, 115, "A5", NULL, LEDStrip, 1},
    { 15, 116, "A6", NULL, LEDStrip, 2},


    /* fx 2 */

    { 15, 88, "A1", NULL, LEDStrip, 3},
    { 15, 89, "A2", NULL, LEDStrip, 4},
    { 15, 90, "A3", NULL, LEDStrip, 5},
    { 15, 106, "A4", NULL, LEDStrip, 6},
    { 15, 109, "A5", NULL, LEDStrip, 7},
    { 15, 110, "A6", NULL, LEDStrip, 8},

    /* Filter  */
    { 15, 74, "H1", NULL, Reverb_SetLevel, 0},
};

struct midiMapping_s midiMapping =
{
    NULL,
#ifdef MIDI_CTRL_ENABLED
    MidiCtrl_NoteOn,
    MidiCtrl_NoteOff,
#else
#ifdef ARP_MODULE_ENABLED
    Arp_NoteOn,
    Arp_NoteOff,
#else
    FmSynth_NoteOn,
    FmSynth_NoteOff,
#endif
#endif
    FmSynth_PitchBend,
    FmSynth_ModulationWheel,
    NULL, /* assign program change callback here! */
    Synth_RealTimeMsg,
    Synth_SongPosition,
    circuitMapping,
    sizeof(circuitMapping) / sizeof(circuitMapping[0]),
};
