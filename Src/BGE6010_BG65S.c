/*
 * BGE6010_BG65S.c
 *
 *  Created on: 21.05.2019
 *      Author: CM
 */

#include "BGE6010_BG65S.h"

canOpen_typeDef_SDOprimitive BGE6010_BG65S_initSDOs[INIT_SDO_COUNT]=
{
         {0x3000, 0, 1, 1}             // Fehlerregister löschen                           1
        ,{0x3004, 0, 0, 1}             // Leistungsstufe deaktivieren                      2
        ,{0x3900, 0, 1, 1}             // Motor Typ BLDC eintragen                         3
        ,{0x3910, 0, 3*10, 1}          // Motor Pole eintragen                             4
        ,{0x3911, 0, 2, 1}             // Motor Polarität eintragen                        5
        ,{0x3962, 0, 2000, 2}          // Encoder Auflösung eintragen                      6
        ,{0x3350, 0, 0x096A, 2}        // Drehzahlrückführung eintragen (Encoder)          7
        ,{0x3901, 0, 3240, 2}          // Motor Nenndrehzahl                               8
        ,{0x3902, 0, 24000, 2}         // Motor Nennspannung                               9
        ,{0x3221, 0, 32000, 4}         // Strombegrenzung pos                              10
        ,{0x3223, 0, 32000, 4}         // Strombegrenzung neg                              11
        ,{0x3224, 0, 0, 1}             // Dynamische Strombegrenzung deaktiviert           12
        ,{0x39A0, 0x00, 1, 1}          // Bremsmanagement aktivieren                       13
        ,{0x39A0, 0x08, -0x161, 2}     // Bremsausgang Dout1 Low-Aktiv                     14
        ,{0x3154, 0x00, 0xFD, 1}       // Ausgang Dout1 für manuelles Setzen sperren       15
        ,{0x39A0, 0x10,  30, 2}        // t1 = Zeitverzögerung1 Aktivieren = 30 ms         16
        ,{0x39A0, 0x11,  50, 2}        // t2 = Zeitverzögerung2 Aktivieren = 50 ms         17
        ,{0x39A0, 0x12, 200, 2}        // t3 = Zeitverzögerung1 Deaktivieren = 200 ms      18
        ,{0x39A0, 0x13, 250, 2}        // t4 = Zeitverzögerung2 Deaktivieren = 250 ms      19
        ,{0x39A0, 0x18, 5, 2}          // Bit0, Bit2: Aktivierung der Bremse nach Betriebsfreigabe     20
                                    //             ("Enable") oder nach dem Starten einer Bewegung
        ,{0x39A0, 0x1A, 4, 2}          // Bit2: Deaktivierung der Bremse nach gesperrter Betriebs-     21
                                    //       freigabe ("Disable")
        ,{0x2002, 1, 0x6E657277, 4}    // Schreiben aktivieren                             22
        ,{0x2002, 2, 1, 4}             // DSP402 aktivieren                                23
        ,{0x1010, 5, 0x65766173, 4}    // Änderung (2000er) speichern                      24
        ,{0x608C, 0, 164, 1}           // Geschwindigkeitseinheit U/min                    25
        ,{0x6060, 0, 2, 1}             // Betriebsart Velocity Mode                        26
        ,{0x6007, 0, 3, 2}             // Aktion bei Kommunikationsabbruch: Quickstop      27
        ,{0x6040, 0, 0, 2}             // Kontrollwort disable                             28
        ,{0x6040, 0, 0x80, 2}          // Kontrollwort Fehler löschen                      29
        ,{0x6040, 0, 0, 2}             // Kontrollwort disable                             30
        ,{0x6042, 0, 0, 2}             // Sollgeschwindigkeit 0                            31
        ,{0x60FE, 2, 0x10000, 4}       // Dout0 freigeben                                  32
        ,{0x60FE, 1, 0, 4}             // Dout0 auf 0 setzen (sonst aus unerfindlichen Gründen gesetzt)    33
        ,{0x6040, 0, 6, 2}             // Kontrollwort enable voltage + quick stop off     34
        ,{0x6040, 0, 7, 2}             // Kontrollwort enable operation                    35
         /* ATTENTION!
          * always check number of SDOs INIT_SDO_COUNT!
          * must match entries here
          */
};
