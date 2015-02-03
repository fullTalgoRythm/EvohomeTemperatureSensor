// EvohomeWirelessFW - RFBee firmware for evohome wireless communications
// Copyright (c) 2015 Hydrogenetic
//
// based on HoneyComm - Alternative RFBee firmware to communicate with
//             HR80 radiator controllers.
//
// Copyright (C) 2011 Wladimir Komarow
//
// and work from CrazyDiamond and others at http://www.domoticaforum.eu/viewtopic.php?f=7&t=5806
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//

#ifndef CCXCFG_H
#define CCXCFG_H

#include "CCx.h"
#include <avr/pgmspace.h>

#define CCX_NR_OF_REGISTERS 30

// list the registers in the same order as CCxRegisterSettings
// stored in progmem to save on RAM
static const byte CCx_registers[CCX_NR_OF_REGISTERS] PROGMEM = {
CCx_IOCFG2,
CCx_IOCFG1,
CCx_IOCFG0,
CCx_PKTCTRL1,
CCx_PKTCTRL0,
CCx_FSCTRL1,
CCx_FREQ2,
CCx_FREQ1,
CCx_FREQ0,
CCx_MDMCFG4,
CCx_MDMCFG3,
CCx_MDMCFG2,
CCx_MDMCFG1,
CCx_DEVIATN,
CCx_MCSM2,
CCx_MCSM1,
CCx_MCSM0,
CCx_FOCCFG,
CCx_AGCCTRL2,
CCx_AGCCTRL1,
CCx_AGCCTRL0,
CCx_FSCAL3,
CCx_FSCAL2,
CCx_FSCAL1,
CCx_FSCAL0,
CCx_FSTEST,
CCx_TEST2,
CCx_TEST1,
CCx_TEST0,
CCx_PATABLE,
};

#define CCX_NR_OF_CONFIGS 1

// configuration for CC1101
// stored in progmem to save on RAM
static const byte CCx_registerSettings[CCX_NR_OF_CONFIGS][CCX_NR_OF_REGISTERS] PROGMEM = {

      {
        0x0b,//CCx_IOCFG2 (Serial Clock. Synchronous to the data in synchronous serial mode.)
        0x2e,//CCx_IOCFG1
        0x0c,//CCx_IOCFG0 (Serial Synchronous Data Output. Used for synchronous serial mode.)
        0x00,//CCx_PKTCTRL1
        0x12,//CCx_PKTCTRL0 (Synchronous serial mode, Data in on GDO0 and data out on either of the GDOx pins)
        0x06,//CCx_FSCTRL1
        0x21,//CCx_FREQ2
        0x65,//CCx_FREQ1
        0x6c,//CCx_FREQ0
        0x6a,//CCx_MDMCFG4
        0x83,//CCx_MDMCFG3 (DRATE_M=131 data rate=38,383.4838867Hz)
        0x10,//CCx_MDMCFG2 (GFSK No Sync Word / Preamable)
        0x22,//CCx_MDMCFG1 (CHANSPC_E=2 NUM_PREAMBLE=4 FEC_EN=0)
        0x50,//CCx_DEVIATN
        0x07,//CCx_MCSM2
        0x30,//CCx_MCSM1 (0x30=110000 defaults to idle for RX,TX CCA_MODE=11  If RSSI below threshold unless currently receiving a packet)
        0x18,//CCx_MCSM0 (0x18=11000 FS_AUTOCAL=1 When going from IDLE to RX or TX)
        0x16,//CCx_FOCCFG,
        0x43,//CCx_AGCCTRL2,
        0x40,//CCx_AGCCTRL1,
        0x91,//CCx_AGCCTRL0,
        0xe9,//CCx_FSCAL3,
        0x2a,//CCx_FSCAL2,
        0x00,//CCx_FSCAL1,
        0x1f,//CCx_FSCAL0,
        0x59,//CCx_FSTEST,
        0x81,//CCx_TEST2,
        0x35,//CCx_TEST1,
        0x09,//CCx_TEST0,
        0xc0,//CCx_PATABLE,
      }
};

// PATABLE (dBm output power)
// stored in progmem to save on RAM
static const byte CCx_paTable[CCX_NR_OF_CONFIGS][CCx_PA_TABLESIZE] PROGMEM ={
      // -30  -20   -15  -10    0   5    7   10
      {
            0x03,0x0E,0x1E,0x27,0x8E,0xCD,0xC7,0xC0      }
};

#endif



