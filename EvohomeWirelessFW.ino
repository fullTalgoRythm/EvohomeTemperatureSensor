// EvohomeWirelessFW - RFBee firmware for evohome wireless communications
// Copyright (c) 2015 Hydrogenetic
//
// based on HoneyCommLite - Alternative RFBee firmware to communicate with
//                 Evohome / Hometronix / CM67z and other Honeywell 868MHz based RF devices.
//
// Copyright (C) 2012 JB137
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

// Compile for RFbee using board: Arduino Pro or Pro Mini (3.3V, 8MHz) w/ATmega 168

//some interesting notes on optimisation...
//http://blog.kriegsman.org/2013/12/01/optimizing-10-lines-of-arduino-code-youre-wrong-so-am-i/

#include "CCx.h"
#include "CCxCfg.h"
#include "Buffer.h"

#define VERSION_NO "0.8"

#define GDO0_INT 0 // INT0(PD2) wired to GDO0 on CC1101
#define GDO2_INT 1 // INT1(PD3) wired to GDO2 on CC1101 (CCx_IOCFG2==0x0B Serial Clock. Synchronous to the data in synchronous serial mode. In RX mode, data is set up on the falling edge by CC1101 when GDOx_INV=0. In TX mode, data is sampled by CC1101 on the rising edge of the serial clock when GDOx_INV=0.)

#define GDO0_PD 4 // PD2(INT0) wired to GDO0 on CC1101 (CCx_IOCFG0==0x0C Serial Synchronous Data Output. Used for synchronous serial mode.)
#define GDO2_PD 8 // PD3(INT1) wired to GDO2 on CC1101

#define SYNC_WORD     (uint16_t)0x5595//This is the last 2 bytes of the preamble / sync words..on its own it can be confused with the end of block when the last bit of the checksum is 0 (the following 0x55 pattern is then converted to 0xFF)
#define GW_ID         0x48DADA //This should ideally be unique for every device and some addresses may not be valid

enum progMode{
  pmIdle,
  pmPacketStart,
  pmNewPacket,
  pmSendBadPacket,
  pmSendHavePacket,
  pmSendReady,
  pmSendActive,
  pmSendFinished,
};

enum enflags{
  enDev0=1,
  enDev1=enDev0<<1,
  enDev2=enDev1<<1,
  enRQ=enDev2<<1,
  enRP=enRQ<<1,
  enI=enRP<<1,
  enW=enI<<1,  
};

const byte manc_enc[16]={0xAA,0xA9,0xA6,0xA5,0x9A,0x99,0x96,0x95,0x6A,0x69,0x66,0x65,0x5A,0x59,0x56,0x55};
const byte pre_sync[5]={0xff,0x00,0x33,0x55,0x53};
const byte header_flags[16]={0x0f,0x0c,0x0d,0x0b,0x27,0x24,0x25,0x23,0x47,0x44,0x45,0x43,0x17,0x14,0x15,0x13};

byte out_flags;
byte out_len;

byte in_header;
byte in_flags;

byte unpack_flags(byte header)
{
  return header_flags[(header>>2)&0x0F];
}

byte pack_flags(byte flags)
{
  for(byte n=0;n<16;n++)
    if(header_flags[n]==flags)
      return n<<2;
  return 0xFF;
}

//in strictest terms volatile is only required when an interrupt (or similar) could affect the result of an operation accessing a memory location that may otherwise have been optimised out
volatile CircularBuffer<volatile byte, volatile byte, 255> circ_buffer; //circular buffer (FIFO)
volatile byte send_buffer[128];

byte pm=pmIdle;
volatile byte sm=pmIdle;

byte bit_counter=0;
byte byte_buffer=0;

boolean in_sync=false;
uint16_t sync_buffer=0;
boolean highnib=true;
boolean last_bit;
byte bm;

byte sp=0;
byte op=0;
byte pp=0;

byte check=0;
byte pkt_pos=0;
uint32_t devid;
byte *pDev=(byte*)&devid;
uint16_t cmd;
char tmp[10];
byte len;
char param[10];

// Interrupt to receive data and find_sync_word
void sync_clk_in() {
    byte new_bit=(PIND & GDO0_PD); //sync data
          
    //keep our buffer rolling even when we're in sync
    sync_buffer<<=1;
    if(new_bit)
      sync_buffer++;
  
    if(!in_sync)
    {
      if(sync_buffer==SYNC_WORD)
      {
        circ_buffer.push(0x53,true);
        
        bit_counter=0;
        byte_buffer=0;
        bm=0x10;
        in_sync=true;
      }
    }
    else
    {
      if (bit_counter > 0) // Skip start bit
      {
        if (bit_counter < 9)
        {
          if(bit_counter%2)
          {
            if (new_bit)
              byte_buffer|=bm;
            bm<<=1;
            if(bm==0x10)
            {
              circ_buffer.push(byte_buffer);//we can not see raw 0x35 here as our buffer is already manchester decoded we rely on rejection of 0x35 as it is not manchester encoded to end our packet
              byte_buffer = 0;
            }
            else if(!bm)
              bm=0x01;
          }
          else
          {
           if(new_bit==last_bit)//manchester encoding must always have 1 transition per bit pair
            {
              in_sync=false;
              circ_buffer.push(0x35,true);
              if((sync_buffer&0x7F)==0x2B) //if this might be 0x35 breaking word remove it so it can't be confused with the sync_word
                sync_buffer=0;
              return;
            }
          }
          last_bit=new_bit;
        }
        else
        {  // Last bit has been received
          if(!new_bit)
          {
            in_sync=false;
            circ_buffer.push(0x35,true);
            return;
          }
          bit_counter=0;
          return;
        }
      }
      else
      {
        if(new_bit)
        {
          in_sync=false;
          circ_buffer.push(0x35,true);
          return;
        }
      }
      bit_counter++;
    }
}

//Interrupt to send data
void sync_clk_out() {
    if(sm!=pmSendActive)
      return;
    if(bit_counter<9)
    {
      if(!bit_counter)
      {
        PORTD&=~GDO0_PD;
        if(out_flags<5)
        {
          byte_buffer=0x55;
          out_flags++;
        }
        else if(pp<5)
          byte_buffer=pre_sync[pp++];
        else
        {
          if(sp<op)
          {
            byte_buffer=send_buffer[sp];
            if(highnib)
              byte_buffer=manc_enc[(byte_buffer>>4)&0xF];
            else
            {
              circ_buffer.push(byte_buffer);//echo bytes when sending
              byte_buffer=manc_enc[byte_buffer&0xF];
              sp++;
            }
            highnib=!highnib;
          }
          else if(sp<=op+4)
          {
            if(sp==op)
              byte_buffer=0x35;
            else
              byte_buffer=0x55;
            sp++;
          }
          else
          {
            sm=pmSendFinished;
            return;
          }
        }
      }
      else
      {
        if(byte_buffer&0x01)
          PORTD|=GDO0_PD;
        else
          PORTD&=~GDO0_PD;
        byte_buffer>>=1;
      }
      bit_counter++;
    }
    else
    {
      PORTD|=GDO0_PD;
      bit_counter=0;
    }
}

// Setup
void setup() {
  //from http://forum.arduino.cc/index.php?topic=54623.0
  //an ideal freq of 8.294400 is required for 0% error @ 115200
  //this will cause time delays to be off by a factor of approx 0.964 per sec
  //if OSCCAL is factory calibrated to 8MHz then just adjust by the ratio 
  //... not exactly guaranteed of course!
  //OSCCAL=(double)OSCCAL*1.0368;
  OSCCAL=((uint32_t)OSCCAL*10368+5000)/10000;//Use integer maths here as makes smaller code than fp above
  delay(200); //probably not a bad idea to wait until power etc. have stabilised a little

  // Power up and configure the CC1101
  CCx.PowerOnStartUp();
  CCx.Setup(0);
  while(((CCx.Write(CCx_SIDLE,0)>>4)&7)!=0);
  while(((CCx.Write(CCx_SRX,0)>>4)&7)!=1);//will calibrate when going to rx
  
  // Data is received at 38k4 (packet bytes only at 19k2 due to manchster encoding)
  // 115k2 provides enough speed to perform processing and write the received
  // bytes to serial
  Serial.begin(115200);

  Serial.println(F("# EvohomeWirelessFW v" VERSION_NO " Copyright (c) 2015 Hydrogenetic"));
  Serial.println(F("# Licensed under GPL-3.0+ <http://spdx.org/licenses/GPL-3.0+>"));

  // Attach the find_sync_word interrupt function to the
  // falling edge of the serial clock connected to INT(1)
  attachInterrupt(GDO2_INT, sync_clk_in, FALLING);
}

// Main loop
void loop() {
  if(sm==pmSendFinished)
  {
    detachInterrupt(GDO2_INT);
    in_sync=false;
    bit_counter=0;
    circ_buffer.push(0x35,true);
    pinMode(2,INPUT);
    while(((CCx.Write(CCx_SRX,0)>>4)&7)!=1); 
    attachInterrupt(GDO2_INT, sync_clk_in, FALLING);
    pp=0;
    op=0;
    sp=0;
    sm=pmIdle;
  }
  if (circ_buffer.remain())
  {
    bool mark;
    byte in(circ_buffer.pop(mark));
    
    if(mark)
    {
      if(in==0x53)
      {
        pm=pmPacketStart;
        check=0;
        pkt_pos=0;
      }
      else if(in==0x35)
      {
        if(pm==pmNewPacket)
          Serial.println(F("\x11*INCOMPLETE*"));
        pm=pmIdle;
      }
    }
    else if(pm>pmIdle)
    {
      check+=in;
      if(pkt_pos==0)
      {
        pm=pmNewPacket;
        in_header=in;
        if((in&0xC0) || (in&3)==3) //we don't recognise a header when 2 high reserved bits are set or both parameters bits are set simultaneously (we only have room for 1 parameter in our output - need more feedback could this be a parameter mode?)
          in_flags=0;
        else
          in_flags=unpack_flags(in_header);
        Serial.print("--- ");//dummy val in place of RSSI (095 low presumably -95? and 045 high)
        if(in_flags&enI)
          Serial.print(" I ");
        else if(in_flags&enRQ)
          Serial.print("RQ ");
        else if(in_flags&enRP)
          Serial.print("RP ");
        else if(in_flags&enW)
          Serial.print(" W ");
        else
        {
          Serial.print(F("\x11Unknown header=0x"));
          Serial.println(in,HEX);
          pm=pmIdle;
          return;
        }
        Serial.print("--- ");//parameter not supported... not been observed yet
        pDev=(byte*)&devid+2; //platform specific
      }
      else if(pkt_pos<=6)//ids we only support 2 atm (1,2 or 3 ids are valid)
      {
        *pDev--=in;//platform specific
        if(pkt_pos==3 || pkt_pos==6)
        {
          sprintf(tmp,"%02hu:%06lu ",(uint8_t)(devid>>18)&0x3F,devid&0x3FFFF);
          Serial.print(tmp);
          pDev=(byte*)&devid+2;//platform specific
          if((pkt_pos==3 && !(in_flags&enDev1)) || (pkt_pos==6 && in_flags&enDev1)) //only support 2 ids atm (1,2 or 3 ids are valid)
            Serial.print("--:------ ");
        }
      }
      else if(pkt_pos<=8)//command
      {
        if(pkt_pos==7)
          cmd=in<<8;
        else
        {
          cmd|=in;
          sprintf(tmp,"%04X ",cmd);
          Serial.print(tmp);
        }
      }
      else if(pkt_pos==9)//len
      {
        len=in;
        sprintf(tmp,"%03hu ",len);
        Serial.print(tmp);
      }
      else if(pkt_pos<=9+len)//payload
      {
        sprintf(tmp,"%02hX",in);
        Serial.print(tmp);
      }
      else if(pkt_pos==10+len)//checksum
      {
        if(check==0)
          Serial.println();
        else
          Serial.println(F("\x11*CHK*"));
        pm=pmIdle;
        return;
      }
      else
      {
        Serial.println(F("\x11*E-DATA*"));
        pm=pmIdle;
        return;
      }
      pkt_pos++;
    }
  }
  else if(sm<pmSendReady)
  {
     if(Serial.available())
     {
       char out=Serial.read();
       if(out=='\r')
       {
       }
       else if(out=='\n' && sm==pmSendHavePacket)//send on lf
       {
         byte sc=0;
         for(int n=0;n<op;n++)
           sc+=send_buffer[n];      
         send_buffer[op++]=-sc;
         sm=pmSendReady;
       }
       else if(out=='\x11' || out=='\n')//escape or bad packet
       {
         pp=0;
         op=0;
         sp=0;
         sm=pmIdle;
       }
       else if(sm==pmSendBadPacket)//don't do any more processing if the packet is bad
       {
       }
       else if(sp<7)
       {
         if(out==' ')
         {
           if(pp)
           {
             param[pp]=0;
             if(sp==0){
               out_flags=0;
               if(!strcmp(param,"I"))
                 out_flags|=enI;
               else if(!strcmp(param,"RQ"))
                 out_flags|=enRQ;
               else if(!strcmp(param,"RP"))
                 out_flags|=enRP;
               else if(!strcmp(param,"W"))
                 out_flags|=enW;
               send_buffer[op++]=0;
             }
             else if(sp==1)
             {
             }
             else if(sp>=2 && sp<=4)
             {
               if(param[0]!='-')
               {
                 out_flags|=1<<(sp-2);
                 byte idType;
                 uint32_t idAddr;
                 sscanf(param,"%02hu:%06lu",&idType,&idAddr);
                 if(idType==18 && idAddr==730)
                   idAddr=GW_ID;
                 else
                   idAddr|=(uint32_t)idType<<18;
                 send_buffer[op++]=(idAddr>>16)&0xFF;
                 send_buffer[op++]=(idAddr>>8)&0xFF;
                 send_buffer[op++]=idAddr&0xFF;
               }
             }
             else if(sp==5)
             {
               send_buffer[0]|=pack_flags(out_flags);
               if(send_buffer[0]==0xFF)
               {
                 sm=pmSendBadPacket;
                 return;
               }
               uint16_t cmd;
               sscanf(param,"%04X",&cmd);
               send_buffer[op++]=(cmd>>8)&0xFF;
               send_buffer[op++]=cmd&0xFF;
             }
             else if(sp==6)
             {
               sscanf(param,"%03hu",&out_len);
               send_buffer[op++]=out_len;
             }
             
             pp=0;
             sp++;
           }
         }
         else if(pp<9)
         {
           param[pp++]=out;
         }
       }
       else
       {
         param[pp++]=out;
         if(pp==2)
         {
           param[pp]=0;
           byte pay;
           sscanf(param,"%02hX",&pay);
           send_buffer[op++]=pay;
           pp=0;
           if(op>(out_len+10) || op>127)
             sm=pmSendBadPacket;
           else if(op==(out_len+10))
             sm=pmSendHavePacket;
         }
       }
     }
  }
  else if(sm==pmSendReady && pm!=pmNewPacket)
  {
       detachInterrupt(GDO2_INT);
       while(((CCx.Write(CCx_SIDLE,0)>>4)&7)!=0);
       while(((CCx.Write(CCx_STX,0)>>4)&7)!=2);//will calibrate when going to tx
       pinMode(2,OUTPUT);
       sm=pmSendActive;
       highnib=true;
       bit_counter=0;
       sp=0;
       pp=0;
       out_flags=0;//reuse for preamble counter
       circ_buffer.push(0x53,true); //don't push anything while interrupt is running
       attachInterrupt(GDO2_INT, sync_clk_out, RISING);
  }
}













