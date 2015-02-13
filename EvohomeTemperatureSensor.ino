// EvohomeTemperatureSensor - RFBee firmware for evohome compatible wireless temperature sensor
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
// Includes a stripped down version of DHT library written by Adafruit Industries
// (originally distributed under the MIT license)
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

// Set voltage switch on UartSBee to 3.3v
// Compile for RFbee using board: Arduino Pro or Pro Mini (3.3V, 8MHz) w/ATmega 168

// some interesting notes on optimisation...
// http://blog.kriegsman.org/2013/12/01/optimizing-10-lines-of-arduino-code-youre-wrong-so-am-i/

#include "CCx.h"
#include "CCxCfg.h"
#include "Buffer.h"
#include "DHT.h"
//#include <sleep_bod_disable.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>

#define VERSION_NO "0.5"

//Uncomment the line bellow to enable power on bind if you do not want to connect a bind button. The bind message will then be sent once at power on.
//#define POBIND

//Uncomment the line bellow for some serial debugging messages @ 115200 baud 8N1
//#define DEBUG

#define DHTPIN 4     // Use PD4 for Temp Sensor
#define BTNPIN 8     // Use PB0/PCINT0 for bind button

// Uncomment whatever type you're using!
//#define DHTTYPE DHT11   // DHT 11 
#define DHTTYPE DHT22   // DHT 22  (AM2302)
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

// Connect pin 1 (on the left) of the sensor to +3.3V 
// NOTE: If using a board with 3.3V logic like an Arduino Due connect pin 1
// to 3.3V instead of 5V!
// Connect pin 2 of the sensor to whatever your DHTPIN is (PD4 see above)
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

//For 8Mhz Arduino...
#define DHTTHCOUNT 3

#define GDO0_INT 0 // INT0(PD2) wired to GDO0 on CC1101
#define GDO2_INT 1 // INT1(PD3) wired to GDO2 on CC1101 (CCx_IOCFG2==0x0B Serial Clock. Synchronous to the data in synchronous serial mode. In RX mode, data is set up on the falling edge by CC1101 when GDOx_INV=0. In TX mode, data is sampled by CC1101 on the rising edge of the serial clock when GDOx_INV=0.)

#define GDO0_PD 4 // PD2(INT0) wired to GDO0 on CC1101 (CCx_IOCFG0==0x0C Serial Synchronous Data Output. Used for synchronous serial mode.)
#define GDO2_PD 8 // PD3(INT1) wired to GDO2 on CC1101

#define SYNC_WORD     (uint16_t)0x5595//This is the last 2 bytes of the preamble / sync words..on its own it can be confused with the end of block when the last bit of the checksum is 0 (the following 0x55 pattern is then converted to 0xFF)
#define DEV_ID        (uint32_t)0x1CDADA //This should ideally be unique for every device and some addresses may not be valid
#define EVO_CMD_TEMP  (uint16_t)0x30c9
#define EVO_CMD_BIND  (uint16_t)0x1FC9

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
byte tp=0;

byte check=0;
byte pkt_pos=0;
uint32_t devid;
byte *pDev=(byte*)&devid;
uint16_t cmd;
char tmp[20];
byte len;

byte br_zone;
uint16_t br_cmd;
uint32_t br_dev;

#ifdef DEBUG
byte sp_zone;
int16_t sp_temp;
#endif

byte bindmode=1;//This will force temperature packet init before sending any temp data
uint8_t time_els=0;
uint8_t time_elm=0;
volatile uint8_t f_wdt=0;
volatile uint8_t btnchk=0;

int16_t nTemp=0,nLastTemp=0;
//uint16_t nHum=0,nLastHum=0;

unsigned long time_last_push;

//details about WDT came from http://donalmorrissey.blogspot.co.uk/2010/04/sleeping-arduino-part-5-wake-up-via.html
ISR(WDT_vect)
{
  if(f_wdt == 0)
  {
    f_wdt=1;
    time_els++;
  }
}

//see http://forum.arduino.cc/index.php?topic=108287.0
//also http://www.seeedstudio.com/wiki/WDT_Sleep
//It would be best to determine what can be permanently turned off rather than turning things on and off unnecessarily

void enterSleep(){
  cli();
  sleep_enable();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); //set sleep mode
  //sleep_bod_disable(); //FIXME would this need to be re-enabled?
  sei();
  
  // disable ADC
  /*ADCSRA = 0; 

  power_adc_disable();
  power_spi_disable();
  power_timer0_disable();
  power_timer1_disable();
  power_timer2_disable();
  power_twi_disable();
  
  byte i;
  for (i = 0; i <= 13; i++)
    pinMode (i, OUTPUT);     // as required
  for (i = 0; i <= 13; i++)
    digitalWrite (i, LOW);  // as required
    
  //according to arduino forum setting pins low will save power regardless of pinmode
  //but we'd have to restore the correct settings afterwards
  
  MCUCR = _BV (BODS) | _BV (BODSE);  // turn on brown-out enable select
  MCUCR = _BV (BODS);        // this must be done within 4 clock cycles of above*/
  
  power_all_disable(); 
  sleep_cpu();
  
  // The program will continue from here after the WDT timeout
  sleep_disable(); // First thing to do is disable sleep. 
  
  // Re-enable the peripherals. 
  power_all_enable();
}

boolean DHTread() {
  uint8_t laststate = HIGH;
  uint8_t counter = 0;
  uint8_t j = 0, i;
  uint8_t data[6];
  
  data[0] = data[1] = data[2] = data[3] = data[4] = 0;
  
  // pull the pin high and wait 250 milliseconds
  digitalWrite(DHTPIN, HIGH);
  delay(250);

  // now pull it low for ~20 milliseconds
  pinMode(DHTPIN, OUTPUT);
  digitalWrite(DHTPIN, LOW);
  delay(20);
  noInterrupts();
  digitalWrite(DHTPIN, HIGH);
  delayMicroseconds(40);
  pinMode(DHTPIN, INPUT);

  // read in timings
  for ( i=0; i< MAXTIMINGS; i++) {
    counter = 0;
    while (digitalRead(DHTPIN) == laststate) {
      counter++;
      delayMicroseconds(1);
      if (counter == 255) {
        break;
      }
    }
    laststate = digitalRead(DHTPIN);

    if (counter == 255) break;

    // ignore first 3 transitions
    if ((i >= 4) && (i%2 == 0)) {
      // shove each bit into the storage bytes
      data[j/8] <<= 1;
      if (counter > DHTTHCOUNT)
        data[j/8] |= 1;
      j++;
    }

  }

  interrupts();

  // check we read 40 bits and that the checksum matches
  if ((j >= 40) && 
      (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) ) {
    switch (DHTTYPE) { //Should optimise out unnecessary code
    case DHT11:
      nTemp = data[2];
      nTemp*=100;
      //nHum = data[0];
      break;
    case DHT22:
    case DHT21:
      nTemp = ((data[2] & 0x7F) << 8) + data[3];
      if (data[2] & 0x80)
	nTemp *= -1;
      nTemp*=10;
      //nHum = (data[0] << 8) + data[1];
      break;
    }
    return true;
  }
  
  return false;
}


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
              //circ_buffer.push(byte_buffer);//echo bytes when sending
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

void addId(uint32_t devId) {
  send_buffer[tp++]=(devId>>16)&0xFF;
  send_buffer[tp++]=(devId>>8)&0xFF;
  send_buffer[tp++]=devId&0xFF;
}

void addU16(uint16_t cmd, byte &sp) {
  send_buffer[sp++]=(cmd>>8)&0xFF;
  send_buffer[sp++]=cmd&0xFF;
}

void initSendBCast(uint16_t cmd) {
  tp=0;
  send_buffer[tp++]=0x18;//I dev0 dev2 (broadcast)

  for(uint8_t n=0;n<2;n++)
    addId(DEV_ID);

  addU16(cmd,tp);
}

void initSendTemp() {
  initSendBCast(EVO_CMD_TEMP);

  send_buffer[tp++]=3;
  
  send_buffer[tp++]=0;
}

//Controller returns 3 commands 0x2309 (set point) 0x30c9 (temp) 0x1fc9 (bind)
void initSendBind() {
  bindmode=1;
  initSendBCast(EVO_CMD_BIND);

  send_buffer[tp++]=6;
  
  send_buffer[tp++]=0;
  addU16(EVO_CMD_TEMP,tp);
  addId(DEV_ID);
  op=tp;
}

void sendPacket() {
  byte sc=0;
  for(byte n=0;n<op;n++)
    sc+=send_buffer[n];      
  send_buffer[op++]=-sc;
  
  detachInterrupt(GDO2_INT);
  while(((CCx.Write(CCx_STX,0)>>4)&7)!=2);//will calibrate when going to tx
  pinMode(2,OUTPUT);
  sm=pmSendActive;
  highnib=true;
  bit_counter=0;
  sp=0;
  pp=0;
  out_flags=0;//reuse for preamble counter
  attachInterrupt(GDO2_INT, sync_clk_out, RISING);
}

// Setup
void setup() {
  delay(200); //probably not a bad idea to wait until power etc. have stabilised a little

  // Power up and configure the CC1101
  CCx.PowerOnStartUp();
  CCx.Setup(0);
  while(((CCx.Write(CCx_SIDLE,0)>>4)&7)!=0);

#ifdef DEBUG  
  Serial.begin(115200);
  Serial.println(F("# EvohomeTempSensor v0.5 Copyright (c) 2015 Hydrogenetic"));
  Serial.println(F("# Licensed under GPL-3.0+ <http://spdx.org/licenses/GPL-3.0+>"));
  delay(11);//we may need to delay a little as sleep will prevent interrupts running and hence freeze the serial port
#endif

  pinMode(DHTPIN, INPUT);
  digitalWrite(DHTPIN, HIGH);//use the internal pullup resistor
  
  //Setup the pin for our bind button if required
#ifndef POBIND
  pinMode(BTNPIN, INPUT);
  digitalWrite(BTNPIN, HIGH);//use the internal pullup resistor
  
  //Pin Change Interrupt Control Register
  PCMSK0=0x01;//binary 0b00000001, enables PCINT0 (PB0)
  PCIFR|=0x01;//clear interrupt flag PCIE0 (port B)
  PCICR=0x01;//enable interrupt PCIE0 (port B) only
#endif
  
  // Setup the WDT
  cli();
  MCUSR &= ~(1<<WDRF); //Clear the reset flag.
  WDTCSR |= (1<<WDCE) | (1<<WDE);//In order to change WDE or the prescaler, we need to set WDCE (This will allow updates for 4 clock cycles).
  WDTCSR = 1<<WDP0 | 1<<WDP3; //Set new watchdog timeout prescaler value for 8.0 seconds
  WDTCSR |= _BV(WDIE); //Enable the WD interrupt (note no reset).
  sei();
  
#ifdef POBIND //send a bind signal on power on (because we don't want to connect up a bind button)
  initSendBind();
  sendPacket();
#else
  CCx.Strobe(CCx_SPWD); //enter sleep mode when CSn goes high (should be after strobe)
  enterSleep();
#endif
}

//ISR for PCINT0 (PB0)
ISR(PCINT0_vect) {
  if(!btnchk)
    btnchk=1;
}

void buttonCheck(){
  unsigned long dt=millis()-time_last_push;//this will overflow every 50 days
  if(dt<50)//debounce
    return;
  
  if(digitalRead(BTNPIN))//This will be when the button is released due to internal pullup (high)
  {
    if(dt>1000 && dt<10000 && sm==pmIdle)
    {
      sm=pmSendHavePacket;
      while(((CCx.Write(CCx_SRX,0)>>4)&7)!=1);//will calibrate when going to rx
#ifdef DEBUG  
      Serial.println(F("Sending binding message"));
#endif
      initSendBind();
      sendPacket();
    }
    time_last_push=millis();
    btnchk=0;
  }
}

// Main loop
void loop() {
  if(btnchk)
  {
    if(btnchk==1)
    {
      time_last_push=millis();
      btnchk=2;
    }
    buttonCheck();
  }
  else if(f_wdt)
  {
    f_wdt=0;
    if(time_els>=8) //approx every min
    {
      time_els=0;
      time_elm++; //approx min counter
      if(DHTread())
      {
#ifdef DEBUG  
        Serial.print(F("Temp: "));
        Serial.println(nTemp,DEC);
        delay(1);
#endif
        //the 1f09 message is also implied in message timing & sync
        if(abs(nLastTemp-nTemp)>=20 || time_elm>=56)//.2 deg or approx every hour? (guess work from logs - seems to be around 2 min minimum interval or .2 deg minimum change)
        {
          if(sm!=pmIdle) //If someone hit the bind button wait for bind to complete
            return;
#ifdef DEBUG  
          Serial.println(F("Sending temp"));
          delay(1);
#endif
          sm=pmSendHavePacket; //Prevent bind asap
          //Sleep mode will reset some register which should be reloaded as required
          //Wakeup to RX mode sendpacket will switch to TX as required
          while(((CCx.Write(CCx_SRX,0)>>4)&7)!=1);//will calibrate when going to rx
          if(bindmode)
          {
            bindmode=0;
            initSendTemp();
          }
          op=tp;
          addU16(nTemp,op);//cast should not change underlying bits
          sendPacket();
       
          nLastTemp=nTemp;
          time_elm=0;
          return;//wait until send finished to enter sleep
        }
      }
    }
    //FIXME we should check if we are expecting messages and delay sleep if so
    if(sm==pmIdle)
      enterSleep();
  }
  if(sm==pmSendFinished)
  {
    detachInterrupt(GDO2_INT);
    sm=pmIdle;
    in_sync=false;
    bit_counter=0;
    pinMode(2,INPUT);
    //Sleep mode will reset some register which should be reloaded as required
    if(bindmode) //do a 1 off bind when powered on then reset to temp send mode
    {
      while(((CCx.Write(CCx_SRX,0)>>4)&7)!=1); 
      attachInterrupt(GDO2_INT, sync_clk_in, FALLING); 
    }
    else//FIXME we should check if we are expecting messages and delay sleep if so
    {
      while(((CCx.Write(CCx_SIDLE,0)>>4)&7)!=0);
      CCx.Strobe(CCx_SPWD); //enter sleep mode when CSn goes high (should be after strobe)
      enterSleep();
    }
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
        //if(pm==pmNewPacket) Serial.println(F("\x11*INCOMPLETE*"));
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
        if((in&0xC0) || (in&3)==3) //we don't recognise a header when 2 high reserved bits are set or both parameters bits are set simultaneously (need more feedback could this be a parameter mode?)
          in_flags=0;
        else
          in_flags=unpack_flags(in_header);
        if(in_flags&enI);
        else if(in_flags&enRQ);
        else if(in_flags&enRP);
        else if(in_flags&enW);
        else
        {
          pm=pmIdle;
          return;
        }
        //optional parameter(s) after dev0-2 not supported... not been observed yet
        pDev=(byte*)&devid+2; //platform specific
      }
      else if(pkt_pos<=6)//ids we only support 2 atm (1,2 or 3 ids are valid)
      {
        *pDev--=in;//platform specific
        if(pkt_pos==3 || pkt_pos==6)
          pDev=(byte*)&devid+2;//platform specific
      }
      else if(pkt_pos<=8)//command
      {
        if(pkt_pos==7)
          cmd=in<<8;
        else
          cmd|=in;
      }
      else if(pkt_pos==9)//len
      {
        len=in;
      }
      else if(pkt_pos<=9+len)//payload
      {
        //we could potentially lookout for the 2309 set point message
        //the 1f09 message may contain details about timing at the very least
        //we could use it to time and sync when to wakeup and switch to RX
        //I'm not sure if it would be worth changing the behaviour of our sensor
        //at or near the set point unless we are creating a thermostat
        if(cmd==0x1fc9) //binding message
        {
          byte ofs=(pkt_pos-10)%6;//FIXME position not guaranteed
          if(!ofs)
            br_zone=in;
          else if(ofs<3)
          {
            *((byte*)&br_cmd+1-(ofs-1))=in;
          }
          else
          {
            *((byte*)&br_dev+2-(ofs-3))=in;
#ifdef DEBUG  
            //we could store the binding details in our EPROM
            //this would confirm our sensor is bound and 
            //give us the controller id and zone number if required
            if(ofs==5)
            {
              Serial.print(F("BIND "));
              sprintf(tmp,"[%03hu ",br_zone);
              Serial.print(tmp);
              sprintf(tmp,"%04X ",br_cmd);
              Serial.print(tmp);
              sprintf(tmp,"%02hu:%06lu]",(uint8_t)(br_dev>>18)&0x3F,br_dev&0x3FFFF);
              Serial.println(tmp);
            }
#endif
          }
        }
#ifdef DEBUG //the following messages are also sent after binding presumably to init our sensor
        else if(cmd==0x1f09)//misc message
        {
          if(pkt_pos==10)//FIXME position not guaranteed
          {
              sprintf(tmp,"%04X [",cmd);
              Serial.print(tmp);
          }
          sprintf(tmp,"%02hX",in);
          Serial.print(tmp);
        }
        else if(cmd==0x0004)//Zone Name
        {
          if(pkt_pos==10)//FIXME position not guaranteed
          {
            Serial.print(F("ZONE_NAME "));
            sprintf(tmp,"[%03hu ",in);
            Serial.print(tmp);
          }
          else if(in)
            Serial.print((char)in);
        }
        else if(cmd==0x2309)//Set Point
        {
          byte ofs=(pkt_pos-10)%3;//FIXME position not guaranteed
          if(!ofs)
            sp_zone=in;
          else if(ofs<3)
          {
            *((byte*)&sp_temp+1-(ofs-1))=in;
            if(ofs==2)
            {
              Serial.print(F("SET_POINT "));
              sprintf(tmp,"[%03hu ",sp_zone);
              Serial.print(tmp);
              sprintf(tmp,"%d",sp_temp);
              Serial.print(tmp);
            }
          }
        }   
#endif
      }
      else if(pkt_pos==10+len)//checksum
      {
        if(check==0) //checksum ok
        {
          if(cmd==0x1fc9) //FIXME we're just using this to sleep right now but the WDT could interfere.. so should use more care if we really want to read messages
          {
            //Force a send of our temp at next timeout now we're bound (in approx 30 sec)
            time_els=4;
            time_elm=55;
#ifndef DEBUG //dont sleep when debugging so we can pick up a few extra messages...sleep will be triggered automatically by the WDT later on anyway (and could make us miss those messages)
            detachInterrupt(GDO2_INT);
            while(((CCx.Write(CCx_SIDLE,0)>>4)&7)!=0);
            CCx.Strobe(CCx_SPWD); //enter sleep mode when CSn goes high (should be after strobe)
            enterSleep();
#endif
          }
#ifdef DEBUG //the following messages are also sent after binding presumably to init our sensor
        else if(cmd==0x1f09 || cmd==0x0004 || cmd==0x2309)//misc message
          Serial.println(F("]"));
#endif
        }
        pm=pmIdle;
        return;
      }
      else //extra data
      {
        pm=pmIdle;
        return;
      }
      pkt_pos++;
    }
  }
}













