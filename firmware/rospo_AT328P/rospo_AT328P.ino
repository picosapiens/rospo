//-----------------------------------------------------------------------------
// rospo_AT328P.ino
//-----------------------------------------------------------------------------
//
// This file is part of Rospo.
//
//  Rospo is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  Rospo is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with Rospo.  If not, see <http://www.gnu.org/licenses/>.
//


// ADCBUFFERSIZE MUST BE A POWER OF 2
#define ADCBUFFERSIZE  1024

uint16_t ADCCounter=0;
uint8_t ADCBuffer[ADCBUFFERSIZE];
uint32_t dtbuffered_ns; // nominal time step

#define SERIALBUFFERSIZE 32
#define SERIALMSGSIZE 9
uint8_t serialbuffer[SERIALBUFFERSIZE];

#define NOTRIGGER 0
#define RISINGTRIGGER 1
#define FALLINGTRIGGER 2
#define TRIGGERMODES 3
uint8_t triggertype;
uint8_t triggerlevel;
#define TRIGGERTIMEOUT 100
uint8_t adcprescaler = 16;

#define CH0input 2
#define offset1Pin 2
#define offset2Pin 3
bool bothchannels = true; // sample both channels when true
uint16_t triggerindex;

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif


void setup() {
  Serial.begin(115200);
  //Serial.println("Hello");
  triggertype = NOTRIGGER;
  triggerlevel = 128;
  pinMode(offset1Pin, OUTPUT);
  pinMode(offset2Pin, OUTPUT);
  digitalWrite(offset1Pin, LOW);
  digitalWrite(offset2Pin, LOW);
}

void loop() {
  ADCCounter=0;
  int stopIndex = ADCCounter;
  memset( (void *)ADCBuffer, 0, sizeof(ADCBuffer) ); // clear buffer
  
  //TIMSK0 = 0; // turn off timer0 for lower jitter - delay() and millis() killed
  //analogWrite(6,128);

  switch( adcprescaler )
  {
    case 32:
      ADCSRA = 0xe5;
      dtbuffered_ns = 26000;
      break;
    case 16:
      ADCSRA = 0xe4;
      dtbuffered_ns = 13000;
      break;
    case 8:
      ADCSRA = 0xe3;
      dtbuffered_ns = 6500;
      break;
  }
  if(bothchannels)
    dtbuffered_ns *= 2;

  int cnt = 0;
  bool currentstate;
  bool lookfor;
  uint8_t trigger;
  bool keeprunning = true;
  if(NOTRIGGER == triggertype)
  {
    trigger = 0;
    stopIndex = ADCCounter; // fill the whole buffer
  } else {
    trigger = 4;
    stopIndex = ADCBUFFERSIZE+1; // will never reach this, but will update when triggered
  }
  switch(triggertype)
  {
    case RISINGTRIGGER:
      lookfor = false;
      break;
    case FALLINGTRIGGER:
      lookfor = true;
      break;
  }
  
  ADMUX = 0x60 | CH0input; // use adc0, and set adlar for left adjust
  
  DIDR0 = 0x0C; // turn off the digital input for adc0 to reduce power consumption

  noInterrupts();
  // Enable ADC
  sbi(ADCSRA,ADEN);
  // Start conversion
  sbi(ADCSRA,ADSC);
  while(true)
  {
    if(bothchannels)
      ADMUX ^= 0x01; // Toggle last bit to switch between the two channels
    while( !(ADCSRA&(1<<ADIF)) ) // Wait for result (In free running mode so can't use ADSC to check for completion)
      if(!keeprunning) break; // Wait for new sample or indication to stop running
    ADCBuffer[ADCCounter] = ADCH; // Grab sample (Just the top 8 bits in the interest of conserving memory)
    sbi(ADCSRA,ADIF); // writing 1 clears it
    
    if( !bothchannels || (ADMUX&0x01) )
      currentstate = (ADCBuffer[ADCCounter]>=triggerlevel);   
    switch(trigger)
    {
      case 4: // trigger-disabled period ( to make sure we fill at least ADCBUFFERSIZE-waitDuration before trigger )
        if( ADCCounter > 256 )
          trigger--;
        break;
      case 3: // waiting
        if(currentstate == lookfor)
        {
          lookfor = !lookfor;
          trigger--;
        }
        break;
      case 2: // armed
        if(currentstate == lookfor)
        {
          trigger--;
        }
        triggerindex = ADCCounter;
        break;
      case 1: // triggered
        stopIndex = ( triggerindex + ADCBUFFERSIZE - 256 ) & (ADCBUFFERSIZE-1);
        trigger--;
        break;
    }
    
    ADCCounter = (ADCCounter+1) & (ADCBUFFERSIZE-1);
    if( stopIndex == ADCCounter)
      break;
    if( 0 == ADCCounter )
      cnt += 1;
    if( TRIGGERTIMEOUT < cnt )
      break;
  }
  // Disable ADC and stop Free Running Conversion Mode
  cbi(ADCSRA,ADEN);
  interrupts();


  //Serial.print("DATA ");
  Serial.print('D');
  Serial.print('A');
  Serial.print('T');
  Serial.print('A');
  delayMicroseconds(100);
  if(bothchannels)
    Serial.write(0x02); // two channels
  else
    Serial.write(0x01); // one channel
  Serial.write(0x01); // interleaved
  Serial.write(ADCBUFFERSIZE & 255);
  Serial.write( (ADCBUFFERSIZE >> 8) & 255 );
  delayMicroseconds(100);
  Serial.write( dtbuffered_ns & 255 );
  Serial.write( (dtbuffered_ns>>8)&255 );
  uint16_t adjustedtriggerindex = (triggerindex + ADCBUFFERSIZE - ADCCounter ) & (ADCBUFFERSIZE-1);
  Serial.write(adjustedtriggerindex & 255);
  Serial.write( (adjustedtriggerindex >> 8) & 255 );
  
  for(int i=0;i<ADCBUFFERSIZE;i++)
  {
    Serial.write(ADCBuffer[(ADCCounter + i) & (ADCBUFFERSIZE-1)]);
    delayMicroseconds(100);
  }
  Serial.print('E');
  Serial.print('N');
  Serial.print('D');
  //Serial.print('\n');
  delayMicroseconds(100);

  int keepwaiting = 0;
  memset( (void *)serialbuffer, 0, sizeof(serialbuffer) ); // clear buffer
  while(keepwaiting < 1000)
  {
    int incomingByte = 0; // for incoming serial data
    incomingByte = Serial.read();
    int i = 0;
    int counter = 0;
    while(0 <= incomingByte)
    {
      serialbuffer[i] = incomingByte;
      i += 1;
      if (i==SERIALMSGSIZE)
        break;
      incomingByte = Serial.read();
      if( counter++ > 1000 )
        return;
    }
    delayMicroseconds(100);
    // Parse command
    if( serialbuffer[SERIALMSGSIZE-1] == 'X' ) // Check for end character to make sure message didn't get garbled
      switch(serialbuffer[0])
      {
     /*   case 'C':
          switch(serialbuffer[1])
          {
            case 'H': // CH -- channel enable
              bothchannels = (0!=serialbuffer[2]);
              break;
          }
          break;*/
        case 'R':
          switch(serialbuffer[1])
          {
            case 'N': // RN -- run scope
              keepwaiting = 10000;
              break;
          }
          break;
        case 'T':
          switch(serialbuffer[1])
          {
            case 'R': // TR -- trigger settings
              triggertype = serialbuffer[2];
              if(triggertype >= TRIGGERMODES)
                triggertype = NOTRIGGER;
              triggerlevel = serialbuffer[3];
              break;
          }
          break;
        case 'S':
          switch(serialbuffer[1])
          {
            case 'T': // ST -- settings
              if( (0!=serialbuffer[2]) )
              {
                 digitalWrite(offset1Pin, HIGH);
                 digitalWrite(offset2Pin, HIGH);
              } else {
                 digitalWrite(offset1Pin, LOW);
                 digitalWrite(offset2Pin, LOW);
              }
              if( (0!=serialbuffer[3]) )
              {
                bothchannels = true;
              } else {
                bothchannels = false;
              }
              if( (0!=serialbuffer[4]) )
              {
                adcprescaler = 8;
              } else {
                adcprescaler = 16;
              }
              break;
          }
          break;
      }
    delayMicroseconds(50);
    keepwaiting++;
  }
  
}
