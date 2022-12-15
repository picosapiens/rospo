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
uint8_t triggertype;
uint8_t triggerlevel;
#define TRIGGERTIMEOUT 100

#define CH0input 2
#define offset1Pin 2
#define offset2Pin 3
bool bothchannels = true; // sample both channels when true

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
}

void loop() {
  ADCCounter=0;
  int stopIndex = ADCCounter;
  memset( (void *)ADCBuffer, 0, sizeof(ADCBuffer) ); // clear buffer
  
  //TIMSK0 = 0; // turn off timer0 for lower jitter - delay() and millis() killed
  #warning Not disabling timer
  analogWrite(6,128);

  // Prescaler 32
  //ADCSRA = 0xe5; // set the adc to free running mode
  //dtbuffered_ns = 26000;

  // Prescaler 16
  ADCSRA = 0xe4; // set the adc to free running mode
  dtbuffered_ns = 13000;

  // Prescaler 8
  //ADCSRA = 0xe3;
  //dtbuffered_ns = 6500;

  int cnt = 0;
  bool currentstate;
  bool lookfor;
  uint8_t trigger;
  int triggerindex;
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
  
  DIDR0 = 0x01; // turn off the digital input for adc0

  noInterrupts();
  while(true)
  {
    if(bothchannels)
      ADMUX ^= 0x01; // Toggle last bit to switch between the two channels
    while( !(ADCSRA&(1<<ADIF)) ) // Wait for result (In free running mode so can't use ADSC to check for completion
      if(!keeprunning) break; // Wait for new sample or indication to stop running
    ADCBuffer[ADCCounter] = ADCH; // Grab sample
    sbi(ADCSRA,ADIF); // Apparently writing 1 clears it

    if( (ADMUX&0x01) )
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
  interrupts();


  //Serial.print("DATA ");
  Serial.print('D');
  Serial.print('A');
  Serial.print('T');
  Serial.print('A');
  if(bothchannels)
    Serial.write(0x02); // two channels
  else
    Serial.write(0x01); // one channel
  Serial.write(0x01); // interleaved
  Serial.write(ADCBUFFERSIZE & 255);
  Serial.write( (ADCBUFFERSIZE >> 8) & 255 );
  
  for(int i=0;i<ADCBUFFERSIZE;i++)
  {
    Serial.write(ADCBuffer[(ADCCounter + i) & (ADCBUFFERSIZE-1)]);
    //Serial.print(',');
  }
  //Serial.println("END");
  Serial.print('\n');
  delay(100);

  if(Serial.available() >= SERIALMSGSIZE)
  {
    int incomingByte = 0; // for incoming serial data
    incomingByte = Serial.read();
    int i = 0;
    while(0 <= incomingByte)
    {
      serialbuffer[i] = incomingByte;
      i += 1;
      if (i>SERIALMSGSIZE)
        break;
      incomingByte = Serial.read();
    }
    delay(100); // Give everything a chance to catch up
    // Parse command
    switch(serialbuffer[0])
    {
      case 'C':
        switch(serialbuffer[1])
        {
          case 'H': // CH -- channel enable
          bothchannels = serialbuffer[2];
        }
        break;
      case 'T':
        switch(serialbuffer[1])
        {
          case 'R': // TR -- trigger settings
            //if( '\n' == serialbuffer[8] )
            {
              triggertype = serialbuffer[2];
              triggerlevel = serialbuffer[3];
              //delay(1000);
            }
            break;
        }
        break;
      case 'O':
        switch(serialbuffer[1])
        {
          case 'F': // OF -- offset for negative voltage sensing
            if( serialbuffer[2] )
            {
               digitalWrite(offset1Pin, HIGH);
               digitalWrite(offset2Pin, HIGH);
            } else {
               digitalWrite(offset1Pin, LOW);
               digitalWrite(offset2Pin, LOW);
            }
        }
        break;
    }
    delay(50);
  }
  
}
