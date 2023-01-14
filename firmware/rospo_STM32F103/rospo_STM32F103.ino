#include "libmaple/dma.h"

// BUFFER SIZE MUST BE A POWER OF TWO
#define analogInPin1 PA1
#define analogInPin2 PA0
#define ADCBUFFERSIZE 4096
//define LED_BUILTIN PC13

uint16_t ADCCounter=0;
uint16_t ADCBuffer[ADCBUFFERSIZE];
uint32_t dtbuffered_ns; // nominal time step

#define SERIALBUFFERSIZE 32
#define SERIALMSGSIZE 9
uint8_t serialbuffer[SERIALBUFFERSIZE];

#define NOTRIGGER 0
#define RISINGTRIGGER 1
#define FALLINGTRIGGER 2
uint8_t triggertype = NOTRIGGER;
uint16_t triggerlevel = 128;
#define TRIGGERTIMEOUT 1000

#define offset1Pin PB7
#define offset2Pin PB8
bool bothchannels = true; // sample both channels when true
bool applyoffsets = false;
bool highspeed = false;
uint16_t triggerindex;

// Defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

/*
void capture() {
  if( applyoffsets )
  {
     digitalWrite(offset1Pin, HIGH);
     digitalWrite(offset2Pin, HIGH);
  } else {
     digitalWrite(offset1Pin, LOW);
     digitalWrite(offset2Pin, LOW);
  }

  // Configure ADC speed
  // 
  if(highspeed)
  {
    adc_set_prescaler(ADC_PRE_PCLK2_DIV_2); // 36 MHz ADC Clock
    adc_set_sample_rate(ADC1, ADC_SMPR_1_5); // Sample for 1.5 ADC clock cycles
    adc_set_sample_rate(ADC2, ADC_SMPR_1_5);
  } else {
    adc_set_prescaler(ADC_PRE_PCLK2_DIV_4); // 18 MHz ADC Clock
    adc_set_sample_rate(ADC1, ADC_SMPR_7_5); // Sample for 7.5 ADC clock cycles
    adc_set_sample_rate(ADC2, ADC_SMPR_7_5);
  }
  

  // Set up Direct Memory Access (DMA) for the ADC's.
  // Note that we can't set up separate transfers for the two ADC's. They have to be in Simultaneous mode, which combines them
  // into a 32-bit full word rather than the standard 16-bit half word for a single ADC. The 32 bit word holds two 12-bit values,
  // with ADC1 and ADC2 in bits 0-11 and 16-27, respectively.
  dma_init(DMA1);
  //dma_attach_interrupt(DMA1, DMA_CH1, func); // Don't think I want to call a function upon interrupt
  dma_setup_transfer( DMA1, DMA_CH1, &ADC1->regs->DR, DMA_SIZE_32BITS, ADCBuffer, DMA_SIZE_32BITS, (DMA_CCR_MINC | DMA_CCR_CIRC ) ); // (DMA_CIRC_MODE|DMA_MINC_MODE) ??
  dma_set_num_transfers(DMA1, DMA_CH1, ADCBUFFERSIZE/2);
  dma_enable(DMA1, DMA_CH1);
  if(bothchannels)
  {
    ADC1->regs->CR1 |= 6 << 16;      // Regular simultaneous mode. Required for ADC1 only. ADC2 will follow.
    ADC1->regs->SQR3 = PIN_MAP[analogInPin1].adc_channel;
    ADC2->regs->SQR3 = PIN_MAP[analogInPin2].adc_channel;
  } else {
    ADC1->regs->CR1 |= 7 << 16; // Fast interleaved mode
    ADC1->regs->SQR3 = PIN_MAP[analogInPin1].adc_channel;
    ADC2->regs->SQR3 = PIN_MAP[analogInPin1].adc_channel; // Same pin for single channel mode
  }
  ADC1->regs->CR2 |= ADC_CR2_DMA;     //enable ADC DMA transfer
  ADC1->regs->CR2 |= ADC_CR2_CONT;    //Set the ADC in Continuous Mode
  ADC2->regs->CR2 |= ADC_CR2_DMA;     //enable ADC DMA transfer
  ADC2->regs->CR2 |= ADC_CR2_CONT;    //Set the ADC in Continuous Mode

  int cnt = 0;
  //bool currentstate;
  bool lookfor;
  uint8_t trigger;
  bool keeprunning = true;
  uint32_t stopIndex;
  if(NOTRIGGER == triggertype)
  {
    trigger = 1;
    stopIndex = ADCBUFFERSIZE; // fill the whole buffer
    triggerindex = 0;
  } else {
    trigger = 5;
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
  
  bool done = false;
  uint16_t ADCcpuindex = 0;
  uint16_t prevADCcpuindex = 0;
  uint16_t ADCdmaindex;
  uint16_t numsamples;
  // Start conversion
  ADC1->regs->CR2 |= ADC_CR2_SWSTART;
  //ADC2->regs->CR2 |= ADC_CR2_SWSTART;
  //delay(1);
  //currentstate = (ADCBuffer[0]>=triggerlevel);   
  
  while(true)
  {
    //if(bothchannels)
      ADCdmaindex = 2*(ADCBUFFERSIZE/2 - DMA1->regs->CNDTR1);
    //else
    //  ADCdmaindex = ADCBUFFERSIZE - DMA1->regs->CNDTR1;
    if( ADCdmaindex >= ADCcpuindex )
      numsamples = ADCdmaindex - ADCcpuindex;
    else
      numsamples = ADCBUFFERSIZE + ADCdmaindex - ADCcpuindex;
    for(uint32_t idx = 0; idx < numsamples; idx += 2)             // *** THIS SOFTWARE TRIGGER MAY NOT BE WORKABLE AT HIGH SPEED - maybe capture first and retroactively look for trigger?
    {
      ADCCounter = (ADCcpuindex + idx)&(ADCBUFFERSIZE-1);
      //if( ADCCounter >= ADCBUFFERSIZE )
      //  ADCCounter = (ADCCounter)&(ADCBUFFERSIZE-1);
      bool currentstate = (ADCBuffer[ADCCounter]>=triggerlevel);   
      switch(trigger)
      {
        case 5: // trigger-disabled period ( to make sure we fill at least ADCBUFFERSIZE-waitDuration before trigger )
          if( ADCCounter > 256 )
            trigger--;
          break;
        case 4: // waiting
          if(currentstate == lookfor)
          {
            lookfor = !lookfor;
            trigger--;
          }
          break;
        case 3: // armed
          if(currentstate == lookfor)
          {
            trigger--;
            triggerindex = ADCCounter;
          }
          break;
        case 2: // triggered
          stopIndex = ( triggerindex + ADCBUFFERSIZE - 256 ) & (ADCBUFFERSIZE-1);
          trigger--;
          break;
        case 1:
          //if( ADCcpuindex < ADCdmaindex )
          //{
            if( ADCCounter == stopIndex )//(stopIndex>=prevADCcpuindex) && (stopIndex<=ADCcpuindex) )
            {
              dma_disable(DMA1,DMA_CH1); // We will have overshot by the time we get here
              ADC1->regs->CR2 &= ~ADC_CR2_CONT;
              ADC2->regs->CR2 &= ~ADC_CR2_CONT;
              done = true;
              trigger--;
            }
          //} else {
          //  if( (stopIndex>prevADCcpuindex && stopIndex<ADCBUFFERSIZE) || (stopIndex<ADCcpuindex ) )
          //    dma_disable(DMA1,DMA_CH1);
          //    ADC1->regs->CR2 &= ~ADC_CR2_CONT;
          //    ADC2->regs->CR2 &= ~ADC_CR2_CONT;
          //    done = true;
          //}
          //trigger--;
          break;
      }
    }
    if( prevADCcpuindex > ADCcpuindex)
      cnt += 1;
    prevADCcpuindex = ADCcpuindex;
    ADCcpuindex = ADCdmaindex;
    if(done)
      break;
    if( TRIGGERTIMEOUT < cnt )
      break;
  }
  
  // Stop conversion
  //dma_disable(DMA1,DMA_CH1); // Handled above to get it done immediately.
  //ADC1->regs->CR2 &= ~ADC_CR2_CONT;
  //ADCCounter = (ADCCounter+2)&(ADCBUFFERSIZE-1);
  ADCCounter = ( 2*(ADCBUFFERSIZE/2 + 1 - DMA1->regs->CNDTR1) ) & ( ADCBUFFERSIZE - 1);
  //if(bothchannels)
    //  ADCdmaindex = ( 2*(ADCBUFFERSIZE/2 - DMA1->regs->CNDTR1) ) & ( ADCBUFFERSIZE - 1);
   // else
    //  ADCdmaindex = ( ADCBUFFERSIZE - DMA1->regs->CNDTR1 ) & ( ADCBUFFERSIZE - 1);
  if(NOTRIGGER == triggertype)
  {
    triggerindex = ADCCounter;
  }
}



/*
void fastfetch()
{
  if( applyoffsets )
  {
     digitalWrite(offset1Pin, HIGH);
     digitalWrite(offset2Pin, HIGH);
  } else {
     digitalWrite(offset1Pin, LOW);
     digitalWrite(offset2Pin, LOW);
  }

  // Configure ADC speed
  uint32_t adcinterval;
  uint32_t fullbuffertime;
  if(highspeed)
  {
    adc_set_prescaler(ADC_PRE_PCLK2_DIV_2); // 36 MHz ADC Clock
    adc_set_sample_rate(ADC1, ADC_SMPR_1_5); // Sample for 1.5 ADC clock cycles (14 total clocks)
    adc_set_sample_rate(ADC2, ADC_SMPR_1_5);
    adcinterval = 1024*14*2/72; // microseconds per 1024 samples
    fullbuffertime = ADCBUFFERSIZE*14*2/72; // usec
  } else {
    adc_set_prescaler(ADC_PRE_PCLK2_DIV_4); // 18 MHz ADC Clock
    adc_set_sample_rate(ADC1, ADC_SMPR_7_5); // Sample for 7.5 ADC clock cycles (20 total clocks)
    adc_set_sample_rate(ADC2, ADC_SMPR_7_5);
    adcinterval = 1024*20*4/72; // microseconds per 1024 samples
    fullbuffertime = ADCBUFFERSIZE*20*4/72; // usec
  }

  // Set up Direct Memory Access (DMA) for the ADC's.
  // Note that we can't set up separate transfers for the two ADC's. They have to be in Simultaneous mode, which combines them
  // into a 32-bit full word rather than the standard 16-bit half word for a single ADC. The 32 bit word holds two 12-bit values,
  // with ADC1 and ADC2 in bits 0-11 and 16-27, respectively.
  dma_init(DMA1);
  //dma_attach_interrupt(DMA1, DMA_CH1, func); // Don't think I want to call a function upon interrupt
  dma_setup_transfer( DMA1, DMA_CH1, &ADC1->regs->DR, DMA_SIZE_32BITS, ADCBuffer, DMA_SIZE_32BITS, (DMA_CCR_MINC | DMA_CCR_CIRC ) ); // (DMA_CIRC_MODE|DMA_MINC_MODE) ??
  dma_set_num_transfers(DMA1, DMA_CH1, ADCBUFFERSIZE/2);
  dma_enable(DMA1, DMA_CH1);
  if(bothchannels)
  {
    ADC1->regs->CR1 |= 6 << 16;      // Regular simultaneous mode. Required for ADC1 only. ADC2 will follow.
    ADC1->regs->SQR3 = PIN_MAP[analogInPin1].adc_channel;
    ADC2->regs->SQR3 = PIN_MAP[analogInPin2].adc_channel;
  } else {
    ADC1->regs->CR1 |= 7 << 16; // Fast interleaved mode
    ADC1->regs->SQR3 = PIN_MAP[analogInPin1].adc_channel;
    ADC2->regs->SQR3 = PIN_MAP[analogInPin1].adc_channel; // Same pin for single channel mode
  }
  ADC1->regs->CR2 |= ADC_CR2_DMA;     //enable ADC DMA transfer
  ADC1->regs->CR2 |= ADC_CR2_CONT;    //Set the ADC in Continuous Mode
  ADC2->regs->CR2 |= ADC_CR2_DMA;     //enable ADC DMA transfer
  ADC2->regs->CR2 |= ADC_CR2_CONT;    //Set the ADC in Continuous Mode

  bool lookfor;
  uint8_t trigger;
  bool done = false;
  uint32_t stopIndex;
  if(NOTRIGGER == triggertype)
  {
    trigger = 1;
    stopIndex = ADCBUFFERSIZE; // fill the whole buffer
    triggerindex = 0;
  } else {
    trigger = 5;
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

  ADCCounter = 0;

  uint32_t starttime;
  uint32_t currenttime;
  uint32_t ADCCounter_usec;
  uint32_t buffercycles;

  // Start conversion
  ADC1->regs->CR2 |= ADC_CR2_SWSTART;
  starttime = micros();

  // Can we dead reckon how many samples are in the buffer?
  ADCCounter = 0;
  buffercycles = 0;
  uint32_t num;
  while(true)
  {
    ADCCounter_usec = buffercycles*fullbuffertime + ADCCounter*fullbuffertime/ADCBUFFERSIZE;
    num = (micros()-starttime-ADCCounter_usec)*ADCBUFFERSIZE/fullbuffertime; // samples to be processed
    for(uint32_t i=0; i<num; i=i+2)
    {
      bool currentstate = (ADCBuffer[(ADCCounter+i)&(ADCBUFFERSIZE-1)]>=triggerlevel);   
      switch(trigger)
      {
        case 5: // trigger-disabled period ( to make sure we fill at least ADCBUFFERSIZE-waitDuration before trigger )
          if( ADCCounter+i > 256 )
            trigger--;
          break;
        case 4: // waiting
          if(currentstate == lookfor)
          {
            lookfor = !lookfor;
            trigger--;
          }
          break;
        case 3: // armed
          if(currentstate == lookfor)
          {
            trigger--;
            triggerindex = (ADCCounter+i)&(ADCBUFFERSIZE-1);
          }
          break;
        case 2: // triggered
          stopIndex = ( triggerindex + ADCBUFFERSIZE - 256 ) & (ADCBUFFERSIZE-1);
          trigger--;
          break;
        case 1:
            if( (ADCCounter+i)&(ADCBUFFERSIZE-1) == stopIndex )
            {
              dma_disable(DMA1,DMA_CH1); // We will have overshot by the time we get here
              ADC1->regs->CR2 &= ~ADC_CR2_CONT;
              ADC2->regs->CR2 &= ~ADC_CR2_CONT;
              done = true;
              trigger--;
            }
          break;
      }
    }
    ADCCounter += num&(ADCBUFFERSIZE-2); // even number
    if(ADCCounter > ADCBUFFERSIZE)
    {
      ADCCounter = ADCCounter&(ADCBUFFERSIZE-1);
      buffercycles++;
    }
    if(done)
      break;
    if(buffercycles > TRIGGERTIMEOUT)
      break;
  }
  ADCCounter = ( 2*(ADCBUFFERSIZE/2 + 1 - DMA1->regs->CNDTR1) ) & ( ADCBUFFERSIZE - 1);
  if(NOTRIGGER == triggertype)
  {
    triggerindex = ADCCounter;
  }
}
*/


void capture2()
{
  if( applyoffsets )
  {
     digitalWrite(offset1Pin, HIGH);
     digitalWrite(offset2Pin, HIGH);
  } else {
     digitalWrite(offset1Pin, LOW);
     digitalWrite(offset2Pin, LOW);
  }
  ADCCounter = 0;

  // Configure ADC speed
  //uint32_t adcinterval;
  //uint32_t fullbuffertime;
  if(highspeed)
  {
    adc_set_prescaler(ADC_PRE_PCLK2_DIV_2);  // 36 MHz ADC Clock
    adc_set_sample_rate(ADC1, ADC_SMPR_1_5); // Sample for 1.5 ADC clock cycles (14 total clocks for sample + conversion)
    adc_set_sample_rate(ADC2, ADC_SMPR_1_5);
    //adcinterval = 1024*14*2/72; // microseconds per 1024 samples
    //fullbuffertime = ADCBUFFERSIZE*14*2/72; // usec
  } else {
    adc_set_prescaler(ADC_PRE_PCLK2_DIV_4);  //18 MHz ADC Clock
    adc_set_sample_rate(ADC1, ADC_SMPR_7_5); // Sample for 7.5 ADC clock cycles (20 total clocks for sample + conversion)
    adc_set_sample_rate(ADC2, ADC_SMPR_7_5);
    //adcinterval = 1024*20*4/72; // microseconds per 1024 samples
    //fullbuffertime = ADCBUFFERSIZE*20*4/72; // usec
  }

  // Set up ADC's in Simultaneous mode, which combines them into a 32-bit full word rather than the 
  // standard 16-bit half word for a single ADC. The 32 bit word holds two 12-bit values, with ADC1
  // and ADC2 in bits 0-11 and 16-27, respectively.
  if(bothchannels)
  {
    ADC1->regs->CR1 |= 6 << 16; // Regular simultaneous mode. Required for ADC1 only. ADC2 will follow.
    ADC1->regs->SQR3 = PIN_MAP[analogInPin1].adc_channel;
    ADC2->regs->SQR3 = PIN_MAP[analogInPin2].adc_channel;
  } else {
    ADC1->regs->CR1 |= 7 << 16; // Fast interleaved mode
    ADC1->regs->SQR3 = PIN_MAP[analogInPin1].adc_channel;
    ADC2->regs->SQR3 = PIN_MAP[analogInPin1].adc_channel; // Same pin for single channel mode
  }
  ADC1->regs->CR2 |= ADC_CR2_CONT;    //Set the ADC in Continuous Mode
  ADC1->regs->CR2 |= ADC_CR2_DMA;     //Needs to be in DMA mode for dual mode to work
  ADC2->regs->CR2 |= ADC_CR2_CONT;    //Set the ADC in Continuous Mode
  ADC2->regs->CR2 |= ADC_CR2_DMA;     //Needs to be in DMA mode for dual mode to work

  bool lookfor;
  uint8_t trigger;
  bool done = false;
  uint32_t stopIndex;
  if(NOTRIGGER == triggertype)
  {
    trigger = 1;
    stopIndex = ADCCounter; // fill the whole buffer
    triggerindex = 0;
  } else {
    trigger = 5;
    stopIndex = ADCBUFFERSIZE/2+1; // will never reach this, but will update when triggered
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

  uint32_t *ADCBuffer32 = (uint32_t*)ADCBuffer; // We will be writing 32-bit words with both ADC's combined
  uint32_t buffercycles = 0;
  bool currentstate;

  // Start conversion
  ADCCounter = 0;
  nvic_globalirq_disable();
  ADC1->regs->CR2 |= ADC_CR2_SWSTART;
  
  while(true)
  {
    while (!(ADC1->regs->SR & ADC_SR_EOC)) // Wait for next sample
          ;
    
    ADCBuffer32[ADCCounter] = (ADC1->regs->DR); // & ADC_DR_DATA; // Store sample (32 bits, contains both ADC1 and ADC2); Reading ADC_DR clears the ADC_SR EOC bit
    currentstate = (ADCBuffer[ADCCounter<<1]>=triggerlevel);   // We trigger based on adc1 only
    ADCCounter = (ADCCounter+1) & (ADCBUFFERSIZE/2-1);
    
    switch(trigger)
    {
      case 5: // trigger-disabled period ( to make sure we fill at least ADCBUFFERSIZE-waitDuration before trigger )
        if( ADCCounter > 256 )
          trigger--;
        break;
      case 4: // waiting
        if(currentstate == lookfor)
        {
          lookfor = !lookfor;
          trigger--;
        }
        break;
      case 3: // armed
        if(currentstate == lookfor)
        {
          trigger--;
          triggerindex = ADCCounter;
        }
        break;
      case 2: // triggered
        stopIndex = ( triggerindex + ADCBUFFERSIZE/2 - 256 ) & (ADCBUFFERSIZE/2-1);
        trigger--;
        break;
      case 1:
          if( ADCCounter == stopIndex )
          {
            // Take the ADC's out of continuous conversion mode
            ADC1->regs->CR2 &= ~ADC_CR2_CONT;
            ADC2->regs->CR2 &= ~ADC_CR2_CONT;
            done = true;
            trigger--;
          }
        break;
    }
    
    if(0==ADCCounter)
      buffercycles++;
    if(done)
      break;
    if(buffercycles > TRIGGERTIMEOUT)
      break;
  }
  nvic_globalirq_enable();

  ADCCounter = 2*ADCCounter; // This function was looking at 32 bit elements whereas the loop function uses 16 bit
  triggerindex = 2*triggerindex;
  if(NOTRIGGER == triggertype)
  {
    triggerindex = ADCCounter;
  }
}



void setup() {
  Serial.begin(115200);
  //pinMode(LED_BUILTIN, OUTPUT);
  pinMode(offset1Pin,OUTPUT);
  pinMode(offset2Pin,OUTPUT);
  if( applyoffsets )
  {
     digitalWrite(offset1Pin, HIGH);
     digitalWrite(offset2Pin, HIGH);
  } else {
     digitalWrite(offset1Pin, LOW);
     digitalWrite(offset2Pin, LOW);
  }
  pinMode(analogInPin1, INPUT_ANALOG);
  pinMode(analogInPin2, INPUT_ANALOG);
  memset( (void *)ADCBuffer, 0, sizeof(ADCBuffer) );
  adc_calibrate(ADC1);
  adc_calibrate(ADC2);
  //delay(3000);
}

void fakedata() {
  for(int i=0; i<ADCBUFFERSIZE; i++)
    ADCBuffer[i] = i&(ADCBUFFERSIZE-1);
  triggerindex = 0;
  ADCCounter = 0;
  #warning fake data for test purposes
  return;
}

void loop() {
  // put your main code here, to run repeatedly:
  //fetch();
  //capture();
  //fastfetch();
  capture2();
  //fakedata();
  
  //Serial.print("DATA ");
  Serial.print('D');
  Serial.print('A');
  Serial.print('T');
  Serial.print('A');
  if(bothchannels)
  {
    Serial.write(0x02); // two channels
  } else {
    Serial.write(0x01); // one channel
  }
  Serial.write(uint8_t(0x00)); // simultaneous
  Serial.write(uint8_t( (ADCBUFFERSIZE) & 255));
  Serial.write(uint8_t((ADCBUFFERSIZE) >> 8) & 255 );
  int adjustedtriggerindex = triggerindex - ADCCounter;
  if(0>adjustedtriggerindex)
    adjustedtriggerindex += ADCBUFFERSIZE;
  Serial.write( adjustedtriggerindex & 255);
  Serial.write( (adjustedtriggerindex >> 8) & 255 );
  
  for(int i=0;i<ADCBUFFERSIZE;i++)
  {
    Serial.write((uint8_t)((ADCBuffer[(ADCCounter + i) & (ADCBUFFERSIZE-1)]) >> 4));
    delayMicroseconds(10); // I lose bytes if I try to send too quickly
    //Serial.print(',');
  }
  //Serial.println("END");
  Serial.print('E');
  Serial.print('N');
  Serial.print('D');
  Serial.print('\n');
  delay(100);

  int keepwaiting = 0;
  
  while(keepwaiting < 10000)
  {
    int incomingByte = 0; // for incoming serial data
    incomingByte = Serial.read();
    int i = 0;
    while(0 <= incomingByte)
    {
      serialbuffer[i] = incomingByte;
      i += 1;
      if (i==SERIALMSGSIZE)
        break;
      incomingByte = Serial.read();
    }
    //delay(100); // Give everything a chance to catch up
    // Parse command
    switch(serialbuffer[0])
    {
      case 'C':
        switch(serialbuffer[1])
        {
          case 'H': // CH -- channel enable
            bothchannels = serialbuffer[2];
            break;
        }
        break;
      case 'R':
        switch(serialbuffer[1])
        {
          case 'N': // RN -- run scope
            keepwaiting = 100000;
            break;
        }
        break;
      case 'T':
        switch(serialbuffer[1])
        {
          case 'R': // TR -- trigger settings
            //if( '\n' == serialbuffer[8] )
            //{
              triggertype = serialbuffer[2];
              triggerlevel = (serialbuffer[3]) << 4; // Bit shifting because GUI only uses 8-bit values
              //delay(1000);
            //}
            break;
        }
        break;
      case 'S':
        switch(serialbuffer[1])
        {
          case 'T': // ST -- settings
            applyoffsets = serialbuffer[2];
            if( serialbuffer[3])
            {
              bothchannels = true;
            } else {
              bothchannels = false;
            }
            if( serialbuffer[4])
            {
              // low speed
              highspeed = true;
            } else {
              // high speed
              highspeed = false;
            }
            break;
        }
        break;
    }
    delay(10);
    keepwaiting++;
  }

}
