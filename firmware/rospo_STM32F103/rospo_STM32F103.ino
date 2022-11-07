
// BUFFER SIZE MUST BE A POWER OF TWO
#define analogInPin PA5
#define ADCBUFFERSIZE 4096

uint16_t ADCCounter=0;
uint16_t ADCBuffer[ADCBUFFERSIZE];
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

void fetch() {  
//  const adc_dev *dev = PIN_MAP[analogInPin].adc_device;
  int channel = PIN_MAP[analogInPin].adc_channel;
  adc_set_sample_rate(ADC1, ADC_SMPR_1_5); // ADC_SMPR_13_5, ADC_SMPR_1_5
  adc_set_sample_rate(ADC2, ADC_SMPR_1_5); // ADC_SMPR_13_5, ADC_SMPR_1_5
  adc_set_prescaler(ADC_PRE_PCLK2_DIV_2); // 36 MHz ADC Clock
  adc_set_reg_seqlen(ADC1, 1);
  adc_set_reg_seqlen(ADC2, 1);
  ADC1->regs->SQR3 = channel;
  ADC2->regs->SQR3 = channel;

  ADC1->regs->CR1 |= 0x70000; // ADC_CR1_FASTINT;
  ADC1->regs->CR2 |= ADC_CR2_CONT | ADC_CR2_SWSTART;
  ADC2->regs->CR2 |= ADC_CR2_CONT | ADC_CR2_SWSTART;
  ADC2->regs->CR1 |= 0x70000; // ADC_CR1_FASTINT;
  nvic_globalirq_disable();
//  uint32_t t = micros();
  // .584 uS
  for (int j = 0; j < ADCBUFFERSIZE ; j+=2 )
  {
    while (!(ADC1->regs->SR & ADC_SR_EOC))
        ;
    ADCBuffer[j] = ADC1->regs->DR & ADC_DR_DATA;
    while (!(ADC2->regs->SR & ADC_SR_EOC))
        ;
    ADCBuffer[j+1] = ADC2->regs->DR & ADC_DR_DATA;
  }
  nvic_globalirq_enable();
  //t = micros()-t;
  //Serial.println(String(t));
}


void setup() {
  Serial.begin(115200);
  pinMode(PA2,OUTPUT);
  digitalWrite(PA2,0);
  delay(3000);
}

uint32_t runNumber = 0;

void loop() {
  // put your main code here, to run repeatedly:
  fetch();
  
  //Serial.print("DATA ");
  Serial.print('D');
  Serial.print('A');
  Serial.print('T');
  Serial.print('A');
  Serial.write(0x01); // one channel
  Serial.write(0x01); // first channel
  Serial.write( (uint8_t)(ADCBUFFERSIZE & 255) );
  Serial.write( (uint8_t)((ADCBUFFERSIZE >> 8) & 255) );
  
  for(int i=0;i<ADCBUFFERSIZE;i++)
  {
    Serial.write((uint8_t)((ADCBuffer[(ADCCounter + i) & (ADCBUFFERSIZE-1)]) >> 4));
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
    delay(200); // Give everything a chance to catch up
    // Parse command
    switch(serialbuffer[0])
    {
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
        }
    }
    delay(50);
  }
}
