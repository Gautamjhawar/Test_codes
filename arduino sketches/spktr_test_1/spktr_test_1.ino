#include <PinChangeInt.h>
#include <eRCaGuy_Timer2_Counter.h>

//project's contants
#define RECEIVER_AUX1_IN_PIN 2 // we could choose any pin
#define BUFFER_SIZE 250

//project's switches
#define ENABLE_SERIAL_OUTPUT

//DECLARE_RECEIVER_SIGNAL(receiver_aux1_handler);
volatile unsigned long risingTime; //time in micros() on last pin rise
volatile unsigned long fallingTime; //time in micros() on last pin fall
volatile unsigned long pwmValue; //PWM value expected from 1000 us to 2000 us
         unsigned long prevPwmValue; //previous value of pwmValue

unsigned long readingsBuffer[BUFFER_SIZE];
unsigned long numReadings = 0;

unsigned long getPwmValue() {
  uint8_t pushedSREG = SREG;            // save interrupt flag
  cli();                                // disable interrupts
  unsigned long pwmCopy = pwmValue;    // access the shared data
  SREG = pushedSREG;                    // restore the interrupt flag

  return pwmCopy;
}

bool hasChanged() {
  unsigned long currentValue = getPwmValue();
  bool changed = (currentValue != prevPwmValue);
  prevPwmValue = currentValue;
  return changed;
}

void onPinChanged() {
  if(PCintPort::pinState == HIGH)
  {
    //this is a rising edge
    //record time
    risingTime = timer2.get_count(); //count units of 0.5us each
  }
  else
  {
    //this is a falling edge
    //read elapsed time since RISING
    fallingTime = timer2.get_count(); //count units of 0.5us each;
    pwmValue = (fallingTime - risingTime)/2; //to a precision of 1us
  }
}

void setup() {
  risingTime = 0;
  fallingTime = 0;
  pwmValue = 0;

  //setup interrupts
  pinMode(RECEIVER_AUX1_IN_PIN, INPUT);
  digitalWrite(RECEIVER_AUX1_IN_PIN, HIGH); //use the internal pullup resistor
  
  //prepare first interruption on pin CHANGE
  PCintPort::attachInterrupt(RECEIVER_AUX1_IN_PIN, &onPinChanged, CHANGE);
  
  //configure Timer2
  timer2.setup(); //this MUST be done before the other Timer2_Counter functions work; Note: since this messes up PWM outputs on pins 3 & 11, as well as 
                  //interferes with the tone() library (http://.cc/en/reference/tone), you can always revert Timer2 back to normal by calling 
                  //timer2.unsetup()
  
  #ifdef ENABLE_SERIAL_OUTPUT
    Serial.begin(115200);
    Serial.println("ready");
  #endif
}

void loop() {

  //detect when the receiver AUX1 value has changed
  if (hasChanged())
  {
    unsigned long pwmValue = getPwmValue();

    //add value to reading buffer
    readingsBuffer[numReadings] = pwmValue;
    numReadings++;

    if (numReadings == BUFFER_SIZE)
    {
      //flush
      #ifdef ENABLE_SERIAL_OUTPUT
        char pwnStr[10];
        unsigned long sum = 0;
        for(unsigned long i=0; i<BUFFER_SIZE; i++)
        {
          pwmValue = readingsBuffer[i];
          sum += pwmValue;
          sprintf(pwnStr, "%04d", pwmValue);
          Serial.println(pwnStr);
        }
        double avg = sum/double(BUFFER_SIZE);
        //Serial.print("sum=");
        //Serial.println(sum);
        //Serial.print("avg=");
        //Serial.println(avg);
      #endif

      numReadings = 0;

      //ignore the next change since serial printing may have impacted the timing...
      while( !hasChanged() )
      {
      }
      
    }
  }
}
