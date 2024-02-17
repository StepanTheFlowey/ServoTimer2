#include "ServoTimer2.hpp"

#include <Arduino.h>

#define FRAME_SYNC_INDEX  0        //frame sync delay is the first entry in the channel array
#define FRAME_SYNC_PERIOD 12750    //total frame duration in microseconds
#define FRAME_SYNC_DELAY  ((FRAME_SYNC_PERIOD - (NBR_CHANNELS * DEFAULT_PULSE_WIDTH)) / 128) //number of iterations of the ISR to get the desired frame rate
#define DELAY_ADJUST      8        //number of microseconds of calculation overhead to be subtracted from pulse timings

static servo_t servos[NBR_CHANNELS + 1]; //static array holding servo data for all channels
static volatile uint8_t Channel = 0;     //counter holding the channel being pulsed
static volatile uint8_t ISRCount = 0;    //iteration counter used in the interrupt routines
static uint8_t ChannelCount = 0;         //counter holding the number of attached channels
static bool isStarted = false;           //flag to indicate if the ISR has been initialised

ISR(TIMER2_OVF_vect) {
  ++ISRCount; //increment the overlflow counter

  if(ISRCount == servos[Channel].counter) { //are we on the final iteration for this channel?
    TCNT2 = servos[Channel].remainder;      //yes, set count for overflow after remainder ticks
  }
  else if(ISRCount > servos[Channel].counter) {
    //we have finished timing the channel so pulse it low and move on
    if(servos[Channel].pin.isActive) {            //check if activated
      digitalWrite(servos[Channel].pin.nbr, LOW); //pulse this channel low if active
    }

    ++Channel;    //increment to the next channel
    ISRCount = 0; //reset the isr iteration counter
    TCNT2 = 0;    //reset the clock counter register

    if(Channel > NBR_CHANNELS) {
      Channel = 0; //all done so start over
    }
    else if(Channel != FRAME_SYNC_INDEX) { //check if we need to pulse this channel
      if(servos[Channel].pin.isActive) {             //check if activated
        digitalWrite(servos[Channel].pin.nbr, HIGH); //its an active channel, so pulse it high
      }
    }
  }
}

static void writeChan(uint8_t chan, uint16_t pulsewidth) {
  //calculate and store the values for the given channel
  if((chan > 0) && (chan <= NBR_CHANNELS)) { //ensure channel is valid
    if(pulsewidth < MIN_PULSE_WIDTH) {       //ensure pulse width is valid
      pulsewidth = MIN_PULSE_WIDTH;
    }
    else if(pulsewidth > MAX_PULSE_WIDTH) {
      pulsewidth = MAX_PULSE_WIDTH;
    }

    //subtract the time it takes to process the start and end pulses (mostly from digitalWrite)
    pulsewidth -= DELAY_ADJUST;
    servos[chan].counter = pulsewidth / 128;
    servos[chan].remainder = 255 - (2 * (pulsewidth - (servos[chan].counter * 128)));  //the number of 0.5us ticks for timer overflow
  }
}

static void initISR() {
  //channels start from 1
  for(uint8_t i = 1; i <= NBR_CHANNELS; i++) {
    writeChan(i, DEFAULT_PULSE_WIDTH);  //store default values
  }
  servos[FRAME_SYNC_INDEX].counter = FRAME_SYNC_DELAY;   //store the frame sync period

  //setup timer 2
  TIMSK2 = 0;          //disable interrupts
  TCCR2A = 0;          //normal counting mode
  TCCR2B = bit(CS21);  //set prescaler of 8
  TCNT2 = 0;           //clear the timer2 count
  TIMSK2 = bit(TOIE2); //enable the overflow interrupt

  isStarted = true;    //flag to indicate this initialisation code has been executed
}

ServoTimer2::ServoTimer2() {
  if(ChannelCount < NBR_CHANNELS) {
    chanIndex = ++ChannelCount;  //assign a channel number to this instance
  }
  else {
    chanIndex = 0;  //too many channels, assigning 0 inhibits this instance from functioning
  }
}

uint8_t ServoTimer2::attach(const uint8_t pin) {
  if(!isStarted) {
    initISR();
  }

  if(chanIndex > 0) {
    pinMode(pin, OUTPUT);  //set servo pin to output
    servos[chanIndex].pin.nbr = pin;
    servos[chanIndex].pin.isActive = true;
  }

  return chanIndex;
}

void ServoTimer2::detach() {
  servos[chanIndex].pin.isActive = false;
}

uint16_t ServoTimer2::readMicroseconds() const {
  if(chanIndex > 0) {
    return servos[chanIndex].counter * 128 + ((255 - servos[chanIndex].remainder) / 2) + DELAY_ADJUST;
  }

  return 0;
}

void ServoTimer2::writeMicroseconds(const uint16_t pulsewidth) {
  writeChan(chanIndex, pulsewidth); //call the static function to store the data for this servo
}

bool ServoTimer2::attached() const {
  return servos[chanIndex].pin.isActive;
}
