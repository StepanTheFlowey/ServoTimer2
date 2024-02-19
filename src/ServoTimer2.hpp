/*
  ServoTimer2.hpp - Interrupt driven Servo library for Arduino using Timer2 - Version 0.2
  Copyright (c) 2008 Michael Margolis.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

  This library uses Timer2 to drive up to 8 servos using interrupts so no refresh activity is required from within the sketch.
  The usage and method naming is similar to the Arduino software servo library http://www.arduino.cc/playground/ComponentLib/Servo
  except that pulse widths can only be in microseconds.

  A servo is activated by creating an instance of the Servo class passing the desired pin to the attach() method.
  The servo is pulsed in the background to the value most recently written using the write() method

  Note that analogWrite of PWM on pins 3 and 11 is disabled when the first servo is attached

  * Updated for Arduino 1.x by Nick Bontrager 2013
*/
#pragma once

#include <Arduino.h>

#define NBR_CHANNELS            4  //the maximum number of channels
#define MIN_PULSE_WIDTH       500  //the shortest pulse sent to a servo
#define MAX_PULSE_WIDTH      2000  //the longest pulse sent to a servo
#define DEFAULT_PULSE_WIDTH  1500  //default pulse width when servo is attached

struct servo_t {
  struct {
    uint8_t nbr : 7;       //a pin number from 0 to 127
    uint8_t isActive : 1;  //false if this channel not enabled, pin only pulsed if true
  } pin;

  uint8_t counter;
  uint8_t remainder;
};

class ServoTimer2 {
public:

  ServoTimer2();

  ~ServoTimer2() = default;

  //attach the given pin to the next free channel, sets pinMode, returns channel number or 0 if failure
  //the attached servo is pulsed with the current pulse width value, (see the write method)
  uint8_t attach(const uint8_t pin);
  void detach();

  //returns/stores current pulse width in microseconds for this servo
  uint16_t readMicroseconds() const;
  void writeMicroseconds(const uint16_t);

  //return true if this servo is attached
  bool attached() const;
protected:

  uint8_t chanIndex = 0;  // index into the channel data for this servo
};
