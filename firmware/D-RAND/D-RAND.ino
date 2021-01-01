/*
  Copyright (C) 2020  Ben Reeves

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <https://www.gnu.org/licenses/>.

  ==============================================================================

  Project:     JoyfulNoise D-RAND
  File:        D-RAND.ino
  Description: 4HP Voltage-Controlled Random Generator

  The JoyfulNoise D-RAND is an 8-bit random voltage generator. On every trigger
  received, a new random voltage is generated between some Low and High bounds.
  These Low and High bounds range from -5V to 5V and are voltage-controllable.
  
  Many thanks to Ross Fish at Moffenzeef Modular for designing the Deviant
  module which inspired this.

  ----------
  PARAMETERS
  ----------

  PARAM 1 - Low
    Sets the lower bound of the random output.

  PARAM 2 - High
    Sets the upper bound of the random output.

  PARAM 3 - Bias
    Biases the output towards either end of the configured output range.
      - 12 o'clock = voltages evenly distributed between Low and High
      - All the way left = always outputs Low
      - All the way right = always outputs High
      

  GATE/TRG - Trigger New Value

  -----
  JNTUB
  -----

  D-RAND is based on the JoyfulNoise Tiny Utility Board (JNTUB), a
  reprogrammable 4HP module with a standard set of inputs and one 8-bit
  analog output.

 */


void setup()
{
}

void loop()
{
}
