/*
  Copyright (C) 2021  Ben Reeves

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
  File:        D-RAND.h
  Description: 4HP Voltage-Controlled Random Generator

 */

#ifndef D_RAND_H
#define D_RAND_H

// JoyfulNoise Tiny Utility Board Library
#include <JNTUB.h>

class D_RandModule : public JNTUB::Module {
public:
  D_RandModule();
  void setup() override;
  uint8_t loop(const JNTUB::Environment &env) override;

private:
  // Implementation details hidden behind opaque pointer.
  class D_Rand *mImpl;
};

#endif //D_RAND_H
