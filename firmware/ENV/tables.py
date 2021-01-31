# Copyright (C) 2021  Ben Reeves
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
#
# ==============================================================================
#
# Project:     JoyfulNoise ENV
# File:        tables.py
# Description: Code for generating envelope curves.


import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import scipy as sp


def as_c_decl(a, name, signed=False, bits=8, indent=2):
  ret = f'const {"u" if not signed else ""}int{bits}_t {name}[{len(a)}] PROGMEM = {{\n'
  for i in range(len(a)):
    ret += f'{" "*indent}{int(a[i])},  // {i}\n'
  ret += '};'
  return ret


x = np.arange(256)

linear = x

quadratic = np.floor(256 * (x / 256) ** 2)

exponential = np.floor(256 ** (x / 256) - 1)

m = 1536
inverse = np.clip(np.floor(m / (256 - x) - m/256), 0, 255)

square = np.zeros(256)
square[255] = 255

plt.plot(x, linear)
plt.plot(x, quadratic)
plt.plot(x, exponential)
plt.plot(x, inverse)
plt.plot(x, square)
plt.show()

print(as_c_decl(linear, 'CURVE_LINEAR'))
print()
print(as_c_decl(quadratic, 'CURVE_QUADRATIC'))
print()
print(as_c_decl(quadratic, 'CURVE_EXPONENTIAL'))
print()
print(as_c_decl(quadratic, 'CURVE_INVERSE'))
print()
print(as_c_decl(square, 'CURVE_SQUARE'))
print()

