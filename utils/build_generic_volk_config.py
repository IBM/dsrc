#!/usr/bin/env python
# 
# Copyright 2018 IBM
# 
# This is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
# 
# This software is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this software; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.
# 

from __future__ import print_function
import sys

if (len(sys.argv) - 1) != 2:
    print('Usage: ' + sys.argv[0] + ' source_volk_config dest_volk_config')
    sys.exit(1)

f = open(sys.argv[1], "r")
lines = f.readlines()
f.close()

for i in range(len(lines)):
    line = lines[i]
    if line.startswith("#"):
        continue    
    fields = line.split(' ')
    lines[i] = fields[0] + ' generic generic\n'  # I needed to replicate 'generic' twice to make it work.

f = open(sys.argv[2], "w")
f.write("".join(lines))
f.close()
