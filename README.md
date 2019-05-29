Novatel GPS Driver
==================
  
This project provides a cross-platform interface for the Novatel OEM4 and OEMV series of GPS receivers.  The Novatel SPAN system is also supported. 

The Novatel driver is written as a standlone library which depends on [Boost](http://http://www.boost.org) and a simple cross-platform serial port library [serial port library](https://github.com/wjwwood/serial).  It uses Visual Studio 2017 for the build system.  Example programs are provided that demonstrate the basic functionality of the library.

I forked it from [GAVLab/novatel](https://github.com/GAVLab/novatel) , fixed some bugs and made it suit for Windows.

# Installation 
Open the .sln file and complie.

# Operation

## Callback Definitions

## Supported Messages

# License

The BSD License

Copyright (c) 2013 David Hodo - Integrated Solutions for Systems / Auburn University

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

# Authors

David Hodo <david.hodo@gmail.com>
xxzl0130 <zaxs0130@gmail.com>

Portions of this library are based on previous code by William Travis and Scott Martin.  Thanks to William Woodall for the serial library.
