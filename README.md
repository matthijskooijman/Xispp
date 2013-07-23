Xispp: Xenophobia Intermittent Serial Peer-to-peer Protocol library for Arduino
===============================================================================
Xispp is a serial protocol that can be used to connect Arduinos (or
other microcontrollers) together and let them talk, without having to
know in advance which devices connect to which other devices.

This library implements the protocol for the Arduino, but is probably
also usuable for other AVR microcontrollers outside of the Arduino
environment.

What does Xispp stand for?
--------------------------
**X**enophobia: The name of the live roleplay event this library was
originally developed for.

**I**ntermittent: This refers to the fact that idle peers can intermittently
poll each line, without having to continously monitor the line. It also
refers to the fact that the serial connection between peers can be
disconnected or reconnected to any other peer at any time.

**S**erial: It is a serial protocol, obviously.

**P**eer-to-peer: The protocol connects two equal peers, without the
need of marking them as master/slave. Note that this does not mean that
the protocol forms any kind of global network (like the peer-to-peer
filesharing networks). The protocol only offers communication between
directly connected peers.

**P**rotocol: Just that :-)

How is this library licensed?
-----------------------------
This library is licensed under the MIT license:

Copyright (c) 2013 Matthijs Kooijman <<matthijs@stdin.nl>>

Permission is hereby granted, free of charge, to any person obtaining a
copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be included
in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
