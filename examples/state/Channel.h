/*
Copyright (c) 2013 Matthijs Kooijman <matthijs@stdin.nl>

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
*/

#ifndef __CHANNEL_H
#define __CHANNEL_H

#include "OneWireSymmetrical.h"

#define MAX_CHANNELS 8

class Channel : public OneWireSymmetrical::Callbacks {
public:
  OneWireSymmetrical ows;

  Channel(uint8_t data_pin, uint8_t request_pin);
  bool begin(int baud_rate);

  static Channel *channels[MAX_CHANNELS];
  static uint8_t num_channels;

  /**
   * Time of last beacon received as returned by millis().
   *
   * This is only the lower 16 bits, since the beacon timeout should be
   * much less than 65536 ms.
   *
   * Only valid when connected.
   */
  uint16_t last_beacon_received;

  uint16_t last_beacon_sent;

  bool connected;

  static void handlePendingAll();
  void handlePending();
  void queueBeacon();
  void setState(uint8_t state);

  uint8_t state;
  uint8_t peer_state;

  bool beacon_ok;

  ////////////////////////////////////////////
  // Callbacks for OneWireSymmetrical
  ////////////////////////////////////////////

  /**
   * Called when a channel sees an ack or a request and wants to become busy
   * for a while (e.g., handlePending might not return for a while and/or
   * a SoftwareSerial will be set to listen).
   * If this happens, we'll have to cancel all the requests on the other
   * channels, since we won't be able to see any acks coming in on them.
   */
  bool becomeBusy(OneWireSymmetrical* c);
  void txStart(OneWireSymmetrical *c);
  void txDone(OneWireSymmetrical *c);
  void rxStart(OneWireSymmetrical *c);
  void rxDone(OneWireSymmetrical *c, bool ok);
  uint16_t time();
};

#endif // __CHANNEL_H

/* vim: set sw=2 sts=2 expandtab filetype=cpp: */
