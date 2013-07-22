#include <Arduino.h>
#include "Channel.h"
#include "Veste.h"

/* Interval between two beacons, in ms */
#define BEACON_INTERVAL 100

/* How much time after the last beacon received that the connection will
 * be assumed broken, in ms. */
#define BEACON_TIMEOUT (4 * TICKS_TO_US(CYCLE_INTERVAL) / 1000)

#ifndef lengthof
#define lengthof(x) (sizeof(x)/sizeof(*x))
#endif

// Zero init
uint8_t Channel::num_channels = 0;
Channel *Channel::channels[MAX_CHANNELS];

Channel::Channel(uint8_t data_pin, uint8_t request_pin) :
  ows(data_pin, request_pin)
{
  this->ows.setCallbacks(this);
};

bool Channel::begin(int baud_rate) {
  // See if we still have room in the channels list
  if (Channel::num_channels >= lengthof(Channel::channels))
    return false;

  Channel::channels[Channel::num_channels++] = this;

  this->last_beacon_sent = this->time();
  this->connected = false;

  return this->ows.begin(baud_rate);
}

uint16_t Channel::time() {
  uint16_t t = millis();
  if (t == 0)
    --t;
  return t;
}

void Channel::handlePendingAll()
{
  for (int i=0; i < Channel::num_channels; ++i)
    Channel::channels[i]->handlePending();
}

void Channel::handlePending()
{
  this->ows.handle_pending();

  uint16_t t = millis();
  if (this->last_beacon_sent && (uint16_t)(t - this->last_beacon_sent) > BEACON_INTERVAL) {
    // It has been a while since we last sent a beacon, queue a new one
    this->queueBeacon();
  }

  if (this->connected && (uint16_t)(t - this->last_beacon_received) > BEACON_TIMEOUT) {
    // We haven't received a beacon for a while, our peer probably
    // disconnected
    this->connected = false;
    digitalWrite(13, LOW);
  }
}

void Channel::queueBeacon()
{
    this->last_beacon_sent = 0;
    this->ows.startPacket();
    this->ows.write(0x55);
    this->ows.write(this->state);
    this->ows.finishPacket();
}

void Channel::setState(uint8_t state) {
  if (state != this->state) {
    this->state = state;
    this->queueBeacon();
  }
}

bool Channel::becomeBusy(OneWireSymmetrical* c) {
  for (size_t i=0; i < Channel::num_channels; ++i)
    if (Channel::channels[i] != this)
      Channel::channels[i]->ows.cancel_request();
  // TODO: Iets met INACTIVE?
  return true;
}

void Channel::txStart(OneWireSymmetrical *c) {
}

void Channel::txDone(OneWireSymmetrical *c) {
  this->last_beacon_sent = this->time();
  this->beacon_ok = true;
}

void Channel::rxStart(OneWireSymmetrical *c) {
}

void Channel::rxDone(OneWireSymmetrical *c, bool ok) {
  if (ok && c->rec_len >= 2) {
    switch (c->rec_buf[0]) {
      case 0x55:
        // Beacon
        this->last_beacon_received = this->time();
        this->connected = true;
        this->peer_state = c->rec_buf[1];
        digitalWrite(13, HIGH);
        break;
    }
  } else {
    Serial.println("RXFAIL!");
  }
  // TODO: Move into OWS
  c->rec_len = 0;
}

/* vim: set sw=2 sts=2 expandtab filetype=cpp: */
