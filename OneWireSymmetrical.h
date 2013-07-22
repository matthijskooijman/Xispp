#ifndef __ONE_WIRE_SYMMETRICAL_H
#define __ONE_WIRE_SYMMETRICAL_H

#include <SoftwareSerial.h>
#include "Crc.h"

/* Last part of a frame that should be left empty to account for small
 * synchronisation differences.
 * Known differences:
 *  - The phase of the tick interrupt is not synchronized when a beacon
 *    is received, so the tick interrupts of the two devices can be up
 *    to 1/2 a tick apart.
 *  - Delays between receiving the first bit of beacon packet and processing it because
 *    interrupts are disabled. These should be at most one bit-time, so
 *    1/10 tick at 9600 baud.
 *  - Delays between putting a packet in the serial buffer and
 *    processing it in receive(), because the mainloop is busy with
 *    other things. These should be at most a few hundred CPU cycles, so
 *    only a few half-dozen of us. TODO: is this really true?
 *
 * In the above, the 1/2 tick is the largest difference, so using a
 * guard interval of 1 tick should be sufficient.
 */
#define GUARD_INTERVAL 1

/* The baudrate to use for channels (bits/s) */
#define BAUD_RATE 9600

/* The time needed to transmit a single byte (== start + 8 data + stop
 * bit). (10 bits) * (1,000,000 us/s) / (9600 bits/s) == us */
#define BYTE_TIME (10L * 1000 * 1000 / BAUD_RATE)

/* Arduino core configures the clock divider to 64 */
#define T0_CLOCK (F_CPU / 64)
/* How many clock cycles in one tick? Configured by Arduino core. */
#define CYCLES_PER_TICK 256
/* Number of "ticks" (timer interrupts) per second. Timer wraps at 256. */
#define TICKS_PER_SECOND (T0_CLOCK / CYCLES_PER_TICK)

#define US_TO_HZ(u) (1000l*1000/u)
#define HZ_TO_US(f) (1000l*1000/f)

/* How many ticks in the given amount of microseonds, properly rounded. */
#define US_TO_TICKS(u) ((u + HZ_TO_US(TICKS_PER_SECOND) / 2) / HZ_TO_US(TICKS_PER_SECOND))
#define TICKS_TO_US(t) (t * HZ_TO_US(TICKS_PER_SECOND))

/* Interval between two subsequent frame starts, in ticks */
#define FRAME_INTERVAL 10

/* The number of frames per cycle. Must be a multiple of 2, but not a
 * multiple of 4 (e.g., it must be the double of an odd value). */
#define FRAMES_PER_CYCLE 10

/* Interval between two subsequent cycle starts, in ticks */
#define CYCLE_INTERVAL (FRAMES_PER_CYCLE * FRAME_INTERVAL)

/* How much time after the last beacon received that the connection will
 * be assumed broken, in ms. */
#define BEACON_TIMEOUT (4 * TICKS_TO_US(CYCLE_INTERVAL) / 1000)

/* Minimum and maximum time between beacons when disconnected, in ticks. */
#define RANDOM_BEACON_INTERVAL_MIN (CYCLE_INTERVAL * 8 / 10)
#define RANDOM_BEACON_INTERVAL_MAX (CYCLE_INTERVAL * 12 / 10)

static_assert(FRAMES_PER_CYCLE % 2 == 0 && FRAMES_PER_CYCLE % 4 != 0, "Invalid value for FRAMES_PER_CYCLE");

#define MAX_PACKET 10

/**
 * An external channel, implemented using a common serial protocol on a
 * IO pin.
 */
class OneWireSymmetrical {
public:
  class Callbacks {
  public:
    virtual bool becomeBusy(OneWireSymmetrical *c) = 0;
    virtual void txStart(OneWireSymmetrical *c) = 0;
    virtual void txDone(OneWireSymmetrical *c) = 0;
    virtual void rxStart(OneWireSymmetrical *c) = 0;
    virtual void rxDone(OneWireSymmetrical *c, bool ok) = 0;
  };

  OneWireSymmetrical(uint8_t data_pin, uint8_t request_pin);
  bool begin(int baud_rate);

  uint8_t data_pin;
  uint8_t request_pin;

  uint8_t send_buf[MAX_PACKET];
  size_t send_len = 0;
  uint8_t rec_buf[MAX_PACKET];
  size_t rec_len = 0;

  /**
   * CRC value over all the data currently in send_buf.
   */
  uint16_t crc;

  int write(uint8_t c);

  /**
   * Are we receiving valid beacons through this channel?
   */
  bool connected;

  enum class LineState:uint8_t {
    IDLE, /* Idle, data pin pullup pulls high */
    REQUEST, /* We have data to send, request pin pulls low */
    SIGNAL, /* Signal peer, data pin forces high */
  };

  /**
   * A subset of the protocol states. Includes all states that we can be
   * in when handle_pending() returns.
   */
  enum class ProtocolState:uint8_t {
    IDLE,
    INACTIVE,
    DATA_READY,
    RECEIVING,
    SENDING,
  };

  ProtocolState protstate;

  /**
   * The timestamp of the next allowed / planned state change (lower 16
   * bits of micros()).
   *
   * When the protstate is IDLE or INACTIVE, this indicates when a new
   * request may be started. When protstate is DATA_READY, this
   * indicates when a collision check should be done. When protstate is
   * RECEIVING this indicates when reception is finished (if no new byte
   * is received beforehand). When protstate is RECEIVER this is the
   * time when the reception is finished (reset whenever a new byte is
   * received).
   */
  uint16_t wait;

  /**
   * SoftwareSerial instance used to send/receive on this channel.
   */
  SoftwareSerial serial;

  Callbacks *callbacks;

  /**
   * Check if our peer wants to send anything and receive it if so.
   */
  bool handle_pending();
  bool handle_send_ack();
  bool handle_receive_ack();
  bool initiate_request();
  bool cancel_request();
  bool collision_check();
  bool handle_receiving();
  bool check_wait();
  void set_wait(uint16_t wait);
  bool handle_sending_done();

  bool setActive(bool active);

  void setCallbacks(Callbacks *callbacks);

  void startPacket();
  void finishPacket();

  /**
   * Check if the line is currently in the state given by if_read (HIGH,
   * LOW) and change the state to new_state if so.
   *
   * Returns true if the state was changed and false if the line was in
   * the wrong state.
   */
  bool check_and_change_linestate(uint8_t check, LineState new_state);
  bool change_linestate(LineState new_state);

#if 0
  /**
   * Time of last beacon received as returned by millis().
   *
   * This is only the lower 16 bits, since the beacon timeout should be
   * much less than 65536 ms.
   *
   * Only valid when connected.
   */
  uint16_t last_beacon_received;

  /**
   * How many tick interrupts should occur before the next frame starts.
   *
   * Only valid when connected.
   */
  uint8_t ticks_to_next_frame;

  /**
   * The number of the current frame within this cycle
   *
   * Only valid when connected.
   */
  uint8_t frame_num;

  /**
   * How many tick interrupts should occur before the next random beacon
   * should be sent.
   *
   * Only valid when not connected.
   */
  uint16_t ticks_to_next_beacon;

  /**
   * Send any pending events or beacons, if the time is right.
   */
  void sendPending();

  /**
   * Receives the next lengthof(magic) bytes, as long as their values
   * are ok. If all bytes are ok, returns true, otherwise returns false
   * (in the latter case, less than lengthof(magic) may have been read).
   *
   * Should only be called when at least lengthof(magic) bytes are
   * available in the Stream).
   */
  bool checkMagic();

  void sendPacket(uint8_t flags, uint8_t type);

  /**
   * Send a beacon packet.
   */
  void sendBeacon();

  /*
   * Should be called every 1024us from the timer interrupt.
   */
  void tick();

  void check_request();
  #endif
};

/* TODO: static_assert variable sizes against constants */

#endif // __ONE_WIRE_SYMMETRICAL_H

/* vim: set sw=2 sts=2 expandtab filetype=cpp: */
