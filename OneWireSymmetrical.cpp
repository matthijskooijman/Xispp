#include "Arduino.h"
#include "OneWireSymmetrical.h"
#include <TStreaming.h>
#include <util/atomic.h>

#ifndef lengthof
#define lengthof(x) (sizeof(x)/sizeof(*x))
#endif
#define bitsize(t) (sizeof(t) * 8)
#define maxval(t) ((1LL << bitsize(t)) - 1)

/*
 * A "small delay", in us. This must be big enough that an active peer
 * has the chance to read our state and perform one or a few state
 * transitions.  Since those transitions occur with interrupts enabled,
 * this delay must be long enough to cover the handling of all
 * interrupts. "Active peer" here means that the peer is checking the
 * line directly after a state change of its own and or polling the
 * line.
 *
 * A typical timer interrupt seems to take 5-10 us, so setting this
 * value to that amount seems to be sufficient.
 */
#define SMALL_DELAY 10

/*
 * The maximum time between calls to send_pending(), in us.
 */
#define LOOP_TIME 400

/*
 * The ack time, in us.
 * This must be bigger than the interval between line checks of an
 * inactive peer in the DATA_READY state.
 *
 * This is the sum of a worst-case estimation of a single loop
 * iteration, plus a SMALL_DELAY to account for interrupt handling
 * delay.
 *
 * Choosing this value too low can cause acks to be missed and an
 * invalid state to be reached (sender DATA_READY, receiver COMM).
 * Choosing this value higher will increase the duration of a handshake,
 * but since it's only in the us range, that's not a big problem.
 */
#define ACK_TIME (LOOP_TIME + SMALL_DELAY)

/**
 * The amount of time (in us) we must spend idle before bringing the
 * line into the REQUEST state again after canceling a request. This is
 * to prevent us going from DATA_READY to IDLE to DATA_READY exactly
 * while the peer sending an ack so we don't see its ack and the peer
 * doesn't see our cancel.
 *
 * It should be at least as big as ACK_TIME, plus interrupt handling
 * margin.
 */
#define CANCEL_BACKOFF_TIME (ACK_TIME + SMALL_DELAY)

/**
 * The amount of time we should spend in the DATA_READY state before
 * concluding there is nobody listening (probably because they are also
 * in the DATA_READY state waiting for us).
 *
 * This should be at least as long as the LOOP_TIME, for the time until
 * our peer will have seen our request, plus one SMALL_DELAY for the
 * time until it will start sending its ack, plus a bit of interrupt
 * handling margin.
 */
#define COLLISION_CHECK_TIME (LOOP_TIME + SMALL_DELAY + SMALL_DELAY)

/**
 * A bit of randomization for the DATA_READY timeout. The only way both
 * peers can end up in the DATA_READY state together is when they decide
 * to enter it pretty much at the same moment. By adding a bit of random
 * to the timeout, we reduce the chance of both peers canceling at the
 * same time and needing a retry.
 *
 * Using a power of two is preferred, since that allows using a bitmask
 * instead of a real modulo.
 */
#define MAX_COLLISION_CHECK_RANDOM 32

#define SENDER_DONE_WAIT BYTE_TIME
#define RECEIVER_DONE_TIMEOUT (BYTE_TIME * 2 / 4)

static_assert(maxval(OneWireSymmetrical::wait) / 2 > COLLISION_CHECK_TIME + MAX_COLLISION_CHECK_RANDOM, "OneWireSymmetrical::wait too small");
static_assert(maxval(OneWireSymmetrical::wait) / 2 > CANCEL_BACKOFF_TIME, "OneWireSymmetrical::wait too small");

// Calculate the difference between two timestamps. T must be an
// unsigned type, otherwise overflowing will not work out.
template<typename T>
T time_diff(T t1, T t2) {
  // Cast back to T, subtraction will have a signed result...
  return (T)(t2 - t1);
}

OneWireSymmetrical::OneWireSymmetrical(uint8_t data_pin, uint8_t request_pin) : serial(data_pin, data_pin)
{
  this->connected = false;
  //this->ticks_to_next_beacon = 1;
  this->data_pin = data_pin;
  this->request_pin = request_pin;
  this->protstate = ProtocolState::INACTIVE;
}
const bool debug = false;

bool OneWireSymmetrical::begin(int baud_rate)
{
  /* Both pins must live in the same port, so we can atomically change
   * our line state. */
  if (digitalPinToPort(this->data_pin) != digitalPinToPort(this->request_pin))
    return false;

  serial.begin(BAUD_RATE);
  serial.stopListening();

  /* This pin is used for the actual datatransfer, has the pullup to
   * float the line normally high and can be used to force the pin
   * high when the other end is driving it low using its request pin.
   * During handshaking, this pin should always be either INPUT_PULLUP
   * or OUTPUT and HIGH, never OUTPUT and LOW (this could cause short
   * circuit).
   */
  pinMode(this->data_pin, INPUT_PULLUP);

  /* This pin is behind a resistor, so we can drive it low and still
   * see the line going high when the other end forces it high using
   * its data pin (at the expense of a few mA of current flowing over
   * the line).
   * During handshaking, this pin should always be either INPUT or
   * OUTPUT LOW, never OUTPUT HIGH (it won't hurt, but it should never
   * be needed).
   */
  pinMode(this->request_pin, INPUT);

  this->protstate = ProtocolState::IDLE;
  this->set_wait(0);
  return true;
}

void OneWireSymmetrical::setCallbacks(Callbacks *callbacks)
{
  this->callbacks = callbacks;
}

bool OneWireSymmetrical::change_linestate(LineState new_state)
{
  // This function effectively inliens the relevant parts of pinMode.
  // This is to make the change in the two pins happen atomically.
  volatile uint8_t *mode_reg = portModeRegister(digitalPinToPort(this->data_pin));
  uint8_t data_bit = digitalPinToBitMask(this->data_pin);
  uint8_t request_bit = digitalPinToBitMask(this->request_pin);

  // Read the existing pin modes
  uint8_t mode = *mode_reg;

  // Prepare the mode change
  switch(new_state) {
    default:
    case LineState::IDLE:
      mode &= ~data_bit;
      mode &= ~request_bit;
      break;
    case LineState::REQUEST:
      mode &= ~data_bit;
      mode |= request_bit;
      break;
    case LineState::SIGNAL:
      mode |= data_bit;
      mode &= ~request_bit;
      break;
  }

  // Change the state atomically
  *mode_reg = mode;

  // Wait a bit after changing the line state. See
  // check_and_change_linestate about why this is necessary.
  delayMicroseconds(1);
  return true;
}

bool OneWireSymmetrical::check_and_change_linestate(uint8_t check, LineState new_state)
{
  // This function effectively inliens the relevant parts of digitalRead
  // and pinMode. This serves three purposes:
  //  1. Minimize the time interrupts need to be disabled
  //  2. Minimize the time between reading the line and changing state
  //     (see the comment at the end of this function for why this is
  //     relevant)
  //  3. Make the changes to the two pins happen atomically
  volatile uint8_t *data_in = portInputRegister(digitalPinToPort(this->data_pin));
  volatile uint8_t *mode_reg = portModeRegister(digitalPinToPort(this->data_pin));
  uint8_t data_bit = digitalPinToBitMask(this->data_pin);
  uint8_t request_bit = digitalPinToBitMask(this->request_pin);
  uint8_t check_val = (check == HIGH ? data_bit : 0);

  // Read the existing pin modes
  uint8_t mode = *mode_reg;

  // Prepare the mode change
  switch(new_state) {
    default:
    case LineState::IDLE:
      mode &= ~data_bit;
      mode &= ~request_bit;
      break;
    case LineState::REQUEST:
      mode &= ~data_bit;
      mode |= request_bit;
      break;
    case LineState::SIGNAL:
      mode |= data_bit;
      mode &= ~request_bit;
      break;
  }

  // Check the line and, if its state is ok, change state. This needs to
  // be atomic so we can guarantee the time between reading the line and
  // changing the line (see comment below on why this is important).
  //
  // This block takes around 10 cycles
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    if (check_val != (*data_in & data_bit))
      return false;

    *mode_reg = mode;
  }

  // Wait a bit, so we're entirely sure the other side has completed any
  // state transitions of its own (based on the old state) before we try
  // to read the line again. This makes the following can not happen:
  //     peer1        peer2
  //       |            |
  //   read line        |
  //       |            |
  //       |            |
  //       |        read line
  //  change line       |
  //       |            |
  //   read line        |
  //       |       change line
  //       |            |
  //       |            |
  //  change line       |
  //       |            |
  //
  // Above, both peers change the line based on the same line reading,
  // but peer1 gets to change the line a second time without seeing the
  // change by peer2. By adding a small delay (e.g., making sure that
  // the time between a change and the subsequent read is at least as
  // large as the time between a read and the corresponding change), we
  // prevent the above. The worst that can happen now is:
  //
  //     peer1        peer2
  //       |            |
  //   read line        |
  //       |        read line
  //  change line       |
  //       |       change line
  //       |            |
  //   read line        |
  //       |            |
  //       |            |
  //
  // Now, the two peers still change the line at the same time, but
  // they're guaranteed to see the other's change when trying to change
  // the line for the second time (so peer1 in this case could decide
  // not to do the second change because the of the change by peer2).
  // Use a delay of 2 microseconds (~= 32 cycles), which is definitely
  // larger than the above ~20 cycles
  delayMicroseconds(1);
  return true;
}

bool OneWireSymmetrical::handle_pending() {
  bool wait_reached = this->check_wait();

  switch (this->protstate) {
    case ProtocolState::IDLE:
      if (digitalRead(this->data_pin) == LOW) {
        return this->handle_send_ack();
      } else if (wait_reached && this->send_len) {
        return this->initiate_request();
      }
      break;
    case ProtocolState::DATA_READY:
      if (wait_reached) {
        /* No ack for too long, we are probably disconnected, or our peer
         * is also in the DATA_READY state. */
        return this->collision_check();
      } else if (digitalRead(this->data_pin) == HIGH) {
        return this->handle_receive_ack();
      }
      break;
    case ProtocolState::RECEIVING:
      return this->handle_receiving();
      break;
    case ProtocolState::SENDING:
      if (wait_reached)
        return this->handle_sending_done();
      break;
    case ProtocolState::INACTIVE:
      // Do nothing
      break;
  }
  return false;
}


// Called when our state is DATA_READY and we haven't seen an ack for at
// least DATA_READY_TIMEOUT us.
bool OneWireSymmetrical::collision_check()
{
  // STATE: DATA_READY
  if (!this->check_and_change_linestate(LOW, LineState::IDLE)) {
    // Line went high, just-in-time ack?
    return this->handle_receive_ack();
  }
  // STATE: COLLISION_CHECK
  if (digitalRead(this->data_pin) == LOW) {
      // Line stays low after we stop pulling it low, our peer is in
      // DATA_READY, so send an ack
      if (this->handle_send_ack()) {
        return true;
      }
  }
  // If we get here, we succesfully changed to idle and then didn't see
  // a request from the peer, or were to late to send an ack. Get back
  // into the request state.
  this->initiate_request();
  return true;
}

// Called when our state is DATA_READY and the line reads HIGH
bool OneWireSymmetrical::handle_receive_ack()
{
  // STATE: DATA_READY
  if (this->callbacks && !this->callbacks->becomeBusy(this)) {
    // Check if it's ok to become busy for a while (e.g., disable other
    // channels etc.). If not, then we'll have to cancel our request.
    return this->cancel_request();
  }

  // STATE: SEE_ACK
  while (digitalRead(this->data_pin) == HIGH) { /* busyloop */ }

  // The ack pulse ended. Wait a bit before floating the line high
  // again, so our peer can see we haven't canceled during their ack
  // STATE: SENDING_DELAY
  if(debug) Serial << "SENDING_DELAY" << endl;
  delayMicroseconds(SMALL_DELAY);

  // Run the txStart callback before making the line idle again, since a
  // bit of extra delay here shouldn't hurt
  if (this->callbacks)
    this->callbacks->txStart(this);

  this->change_linestate(LineState::IDLE);

  // STATE: SENDING
  if(debug) Serial << "SENDING" << endl;
  this->protstate = ProtocolState::SENDING;

  // Wait a bit more before actually starting to send so we can be sure
  // the other side actually started listening.
  delayMicroseconds(SMALL_DELAY);

  // Write out our buffer
  pinMode(this->data_pin, OUTPUT);
  this->serial.write(this->send_buf, this->send_len);
  this->serial.write(this->crc >> 8);
  this->serial.write(this->crc & 0xff);
  this->send_len = 0;
  pinMode(this->data_pin, INPUT_PULLUP);

  // Keep the line idle for a while so the receiver knows the
  // transmission is done
  set_wait(SENDER_DONE_WAIT);
  return true;
}

// Called when our state is SENDING and the timeout expired
bool OneWireSymmetrical::handle_sending_done() {
  if (this->callbacks)
    this->callbacks->txDone(this);

  // STATE: IDLE
  if(debug) Serial << "IDLE" << endl;
  this->protstate = ProtocolState::IDLE;
  return true;
}

// Called when the our state is IDLE and the line reads LOW
bool OneWireSymmetrical::handle_send_ack() {
  // STATE: IDLE
  if(debug) Serial << "IDLE" << endl;

  if (this->rec_len) {
    // Buffer is not empty yet, don't ack
    return false;
  }

  if (this->callbacks && !this->callbacks->becomeBusy(this)) {
    // Check if it's ok to become busy for a while (e.g., disable other
    // channels etc.). If not, then just don't ack the request.

    // TODO: become unavailable?
    return false;
  }

  // Start the ack
  if (!this->check_and_change_linestate(LOW, LineState::SIGNAL)) {
    // Our peer apparantly canceled the request, our state didn't change
    // STATE: IDLE
    if(debug) Serial << "IDLE" << endl;
    return false;
  }

  // STATE: SEND_ACK
  if(debug) Serial << "SEND_ACK" << endl;

  // Keep our ack on for a while, so we're sure the other side sees it
  delayMicroseconds(ACK_TIME);

  // And turn it off again
  this->change_linestate(LineState::IDLE);

  // STATE: ACK_SENT
  if(debug) Serial << "ACK_SENT" << endl;

  if (digitalRead(this->data_pin) == HIGH) {
    // The line is no longer low, this means our peer canceled its
    // request just before it could see our ack.
    // STATE: IDLE
    if(debug) Serial << "IDLE" << endl;
    return false;
  }

  // Call the callbacks now, since a bit of extra delay here shouldn't
  // hurt
  if (this->callbacks)
    this->callbacks->rxStart(this);

  // The other side is still in the request state, so they must have
  // seen our ack.
  // Wait for the peer to finish its COMM_DELAY state
  while(digitalRead(this->data_pin) == LOW) { /* busyloop */ }

  // STATE: RECEIVING
  if(debug) Serial << "RECEIVING" << endl;
  this->protstate = ProtocolState::RECEIVING;

  // Enable serial reception
  this->serial.listen();

  // Reception is finished when nothing is received for a while
  set_wait(RECEIVER_DONE_TIMEOUT);

  return true;
}

// Called when our state is RECEIVING
bool OneWireSymmetrical::handle_receiving()
{
  if (this->check_wait()) {
    // If we get here, than at least RECEIVER_DONE_TIMEOUT has passed.
    // This can happen for two reasons:
    //   1. No start bit was seen for that amount of time
    //   2. A start bit was seen before the timeout, and the blocking
    //      interrupt handler of SoftwareSerial received a full byte. In
    //      this case, we'll have significantly exceeded the timeout, but
    //      we couldn't detect that before because the interrupt handler
    //      was blocking.
    //   3. The receive buffer is has overflown, which disables
    //      the SoftwareSerial interrupt. This should not normally
    //      happen, but if for whatever reason we're in RECEIVING while
    //      our peer is in DATA_READY, our peer's CHECK_COLLISION ->
    //      DATA_READY transitions will look like start bits that will
    //      keep SoftwareSerial busy forever, so we need to handle this
    //      case.
    //
    // By checking the timer first and then the serial buffer instead of
    // the other way around, we prevent a race condition where we first
    // conclude the buffer is empty, then the SoftwareSerial interrupt
    // trigger, and then the timeout triggers leading use to believe
    // there wasn't a start bit for too long.
    if (this->serial.available()) {
      // This is case 2. above.
      while (true) {
        if (this->serial.overflow() || this->rec_len == lengthof(this->rec_buf)) {
          // This is case 3. above. We need to check this inside the loop,
          // since the otherwise we can get stuck in this loop without
          // detecting the overflow
          // Alternatively, the serial buffer might not be overflowing
          // yet, but our own buffer is, which we should handle the
          // same: Stop receiving and flush the incoming buffers.
          this->serial.stopListening();
          this->serial.flush();
          this->rec_len = 0;

          // STATE: IDLE
          Serial << "OVERFLOW" << endl;
          if(debug) Serial << "IDLE" << endl;
          this->protstate = ProtocolState::IDLE;

          // Call the rxDone callback with ok = false
          if (this->callbacks)
            this->callbacks->rxDone(this, false);
          return true;
        }
        int c = this->serial.read();
        //Serial << c << endl;
        if (c < 0)
          break;
        // Get the next byte
        this->rec_buf[this->rec_len++] = c;
      }
      // And reset the timeout
      this->set_wait(RECEIVER_DONE_TIMEOUT);
    } else {
      // This is case 1. above: Haven't received anything for a while,
      // so the transmission ended.
      // STATE: IDLE
      if(debug) Serial << "IDLE" << endl;
      this->protstate = ProtocolState::IDLE;
      this->serial.stopListening();

      // Check CRC
      bool ok = false;
      if (this->rec_len >= 2) {
        this->rec_len -= 2;
        uint16_t rec_crc = (this->rec_buf[this->rec_len] << 8) | this->rec_buf[this->rec_len + 1];
        uint16_t crc = calc_crc(this->rec_buf, this->rec_len);
        ok = (rec_crc == crc);
        if (!ok)
          Serial << "OVERFLOW" << endl;
      }

      if (this->callbacks)
        this->callbacks->rxDone(this, ok);
    }
  }

  return true;
}

// Called when our state is IDLE, the line reads HIGH and there is data
// to send.
bool OneWireSymmetrical::initiate_request()
{
  // STATE: IDLE
  if(debug) Serial << "IDLE" << endl;
  if (!this->check_and_change_linestate(HIGH, LineState::REQUEST)) {
    // Our peer just started a request of its own, so ack and receive
    // that one instead
    return this->handle_send_ack();
  }

  // STATE: DATA_READY
  if(debug) Serial << "DATA_READY" << endl;

  // Generate a random number using the micros timer. We just use the
  // lower bits of the timer, which should work well enough. We still
  // occasionally get collision checks happening at the same time, but
  // given that a collision is not so likely in the first place, that's
  // ok.
  uint8_t random = micros() % MAX_COLLISION_CHECK_RANDOM;
  this->set_wait(COLLISION_CHECK_TIME + random);
  this->protstate = ProtocolState::DATA_READY;
  return true;
}

/**
 * Cancel a pending request, if any.
 *
 * When a request is not pending, returns false.
 * When a request is pending, returns true and always returns
 * immediately (e.g., does not check the line for an ack to handle that
 * first).
 */
bool OneWireSymmetrical::cancel_request() {
  if (this->protstate != ProtocolState::DATA_READY)
    return false;

  // STATE: DATA_READY
  this->change_linestate(LineState::IDLE);

  // STATE: IDLE
  this->set_wait(CANCEL_BACKOFF_TIME);
  this->protstate = ProtocolState::IDLE;
  return true;
}

void OneWireSymmetrical::startPacket() {
  this->send_len = 0;
  this->crc = CRC_INIT;
}

int OneWireSymmetrical::write(uint8_t c)
{
  if (this->send_len >= lengthof(this->send_buf))
    return 0;
  this->send_buf[this->send_len++] = c;
  this->crc = calc_crc_step(c, this->crc);
  return 1;
}

void OneWireSymmetrical::finishPacket() {
  // Nothing to do here
}

/**
 * Update the wait timestamp.
 *
 * This resets the wait time to the given number of microseconds from
 * now.
 *
 * Wait must be < 128.
 */
void OneWireSymmetrical::set_wait(uint16_t wait)
{
  this->wait = ((uint16_t)micros()) + wait;
}

/**
 * Checks if the wait time has been reached and returns true if so. Once
 * the wait time is reached, this function continues to return true
 * until set_wait is called again.
 *
 * This function should be called at least once every 128 us to prevent
 * from overflowing the wait timestamp.
 */
bool OneWireSymmetrical::check_wait()
{
  uint16_t t = micros();
  if ((int16_t)(this->wait - t) <= 0) {
    // Update timestamp to the current time, to prevent the current time
    // to ever "catch up" on it.
    this->wait = t;
    return true;
  }
  //Serial << this->wait << " " << t << endl;
  return false;
}

bool OneWireSymmetrical::setActive(bool active) {
  // TODO: Improve control flow here
  if (this->protstate == ProtocolState::INACTIVE) {
    if (!active)
      return false; // Already in the right state

    // Activating is easy, just set the protstate
    this->protstate = ProtocolState::IDLE;

    return true;
  } else {
    if (active)
      return false; // Already in the right state

    if (this->protstate == ProtocolState::DATA_READY) {
      this->cancel_request();
      this->protstate = ProtocolState::INACTIVE;
      return true;
    }
    if (this->protstate == ProtocolState::IDLE) {
      this->protstate = ProtocolState::INACTIVE;
      return true;
    }

    // STATE:  RECEIVER or SENDER
    // Can't become inactive during a transmission
    return false;
  }
}

#if 0
void OneWireSymmetrical::receive() {
  /* TODO: Use a proper framing format */
  while(s.available() >= lengthof(magic) + 2) {
    /* Check magic bytes. If there is a mismatch, the remaining bytes
     * are left in the buffer and tried again if/when there are enough
     * bytes.
     */
    if (!checkMagic())
      continue;

    uint8_t flags = s.read();
    uint8_t type = s.read();
    Serial << "Read packet with flags 0x" << V<Hex>(flags) << " and type 0x" << V<Hex>(type) << endl;

    if (type == TYPE_BEACON) {
      this->connected = true;
      /* Receiving a beacon always starts a new cycle. This keeps us
       * synchronised properly. */
      this->last_beacon_received = (uint16_t)millis();
      /* How many clock cycles until the next tick? */
      //uint8_t next_tick = CYCLES_TO_US(OCR0A - TCNT0);
      //next_tick += 6 * BYTE_TIME;
      this->ticks_to_next_frame = FRAME_INTERVAL;
      //if (...) {
      //  this->ticks_to_next_frame--; /* of ++? */
      this->frame_num = 0;

      digitalWrite(LED_PIN, HIGH);
    }
  }

}

void OneWireSymmetrical::sendPending() {
}

/**
 * Receives the next lengthof(magic) bytes from the given Stream, as
 * long as their values are ok. If all bytes are ok, returns true,
 * otherwise returns false (in the latter case, less than
 * lengthof(magic) may have been read).
 *
 * Should only be called when at least lengthof(magic) bytes are
 * available in the Stream).
 */
bool OneWireSymmetrical::checkMagic()
{
  for (int i=0; i < lengthof(magic); ++i) {
    uint8_t value = s.read();
    if (value != magic[i]) {
      Serial << "Invalid magic byte read: 0x" << V<Hex>(value) << " (expected 0x" << V<Hex>(magic[i]) << ")" << endl;
      return false;
    }
  }
  return true;
}


void OneWireSymmetrical::sendPacket(uint8_t flags, uint8_t type) {
  pinMode(TXRX_PIN, OUTPUT);
  s.write(magic, lengthof(magic));

  s.write(flags);
  s.write(type);
  pinMode(TXRX_PIN, INPUT_PULLUP);
}

void OneWireSymmetrical::sendBeacon()
{
  sendPacket(0, TYPE_BEACON);
}

bool state = 0;
void OneWireSymmetrical::tick()
{
  digitalWrite(11, !digitalRead(11));
  /* If we haven't received beacons for too long, we're disconnected and
   * should start sending out beacons at random intervals. */
  if (this->connected && (uint16_t)millis() - last_beacon_received > BEACON_TIMEOUT) {
    this->connected = false;
    digitalWrite(LED_PIN, LOW);
    /* Send a beacon right away */
    this->ticks_to_next_beacon = 1;
  }

  if (this->connected) {
    if (--this->ticks_to_next_frame == 0) {
      digitalWrite(12, !digitalRead(12));
      /* Start of next frame */
      this->ticks_to_next_frame = FRAME_INTERVAL;
      this->frame_num = (this->frame_num + 1) % FRAMES_PER_CYCLE;

      if (this->frame_num == FRAMES_PER_CYCLE / 2) {
        /* This is our frame and it is time to send a beacon */
        this->sendBeacon();
      } else if (this->frame_num % 2 == 1) {
        //this->s.print("x"); /* This is our frame, send any queued events */
      }
    }
  } else {
    if (--this->ticks_to_next_beacon == 0) {
      this->ticks_to_next_beacon = random(RANDOM_BEACON_INTERVAL_MIN, RANDOM_BEACON_INTERVAL_MAX);
      this->sendBeacon();
    }
  }
}

/*
 * Timer interrupt that fires every 1024us. We do not control the phase
 * of the interrupt (which would only work for a single connected
 * channel anyway...). Since this timer is also used for PWM, changing
 * the PWM duty cycle on some specific pin could cause a phase shift in
 * for this timer interrupt (i.e.. time between two specific interrupts
 * is bigger or smaller than 1024us).
 */
ISR(TIMER0_COMPA_vect)
{
  for (int i=0; i < lengthof(channels); ++i) {
    channels[i].tick();
  }

}
#endif

