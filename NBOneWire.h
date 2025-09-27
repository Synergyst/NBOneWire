// Non-blocking one-wire soft serial (8-N-1) for Arduino (no interrupts)
// ------------------- TX -------------------
class NBOneWireTX {
public:
  NBOneWireTX(uint8_t pin, uint32_t baud = 1200)
    : _pin(pin),
      _bitUs(1000000UL / baud) {}

  void begin() {
    pinMode(_pin, INPUT_PULLUP);  // idle high
    _state = IDLE;
    _head = _tail = 0;
  }

  bool write(uint8_t b) {
    uint8_t nextHead = (uint8_t)(_head + 1) & BUF_MASK;
    if (nextHead == _tail) return false;  // full
    _buf[_head] = b;
    _head = nextHead;
    return true;
  }

  size_t write(const uint8_t* data, size_t len) {
    size_t n = 0;
    for (; n < len; ++n) {
      if (!write(data[n])) break;
    }
    return n;
  }

  void update() {
    unsigned long now = micros();
    if (_state == IDLE) {
      if (_tail != _head) {
        _cur = _buf[_tail];
        _tail = (uint8_t)(_tail + 1) & BUF_MASK;
        driveLow();  // start bit
        _bitIdx = 0;
        _nextEdge = now + _bitUs;
        _state = START;
      }
      return;
    }

    if ((long)(now - _nextEdge) < 0) return;  // wait

    switch (_state) {
      case START:
        {
          sendBit(_cur & 0x01);
          _cur >>= 1;
          _bitIdx = 1;
          _nextEdge += _bitUs;
          _state = DATA;
        }
        break;

      case DATA:
        {
          sendBit(_cur & 0x01);
          _cur >>= 1;
          _bitIdx++;
          _nextEdge += _bitUs;
          if (_bitIdx >= 8) {
            releaseHigh();  // stop bit
            _state = STOP;
          }
        }
        break;

      case STOP:
        {
          _state = IDLE;
        }
        break;
    }
  }

  bool busy() const {
    return !(_state == IDLE && _head == _tail);
  }

  size_t availableForWrite() const {
    if (_head >= _tail) return BUF_SIZE - 1 - (_head - _tail);
    return (_tail - _head - 1);
  }

private:
  inline void driveLow() {
    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, LOW);
  }
  inline void releaseHigh() {
    pinMode(_pin, INPUT_PULLUP);
  }
  inline void sendBit(uint8_t bit) {
    if (bit) releaseHigh();
    else driveLow();
  }

  static constexpr uint8_t BUF_SIZE = 64;
  static constexpr uint8_t BUF_MASK = BUF_SIZE - 1;

  enum State : uint8_t { IDLE,
                         START,
                         DATA,
                         STOP };

  uint8_t _pin;
  unsigned long _bitUs;
  volatile uint8_t _buf[BUF_SIZE]{};
  volatile uint8_t _head = 0, _tail = 0;

  State _state = IDLE;
  uint8_t _cur = 0;
  uint8_t _bitIdx = 0;
  unsigned long _nextEdge = 0;
};

// ------------------- RX -------------------
class NBOneWireRX {
public:
  NBOneWireRX(uint8_t pin, uint32_t baud = 1200)
    : _pin(pin),
      _bitUs(1000000UL / baud) {}

  void begin() {
    pinMode(_pin, INPUT_PULLUP);
    _state = IDLE;
    _head = _tail = 0;
    _framingErrors = 0;
  }

  void update() {
    unsigned long now = micros();

    switch (_state) {
      case IDLE:
        {
          if (digitalRead(_pin) == LOW) {
            _sampleAt = now + _bitUs + _bitUs / 2;  // 1.5T
            _cur = 0;
            _bitIdx = 0;
            _state = DATA;
          }
        }
        break;

      case DATA:
        {
          if ((long)(now - _sampleAt) < 0) break;
          uint8_t bit = (digitalRead(_pin) == HIGH) ? 1 : 0;
          _cur |= (bit << _bitIdx);  // LSB first
          _bitIdx++;
          _sampleAt += _bitUs;
          if (_bitIdx >= 8) _state = STOP;
        }
        break;

      case STOP:
        {
          if ((long)(now - _sampleAt) < 0) break;
          if (digitalRead(_pin) == HIGH) enqueue(_cur);
          else _framingErrors++;
          _state = IDLE;
        }
        break;
    }
  }

  int read() {
    if (_tail == _head) return -1;
    uint8_t b = _buf[_tail];
    _tail = (uint8_t)(_tail + 1) & BUF_MASK;
    return b;
  }

  int available() const {
    if (_head >= _tail) return _head - _tail;
    return BUF_SIZE - (_tail - _head);
  }

  uint16_t framingErrors() const {
    return _framingErrors;
  }

private:
  static constexpr uint8_t BUF_SIZE = 64;
  static constexpr uint8_t BUF_MASK = BUF_SIZE - 1;

  inline void enqueue(uint8_t b) {
    uint8_t nextHead = (uint8_t)(_head + 1) & BUF_MASK;
    if (nextHead == _tail) return;  // overflow: drop
    _buf[_head] = b;
    _head = nextHead;
  }

  uint8_t _pin;
  unsigned long _bitUs;

  enum State : uint8_t { IDLE,
                         DATA,
                         STOP } _state = IDLE;
  unsigned long _sampleAt = 0;
  uint8_t _cur = 0;
  uint8_t _bitIdx = 0;

  volatile uint8_t _buf[BUF_SIZE]{};
  volatile uint8_t _head = 0, _tail = 0;

  uint16_t _framingErrors = 0;
};

// ------------------- Simple redundancy wrappers (repeat + majority vote) -------------------
// Usage:
//   NBOneWireRepeatTX rtx(tx, BAUD, 8 /*repeats*/, 1 /*gapCharTimes*/);
//   NBOneWireRepeatRX rrx(rx, BAUD, 8 /*repeats*/, 2 /*timeoutCharTimes*/);
// In loop(): rtx.update(); rrx.update();
// Send: rtx.write(b);  Receive: if (rrx.available()) { rrx.read(); }

class NBOneWireRepeatTX {
public:
  // repeats: how many times to send each byte (e.g., 8)
  // gapCharTimes: idle gap between repeats, in "character times" (10 bit times per char). e.g., 1 = one char-time.
  NBOneWireRepeatTX(NBOneWireTX& tx, uint32_t baud, uint8_t repeats = 8, uint8_t gapCharTimes = 1)
    : _tx(tx),
      _repeats(repeats ? repeats : 1),
      _bitUs(1000000UL / baud),
      _charUs((unsigned long)(10UL * (1000000UL / baud))),
      _gapUs((unsigned long)gapCharTimes * (unsigned long)(10UL * (1000000UL / baud))) {}

  void begin() {
    _qHead = _qTail = 0;
    _active = false;
    _repsLeft = 0;
    _nextSend = 0;
  }

  // Queue an application byte to be sent with redundancy
  bool write(uint8_t b) {
    uint8_t next = (uint8_t)(_qHead + 1) & Q_MASK;
    if (next == _qTail) return false;
    _q[_qHead] = b;
    _qHead = next;
    return true;
  }

  size_t write(const uint8_t* data, size_t len) {
    size_t n = 0;
    for (; n < len; ++n) {
      if (!write(data[n])) break;
    }
    return n;
  }

  // Non-blocking progress
  void update() {
    _tx.update();

    unsigned long now = micros();

    // If no active byte, pull next from app queue
    if (!_active && _qHead != _qTail) {
      _cur = _q[_qTail];
      _qTail = (uint8_t)(_qTail + 1) & Q_MASK;
      _repsLeft = _repeats;
      _active = true;
      _nextSend = now;  // send first copy ASAP
    }

    if (!_active) return;

    // Wait until it's time and TX has room, and ideally not mid-byte
    if ((long)(now - _nextSend) < 0) return;
    if (_tx.availableForWrite() == 0) return;
    if (_tx.busy()) {
      // Let the ongoing byte finish to keep clean spacing
      return;
    }

    if (_tx.write(_cur)) {
      _repsLeft--;
      _nextSend = micros() + _gapUs;  // schedule next repeat
      if (_repsLeft == 0) {
        _active = false;
      }
    }
  }

  // Optional: how many bytes still waiting at the app level
  uint8_t queued() const {
    if (_qHead >= _qTail) return _qHead - _qTail;
    return (uint8_t)(Q_SIZE - (_qTail - _qHead));
  }

private:
  static constexpr uint8_t Q_SIZE = 64;
  static constexpr uint8_t Q_MASK = Q_SIZE - 1;

  NBOneWireTX& _tx;
  uint8_t _q[Q_SIZE]{};
  volatile uint8_t _qHead = 0, _qTail = 0;

  uint8_t _repeats = 8;
  bool _active = false;
  uint8_t _cur = 0;
  uint8_t _repsLeft = 0;
  unsigned long _nextSend = 0;

  unsigned long _bitUs = 0;
  unsigned long _charUs = 0;
  unsigned long _gapUs = 0;
};

// ------------------- Simple redundancy RX wrapper (repeat + majority vote) -------------------
class NBOneWireRepeatRX {
public:
  // repeats: expected repeats per byte (e.g., 8)
  // timeoutCharTimes: if gap between samples exceeds this many char-times, finalize the group early
  NBOneWireRepeatRX(NBOneWireRX& rx, uint32_t baud, uint8_t repeats = 8, uint8_t timeoutCharTimes = 2)
    : _rx(rx),
      _repeats(repeats ? repeats : 1),
      _bitUs(1000000UL / baud),
      _charUs((unsigned long)(10UL * (1000000UL / baud))),
      _timeoutUs((unsigned long)timeoutCharTimes * (unsigned long)(10UL * (1000000UL / baud))) {}

  void begin() {
    _oHead = _oTail = 0;
    _sampleCount = 0;
    _lastSampleAt = 0;
  }

  void update() {
    _rx.update();

    // Pull raw bytes from underlying RX and group them into samples
    while (_rx.available() > 0) {
      unsigned long now = micros();

      int v = _rx.read();
      if (v < 0) break;
      uint8_t b = (uint8_t)v;

      // If too much time passed since the last sample, finalize previous group
      if (_sampleCount > 0 && (long)(now - _lastSampleAt) > (long)_timeoutUs) {
        finalizeGroup();
      }

      // Add sample
      if (_sampleCount < MAX_REPEATS) {
        _samples[_sampleCount++] = b;
      } else {
        // if more than MAX_REPEATS arrive, finalize and start fresh
        finalizeGroup();
        _samples[_sampleCount++] = b;
      }
      _lastSampleAt = now;

      // If we reached expected repeats, finalize group
      if (_sampleCount >= _repeats) {
        finalizeGroup();
      }
    }

    // Timeout-based finalize (no new bytes coming)
    unsigned long now2 = micros();
    if (_sampleCount > 0 && (long)(now2 - _lastSampleAt) > (long)_timeoutUs) {
      finalizeGroup();
    }
  }

  int available() const {
    if (_oHead >= _oTail) return _oHead - _oTail;
    return OUT_SIZE - (_oTail - _oHead);
  }

  int read() {
    if (_oHead == _oTail) return -1;
    uint8_t b = _out[_oTail];
    _oTail = (uint8_t)(_oTail + 1) & OUT_MASK;
    return b - 128;
  }

private:
  static constexpr uint8_t MAX_REPEATS = 16;  // safety bound
  static constexpr uint8_t OUT_SIZE = 128;
  static constexpr uint8_t OUT_MASK = OUT_SIZE - 1;

  NBOneWireRX& _rx;

  // NEW: store the expected repeats
  uint8_t _repeats = 8;

  // Majority buffer
  uint8_t _samples[MAX_REPEATS]{};
  uint8_t _sampleCount = 0;
  unsigned long _lastSampleAt = 0;

  // Output queue
  uint8_t _out[OUT_SIZE]{};
  volatile uint8_t _oHead = 0, _oTail = 0;

  // Timing
  unsigned long _bitUs = 0;
  unsigned long _charUs = 0;
  unsigned long _timeoutUs = 0;

  // Push to output queue
  inline void outPush(uint8_t b) {
    uint8_t next = (uint8_t)(_oHead + 1) & OUT_MASK;
    if (next == _oTail) return;  // drop on overflow
    _out[_oHead] = b;
    _oHead = next;
  }

  void finalizeGroup() {
    if (_sampleCount == 0) return;

    // Find mode (majority). For small N, O(n^2) is fine.
    uint8_t bestVal = _samples[0];
    uint8_t bestCnt = 1;

    for (uint8_t i = 0; i < _sampleCount; ++i) {
      uint8_t val = _samples[i];
      uint8_t cnt = 1;
      for (uint8_t j = i + 1; j < _sampleCount; ++j) {
        if (_samples[j] == val) cnt++;
      }
      if (cnt > bestCnt) {
        bestCnt = cnt;
        bestVal = val;
      }
    }

    outPush(bestVal);

    _sampleCount = 0;
    _lastSampleAt = 0;
  }
};
