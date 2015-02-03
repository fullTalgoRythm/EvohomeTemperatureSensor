/*
  CircularBuffer.h - circular buffer library for Arduino.

  Copyright (c) 2015 Hydrogenetic - mods for interupt usage
  Copyright (c) 2009 Hiroki Yagita.

  Permission is hereby granted, free of charge, to any person obtaining
  a copy of this software and associated documentation files (the
  'Software'), to deal in the Software without restriction, including
  without limitation the rights to use, copy, modify, merge, publish,
  distribute, sublicense, and/or sell copies of the Software, and to
  permit persons to whom the Software is furnished to do so, subject to
  the following conditions:

  The above copyright notice and this permission notice shall be
  included in all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED 'AS IS', WITHOUT WARRANTY OF ANY KIND,
  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
  IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
  CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
  TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
#ifndef CIRCULARBUFFER_h
#define CIRCULARBUFFER_h
#include <inttypes.h>

template <typename T, typename ST, ST Size>
class CircularBuffer {
public:
  enum {
    Empty = 0,
    Half = Size / 2,
    Full = Size,
  };

  CircularBuffer() :
    wp_(buf_), rp_(buf_), tail_(buf_+Size),  remain_(0) {for(uint8_t n=0;n<sizeof(mark_);n++)mark_[n]=0;}
  ~CircularBuffer() {}
  void push(T value, bool mark=false) volatile{
    if(mark){
      ST pos(buf_-wp_);
      mark_[pos/8]|=1<<(pos%8);
    }
    *wp_++ = value;
    remain_++;
    if (wp_ == tail_) wp_ = buf_;
  }
  //don't use this in an interrupt
  T pop(bool &mark) volatile{
    ST pos(buf_-rp_);
    //this section needs to be atomic
    //1. we could potentially be accessing the same byte of mark
    //2. remain-- is probably not atomic i.e. could be 2 or more ops (read,dec,write)
    cli(); // disable interrupts
    mark=mark_[pos/8]&(1<<(pos%8));
    if(mark) mark_[pos/8]&=~(1<<(pos%8));
    T result = *rp_++;
    remain_--;
    sei(); // remember don't call this in an interrupt as it will re-enable interrupts (including the one we are in)
    if (rp_ == tail_) rp_ = buf_;
    return result;
  }
  ST remain() const volatile{
    return remain_;
  }

private:
  volatile uint8_t mark_[(Size+7)/8];
  T buf_[Size];
  T *wp_;
  T *rp_;
  T *tail_;
  ST remain_;
};

#endif

