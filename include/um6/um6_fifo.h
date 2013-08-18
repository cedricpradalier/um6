#ifndef INCLUDE_FIFO_H_
#define INCLUDE_FIFO_H_

#include <boost/thread.hpp>

namespace um6 {
    class CharFIFO {
        protected:
            typedef boost::mutex::scoped_lock Lock;
            boost::mutex mutex;

            static const unsigned int fifo_size = 1024;
            static const unsigned int fifo_size_mask = 0x3FF;

            uint8_t fifo[fifo_size];
            unsigned int fifo_read;
            unsigned int fifo_write;

            inline size_t unsafe_available() {
                if (fifo_write < fifo_read) {
                    return fifo_write + fifo_size - fifo_read;
                } else {
                    return fifo_write - fifo_read;
                }
            }


        public:
            CharFIFO() {
                fifo_read = fifo_write = 0;
            }

            void clear() {
                Lock lock(mutex);
                fifo_read = fifo_write = 0;
            }

            void push(uint8_t c) {
                Lock lock(mutex);
                fifo[fifo_write] = c;
                fifo_write = (fifo_write+1)&fifo_size_mask;
            }

            void push(uint8_t *c, size_t size) {
                Lock lock(mutex);
                for (size_t i=0;i<size;i++) {
                    push(*c);
                }
            }

            bool pop(uint8_t * c = NULL) {
                Lock lock(mutex);
                if (fifo_read == fifo_write) {
                    return false;
                }
                if (c) *c = fifo[fifo_read];
                fifo_read = (fifo_read+1)&fifo_size_mask;
                return true;
            }

            void discard_until(uint8_t value) {
                Lock lock(mutex);
                size_t avail = unsafe_available();
                for (size_t i=0;i<avail;i++) {
                    if (fifo[fifo_read] == value) {
                        return;
                    }
                    fifo_read = (fifo_read+1)&fifo_size_mask;
                }
            }

            size_t available() {
                Lock lock(mutex);
                if (fifo_write < fifo_read) {
                    return fifo_write + fifo_size - fifo_read;
                } else {
                    return fifo_write - fifo_read;
                }
            }

            size_t pop(uint8_t * c, size_t size) {
                Lock lock(mutex);
                size_t avail = unsafe_available();
                if (size > avail) {
                    size = avail;
                }
                for (size_t i=0;i<size;i++) {
                    if (c) {
                        pop(c++);
                    } else {
                        pop(NULL);
                    }
                }
                return size;
            }

            size_t peek(uint8_t * c, size_t size=1) {
                Lock lock(mutex);
                size_t avail = unsafe_available();
                if (size > avail) {
                    size = avail;
                }
                unsigned int cursor = fifo_read;
                for (size_t i=0;i<size;i++) {
                    *c = fifo[cursor];
                    c ++;
                    cursor = (cursor + 1) & fifo_size_mask;
                }
                return size;
            }
    };

};

#endif // INCLUDE_FIFO_H_
