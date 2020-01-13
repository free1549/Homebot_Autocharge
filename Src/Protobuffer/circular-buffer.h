#ifndef CIRCULAR_BUF_H_
#define CIRCULAR_BUF_H_
#include <inttypes.h>
#include <stddef.h>
#include <algorithm> // for std::min
class CircularBuffer
{ 
    public: 
        static const size_t capacity_ = 2000; 
        CircularBuffer();

        ~CircularBuffer();

        size_t capacity() const { return capacity_; }
        // Return number of bytes written.
        size_t write(const char *data, size_t bytes);
        size_t write(const char data);
        // Return number of bytes read.
        size_t read(void *data, size_t bytes);
        size_t size() { return writeSize_ - readSize_; }
    private:
        size_t overflow_count_ = 0;
        size_t writeSize_ = 0;
        size_t readSize_ = 0;
        size_t size_ = 0;
        size_t beg_index_, end_index_;
        char data_[capacity_];
};

#endif
