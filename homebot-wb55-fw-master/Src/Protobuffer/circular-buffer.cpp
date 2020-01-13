#include "circular-buffer.h"
#include <string.h>
CircularBuffer::CircularBuffer()
    : beg_index_(0)
    , end_index_(0)
      , size_(0)
{

}

CircularBuffer::~CircularBuffer()
{
}

size_t CircularBuffer::write(const char data) {
    data_[end_index_] = data;
    end_index_++;
    size_++;
    if (end_index_ == capacity_) end_index_ = 0;
    return 1;
}
size_t CircularBuffer::write(const char *data, size_t bytes)
{
    if (bytes == 0) return 0;

    size_t capacity = capacity_;
    size_t bytes_to_write = std::min(bytes, capacity - size());

    if(bytes_to_write < bytes) {
        overflow_count_++;
    }
    // Write in a single step
    if (bytes_to_write <= capacity - end_index_)
    {
        memcpy(data_ + end_index_, data, bytes_to_write);
        end_index_ += bytes_to_write;
        if (end_index_ == capacity) end_index_ = 0;
    }
    // Write in two steps
    else
    {
        size_t size_1 = capacity - end_index_;
        memcpy(data_ + end_index_, data, size_1);
        size_t size_2 = bytes_to_write - size_1;
        memcpy(data_, data + size_1, size_2);
        end_index_ = size_2;
    }

    writeSize_ += bytes_to_write;
    // size_ += bytes_to_write;
    return bytes_to_write;
}

size_t CircularBuffer::read(void *data, size_t bytes)
{
    if (bytes == 0) return 0;

    size_t capacity = capacity_;
    size_t bytes_to_read = 0;
    size_t s = size();
    if(bytes <= s) {
        bytes_to_read = bytes;
    }
    else {
        bytes_to_read = s;
    }

    // Read in a single step
    if (bytes_to_read <= capacity - beg_index_)
    {
        memcpy(data, data_ + beg_index_, bytes_to_read);
        beg_index_ += bytes_to_read;
        if (beg_index_ == capacity) beg_index_ = 0;
    }
    // Read in two steps
    else
    {
        size_t size_1 = capacity - beg_index_;
        memcpy(data, data_ + beg_index_, size_1);
        size_t size_2 = bytes_to_read - size_1;
        memcpy((char*)data + size_1, data_, size_2);
        beg_index_ = size_2;
    }

    readSize_ += bytes_to_read;
    // size_ -= bytes_to_read;
    return bytes_to_read;
}
