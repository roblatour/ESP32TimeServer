#pragma once

#include <cstddef>
#include <cstdint>

class Stream
{
public:
    virtual ~Stream() = default;
    virtual int available() = 0;
    virtual int read() = 0;
    virtual int peek() = 0;
    virtual size_t readBytes(char *buffer, size_t length) = 0;
    virtual size_t readBytes(uint8_t *buffer, size_t length) = 0;
    virtual size_t write(uint8_t value) = 0;
    virtual size_t write(const uint8_t *buffer, size_t length) = 0;
    virtual void flush() = 0;

    void setTimeout(unsigned long timeout) { timeout_ = timeout; }
    unsigned long getTimeout() const { return timeout_; }

private:
    unsigned long timeout_ = 1000;
};
