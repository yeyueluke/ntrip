// Copyright [year] <Copyright Owner>
#pragma once

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>

const int BUFFER_LEN = 1024;
class UartOperations {
 public:
  int UartOpen(const char *p_path) {
    if (p_path != nullptr) {
      fd_ = open(p_path, O_RDWR | O_NOCTTY);
    }
    return fd_;
  }

  int UartSet(int baudrate, int bits, char parity, int stop, char flow) {
    struct termios termios_uart;
    int ret = 0;
    speed_t uart_speed = 0;

    memset(&termios_uart, 0, sizeof(termios_uart));
    ret = tcgetattr(fd_, &termios_uart);
    if (ret == -1) {
      std::cout << "tcgetattr failed\n";
      return -1;
    }

    switch (baudrate) {
      case 0:
        uart_speed = B0;
        break;
      case 50:
        uart_speed = B50;
        break;
      case 75:
        uart_speed = B75;
        break;
      case 110:
        uart_speed = B110;
        break;
      case 134:
        uart_speed = B134;
        break;
      case 150:
        uart_speed = B150;
        break;
      case 200:
        uart_speed = B200;
        break;
      case 300:
        uart_speed = B300;
        break;
      case 600:
        uart_speed = B600;
        break;
      case 1200:
        uart_speed = B1200;
        break;
      case 1800:
        uart_speed = B1800;
        break;
      case 2400:
        uart_speed = B2400;
        break;
      case 4800:
        uart_speed = B4800;
        break;
      case 9600:
        uart_speed = B9600;
        break;
      case 19200:
        uart_speed = B19200;
        break;
      case 38400:
        uart_speed = B38400;
        break;
      case 57600:
        uart_speed = B57600;
        break;
      case 115200:
        uart_speed = B115200;
        break;
      case 230400:
        uart_speed = B230400;
        break;
      case 460800:
        uart_speed = B460800;
        break;
      case 500000:
        uart_speed = B500000;
        break;
      case 921600:
        uart_speed = B921600;
        break;
      default:
      std::cout << "Baud rate not supported: " << baudrate << std::endl;
        return -1;
    }
    cfsetspeed(&termios_uart, uart_speed);

    switch (bits) {
      case 5:
        termios_uart.c_cflag &= ~CSIZE;
        termios_uart.c_cflag |= CS5;
        break;

      case 6:
        termios_uart.c_cflag &= ~CSIZE;
        termios_uart.c_cflag |= CS6;
        break;

      case 7:
        termios_uart.c_cflag &= ~CSIZE;
        termios_uart.c_cflag |= CS7;
        break;

      case 8:
        termios_uart.c_cflag &= ~CSIZE;
        termios_uart.c_cflag |= CS8;
        break;

      default:
        std::cout << "Data bits not supported: " << bits << std::endl;
        return -1;
    }

    switch (parity) {
      case 'n':
      case 'N':
        termios_uart.c_cflag &= ~PARENB;
        termios_uart.c_iflag &= ~INPCK;
        break;

      case 'o':
      case 'O':
        termios_uart.c_cflag |= PARENB;
        termios_uart.c_cflag |= PARODD;
        termios_uart.c_iflag |= INPCK;
        termios_uart.c_iflag |= ISTRIP;
        break;

      case 'e':
      case 'E':
        termios_uart.c_cflag |= PARENB;
        termios_uart.c_cflag &= ~PARODD;
        termios_uart.c_iflag |= INPCK;
        termios_uart.c_iflag |= ISTRIP;
        break;

      default:
        std::cout << "Parity not supported: " << parity << std::endl;
        return -1;
    }

    switch (stop) {
      case 1:
        termios_uart.c_cflag &= ~CSTOPB;
        break;
      case 2:
        termios_uart.c_cflag |= CSTOPB;
        break;
      default:
        std::cout << "Stop bits not supported: " << stop << std::endl;
        break;
    }

    switch (flow) {
      case 'n':
      case 'N':
        termios_uart.c_cflag &= ~CRTSCTS;
        termios_uart.c_iflag &= ~(IXON | IXOFF | IXANY);
        break;

      case 'h':
      case 'H':
        termios_uart.c_cflag |= CRTSCTS;
        termios_uart.c_iflag &= ~(IXON | IXOFF | IXANY);
        break;

      case 's':
      case 'S':
        termios_uart.c_cflag &= ~CRTSCTS;
        termios_uart.c_iflag |= (IXON | IXOFF | IXANY);
        break;

      default:
      std::cout << "Flow control parameter error: " << flow << std::endl;
        return -1;
    }

    termios_uart.c_cflag |= CLOCAL;
    termios_uart.c_cflag |= CREAD;
    termios_uart.c_oflag &= ~OPOST;
    termios_uart.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    termios_uart.c_cc[VTIME] = 1;
    termios_uart.c_cc[VMIN] = 1;

    tcflush(fd_, TCIFLUSH);

    ret = tcsetattr(fd_, TCSANOW, &termios_uart);
    if (ret == -1) {
      std::cout << "tcsetattr failed" << std::endl;
    }

    return ret;
  }
  void UartClose() {
    if (fd_ > 0) close(fd_);
  }

  char readChar() {
    char c;
    int byte_read = read(fd_, &c, 1);
    if (byte_read > 0) {
        return c;
    } else {
        std::cout << "read failded\n";
        return -1;
    }
  }

  /* return char* string with maxBufferSize = 200  */
  char* readString() {
      return readString(200);
  }

  /* return char* string with maxBufferSize  */
  char* readString(int maxBufferSize) {
      int index = 0;
      char c = ' ';

      while (c != 0x0a && index < maxBufferSize - 2) {  // may be 0x0a(\n), 0x0d(\r)
          c = readChar();
          buf_[index] = c;
          index++;
      }
      buf_[index - 1] = 0;
      // buf_[index - 2] = 0;
      return buf_;
  }

  void clearBuf() { memset(buf_, 0, BUFFER_LEN);}

 private:
    int fd_ = -1;
    char buf_[BUFFER_LEN] = {0};
};
