/*
 * Proof of concept HST flash dump.
 * not fast because runs word at a time.
 * 
reads from:- 
  uint32_t addr = 0xa8240000;
  size_t size = 1024 * 16;

see:
  https://ai-thinker-open.github.io/GPRS_C_SDK_DOC/zh/more/flash_map.html
  
  0xa8 24 0000 < app image (1M)
  0xa8 3F E000 < factory cfg (8k) <- IMEI is in here
 */

#include <cstdlib>
#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

static int serial_port;
void dump(uint8_t *buffer, size_t size) {
  printf("Buffer:\n");
  for (int i = 0; i < size; i += 32) {
    for (int c = 0; c < 32; c++) {
      printf("%02x", buffer[i + c]);
    }
    printf(" ");
    for (int c = 0; c < 32; c++) {
      if (buffer[i + c] > 0x30 && buffer[i + c] < 0x7f)
        printf("%c", buffer[i + c]);
      else
        printf(".");
    }
    printf("\n");
  }
}

void initSerial() {
  struct termios tty;
  memset(&tty, 0, sizeof(tty));

  cfsetispeed(&tty, B921600);
  cfsetospeed(&tty, B921600);

  tty.c_cflag = CS8 | CLOCAL | CREAD;
  tty.c_iflag = IGNPAR | IXON | IXOFF | IXANY;

  tty.c_oflag = 0;
  tty.c_lflag = 0;

  tty.c_cc[VMIN] = 1;
  tty.c_cc[VTIME] = 0;
  tty.c_cc[VSTART] = 0x11;
  tty.c_cc[VSTOP] = 0x13;

  tcflush(serial_port, TCIFLUSH);
  tcsetattr(serial_port, TCSANOW, &tty);
}

void toXON(uint8_t *in, uint8_t *out, int &len) {
  int o = 0;
  for (int i = 0; i < len; i++) {
    if (in[i] == 0x11 || in[i] == 0x13 || in[i] == 0x5c) {
      out[o++] = 0x5c;
      out[o++] = in[i] ^ 0xff;
    } else
      out[o++] = in[i];
  }
  len = o;
}

void deXON(uint8_t *in, uint8_t *out, int &len) {
  int o = 0;
  for (int i = 0; i < len; i++) {
    if (in[i] == 0x5c) {
      out[o++] = in[++i] ^ 0xff;
    } else
      out[o++] = in[i];
  }
  len = o;
}

/*  packet      counter
 *      sz      |  / bytes \
 *       \      | |         |  /chksum
 * AD 00 06 FF 67 60 F1 E0 99 70   <read
 */

/* not sure if this works yet */
bool setWord(uint32_t addr, uint32_t word) {
  uint8_t req[14];
  uint8_t req2[28];
  req[0] = 0xad;
  req[1] = 0x00;
  req[2] = 0x0a;
  req[3] = 0xff;
  req[4] = 0x82;
  *(uint32_t *)(&req[5]) = addr;
  *(uint32_t *)(&req[9]) = word;

  uint8_t ck = 0;
  for (int i = 0; i < 10; ++i)
    ck ^= req[i + 3];
  req[13] = ck; // xor checksum
  int len = 14;
  toXON(req, req2, len); // encode

  for (int i = 0; i < len; ++i) {
    printf("%02x ", req2[i]);
  }

  write(serial_port, req2, len);
  tcdrain(serial_port); // Wait for sent

  return true;
}

bool getWord(uint32_t addr, uint8_t *buffer) {
  static uint8_t count = 0;

  uint8_t req[11];
  uint8_t req2[22];

  req[0] = 0xad;
  req[1] = 0x00;
  req[2] = 0x07;
  req[3] = 0xff;
  req[4] = 0x02;
  *(uint32_t *)(&req[5]) = addr;

  ++count;

  req[9] = count; // Request count

  uint8_t ck = 0;
  for (int i = 0; i < 7; ++i)
    ck ^= req[i + 3];
  req[10] = ck; // xor checksum
  static int n = 0;

  int len = 11;
  toXON(req, req2, len);

  write(serial_port, req2, len);
  tcdrain(serial_port); // Wait for sent
  usleep(700);

  // Wait and read.
  uint8_t block[16];
  uint8_t buf[20];
  int nread;
  nread = read(serial_port, buf + n, 16);
  if (nread > 0) {
    deXON(buf, block, nread);

    if (*(uint32_t *)(block) == 0xff0600ad) {
      uint8_t ck = 0;
      for (int i = 0; i < 6; ++i)
        ck ^= block[i + 3];
      if (ck != block[9]) {
        printf("Check mismatch: %02x %02x\n", ck, block[9]);
        return false;
      } else {
        *(uint32_t *)buffer = *(uint32_t *)(block + 5);
      }
    }
  }

  return true;
}

int main() {
  serial_port = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_SYNC);
  if (serial_port < 0) {
    fprintf(stderr, "Unable to open ttyUSB\n");
    _exit(-1);
  }
  initSerial();

  //  fcntl(serial_port, F_SETFL, FNDELAY);

  // Clear any startup info waiting.
  tcflush(serial_port, TCIOFLUSH);

  uint32_t addr = 0xa83fe000;
  size_t size = 1024 * 8;

  printf("\nDumping flash data from 0x%08x to 0x%08x\n", addr, addr + size);

  uint8_t *buffer = (uint8_t *)malloc(size);

  for (size_t word = 0; word < size / 4; word++) {

    if (!getWord(addr + word * 4, &buffer[word * 4])) {
      printf("Failure\n");
      return 0;
    };
  }

  FILE *f = fopen("dump.bin", "w+");
  size_t done = fwrite(buffer, size, 1, f);
  fclose(f);

  close(serial_port);
  return 0;
}
