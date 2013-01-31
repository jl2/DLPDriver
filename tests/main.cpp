#include <stdio.h>
#include <stdlib.h>

#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>

#include <chrono>
#include <thread>
/* #include "dlp2232pbg.h" */
/*
  Appends a checksum to data in position pos
  Returns the calculated check sum
*/
unsigned char append_checksum(unsigned char *data, int pos) {
    int i;
    unsigned char checksum = 0;
  
    for (i=0;i<pos;++i) {
        checksum ^= data[i];
    }
    data[pos] = checksum;
    return checksum;
}

/*
  Writes out a series of bytes in human readable form
*/
void printHex(unsigned char *data, int len) {
    int i=0;
    for (i=0;i<len; ++i) {
        printf("%x ", data[i]);
    }
    printf("\n");
}

int main() {
    unsigned char outputBuffer[256];
    unsigned char inputBuffer[256];
    ssize_t werr;
    // ssize_t rerr;
  
    int fd = open("/dev/ttyUSB0", O_RDWR);
    // int fd = open("/dev/ttyUSB0", O_RDWR);

    if (fd == -1) {
        printf("Could not open /dev/ttyUSB0\n");
        exit(1);
    }

    std::chrono::milliseconds dura( 500 );
    
    outputBuffer[0] = 0x3;
    outputBuffer[1] = 0x5A;
    outputBuffer[2] = 0xaa;
    append_checksum(outputBuffer, 3);
    werr = write(fd,
                 outputBuffer,
                 4);
    
    outputBuffer[0] = 0x4;
    outputBuffer[1] = 0x59;
    outputBuffer[2] = 0x1;
    outputBuffer[3] = 0x1;
    outputBuffer[4] = 50;
    append_checksum(outputBuffer, 5);
    werr = write(fd,
                 outputBuffer,
                 6);


    // rerr = read(fd,
    //             inputBuffer,
    //             1);

    outputBuffer[0] = 0x3;
    outputBuffer[1] = 0x5A;
    outputBuffer[2] = 0x0;
    append_checksum(outputBuffer, 3);
    werr = write(fd,
                 outputBuffer,
                 4);

    printf("Read: %x\n", (int)(inputBuffer[0]));
    // std::this_thread::sleep_for( dura );
        
        
    /* while (1) { */
    /* } */
    // printf("Write done...\n werr = %d\n", (int)werr);
  
    // if (werr!=4) {
    //     printf("Didn't write 3 bytes to the device!\n");
    //     close(fd);
    //     exit(1);
    // } else {
    //     printf("Wrote 3 bytes to the device.  Attempting to read result.\n");
    // }
    // fflush(stdout);
    /* rerr = read(fd, */
    /*             inputBuffer, */
    /*             1); */
  
    /* if (rerr!=1) { */
    /*     printf("Was expecting 6 bytes, got %d!\n", rerr); */
    /* } */
    /* printHex(inputBuffer, rerr); */
  
    close(fd);


    exit(0);
}
