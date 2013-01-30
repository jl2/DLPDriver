#include <stdbool.h>
#include <pthread.h>
#include <termios.h>
#include <signal.h>

// DLP Declarations ***************************************************
enum dlp_command{
        LineIn=0xA5,
        LineOut=0xA6, 
        GetBoardId=0xA7,
        ConfigureAtoDs=0xA8,
        StartAtoD=0xA9,
        EEPROM_Read=0xAA,
        EEPROM_Write=0xAB,
        StartDS18S20=0xAC,
        ReadDS18S20=0xAD,
        ReadDS18S20Id=0xBA,
        SearchDS18S20s=0xBB,
        StartMatchDS18S20=0xBC,
        ReadMatchDS18S20=0xBD,
        Reserved=0xAE,
        Loopback=0xAF,
        ReadPortA=0x55,
        WritePortA=0x56,
        ReadPortC=0x59,
        WritePortC=0x5A,
        ReadPortD=0x5B,
        WritePortD=0x5C,
	MavicaInit=0xC0,
	MavicaBytes=0xC1,
	MavicaCommand=0xC2
};

enum dlp_port{
        PortA=0x28,
        PortB=0x30,
        PortC=0x38,
        PortD=0x40,
        PortE=0x48
};

enum port_pin{
        Pin0=0x00,
        Pin1=0x01,
        Pin2=0x02,
        Pin3=0x03,
        Pin4=0x04,
        Pin5=0x05,
        Pin6=0x06,
        Pin7=0x07,
	All=0x255
};

enum analog_port{
        AN0=0,
        AN1=1,
        AN2=2,
        AN3=3,
        AN4=4
};

enum digital_pin_state{
        Low=0,
        High=1
};

// PIC16F877A clock frequency
// Valid combinations of clock frequency and divider settings
//   are checked in setup_d_to_a () function.
// DLP-2232PB-G and DLP-2232M-G have 20000000 clock crystals
// The maximum allowed clock frequency for the PIC16F877A is 20000000
const unsigned int PIC16F877A_clock_frequency = 20000000;  // 20000000 maximum

// PIC16F877A divider settings
// Valid values are 64, 32, 16, 8, 4, 2 and 0
//   depending on clock frequency.  0 means internal RC clock
const unsigned int PIC16F877A_divider = 32;  // Fastest allowed for 20 MHz clock frequency

// PIC16F877A analog port settings
// Note: DLP-2232PB-G uses AN6 and AN7 for SI/WUB and TXE# so they must be digital in that unit
//         Therefore values of 0x00, 0x01, and 0x08 are not valid for the DLP-2232PB-G
//
// Valid values of 0-15 given here are determined from the function parameters
//    enum analog_port_ref and int analog_ports (number of analog ports) as follows.
//
// analog_port_ref = anything, analog_ports = 0
// 0x06 - AN0, AN1, AN2, AN3, AN4, AN5, AN6, AN7 are digital;
// 0x07 - Same as 6
//
// analog_port_ref = Vdd_Vss, analog_ports = 1, 3, 5, 6, 8
// 0x0E - AN0 is analog;  AN1, AN2, AN3, AN4, AN5, AN6, AN7 are digital; Vref+ = Vdd, Vref- = Vss
// 0x04 - AN0, AN1, AN3 are analog; AN2, AN4, AN5, AN6, AN7 are digital; Vref+ = Vdd, Vref- = Vss
// 0x02 - AN0, AN1, AN2, AN3, AN4 are analog; AN5, AN6, AN7 are digital; Vref+ = Vdd, Vref- = Vss
// 0x09 - AN0, AN1, AN2, AN3, AN4, AN5 are analog; AN6, AN7 are digital; Vref+ = Vdd, Vref- = Vss
// 0x00 - AN0, AN1, AN2, AN3, AN4, AN5, AN6, AN7 are analog;             Vref+ = Vdd, Vref- = Vss
//
// analog_port_ref = AN3_Vss, analog_ports = 2, 4, 5, 7
// 0x05 - AN0, AN1 are analog; AN2, AN4, AN5, AN6, AN7 are digial;       Vref+ = AN3, Vref- = Vss
// 0x03 - AN0, AN1, AN2, AN4 are analog; AN5, AN6, AN7 are digital;      Vref+ = AN3, Vref- = Vss
// 0x0A - AN0, AN1, AN2, AN4, AN5 are analog; AN6, AN7 are digital;      Vref+ = AN3, Vref- = Vss
// 0x01 - AN0, AN1, AN2, AN4, AN5, AN6, AN7, are analog;                 Vref+ = AN3, Vref- = Vss
//
// analog_port_ref = AN3_AN2, analog_ports = 1, 2, 3, 4, 6
// 0x0F - AN0 is analog;  AN1, AN4, AN5, AN6, AN7 are digital;           Vref+ = AN3, Vref- = AN2
// 0x0D - AN0, AN1 are analog; AN4, AN5, AN6, AN7 are digital;           Vref+ = AN3, Vref- = AN2
// 0x0C - AN0, AN1, AN4 are analog; AN5, AN6, AN7 are digial;            Vref+ = AN3, Vref- = AN2
// 0x0B - AN0, AN1, AN4, AN5 are analog; AN6, AN7 are digital;           Vref+ = AN3, Vref- = AN2
// 0x08 - AN0, AN1, AN4, AN5, AN6, AN7 are analog;                       Vref+ = AN3, Vref- = AN2

enum analog_port_ref {
        Vdd_Vss = 0x10,
        AN3_Vss = 0x20,
        AN3_AN2 = 0x30
};

enum polarity{
        Pos=0,
        Neg=1
};

int dlp2232pbg = true;  // Set when identification string returns 2232PB
int dlp_initialized = false;

int DAs_configured = false;
int port_A_is_all_digital = false;
int analog_port[8];
int thermometer_port[40];
int verbose = false;

#define BAUDRATE B230400

// The number of points needed depends on the block_count and block_size.
// For best speed use small block_count and large block_size
// Block size loop is .5 uS faster.
#define MAX_POINTS_PER_SCAN 65536
unsigned char inbuf[MAX_POINTS_PER_SCAN], outbuf[16];

void signal_handler_IO(int status);  // I can only count on getting at least one signal when a read is ready.
struct sigaction saio;
volatile int signals=0;  // Minimum signal count.  Subsequent signals might be hidden when the handler is running
int fd = 0;

pthread_mutex_t dlp_command_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t verbose_mutex = PTHREAD_MUTEX_INITIALIZER;

void send_dlp_command();
int echo_byte(unsigned char c);
void check_id(char* required_id);
void identify();
void dlp_init();
void dlp_deinit();
void signal_handler_IO(int status);
void check_binary_transparency();
int get_burst_size(unsigned char block_count, unsigned char block_size); 
void setup_a_to_d(enum analog_port_ref refs, unsigned int analog_ports, unsigned int PIC16F877A_divider);
void get_voltage(enum analog_port port, unsigned char block_count, unsigned char block_size, unsigned char delay, double* V);
char portToLetter(enum dlp_port port);
char* stateToWord(enum digital_pin_state state);
void save_burst_data();
void show_burst_data();

enum digital_pin_state read_pin_state(enum dlp_port port, enum port_pin pin);
void set_pin_state(enum dlp_port port, enum port_pin pin, enum digital_pin_state state);
void* show_PIC16F877A_eeprom();
int read_PIC16F877A_eeprom(unsigned char address);
void write_PIC16F877A_eeprom(unsigned char address, unsigned char value);
void read_port(enum dlp_port port, unsigned char block_count, unsigned char block_size, unsigned char delay, int* state[], int channels);
void set_port(enum dlp_port port, unsigned char bits);

void dlp_init(char *device){
  sigset_t mask;
  struct termios newtio;

  // Setting up signal handler used by read() when doing asynchronous I/O
  saio.sa_handler = signal_handler_IO;
  sigemptyset(&mask);
  sigaddset(&mask, SIGIO);
  saio.sa_mask = mask;
  saio.sa_flags = SA_NODEFER;
  saio.sa_restorer = NULL;
  sigaction(SIGIO, &saio, NULL);

  if ((fd = open(device, O_RDWR )) < 0 )   // If O_ASYNC is set here, hangs read for lack of ownership
    error(-1, 0, "Could not open %s - Check DLP-2232PB-G USB connection and power jumpers.",device);

//  show_file_descriptor_flags(fd);     // At this point only O_RDWR is set
//  show_file_descriptor_owner_pid(fd); // and the owner process pid = 0

  fcntl(fd, F_SETOWN, getpid());  // Making this process own the file descriptor allows O_ASYNC to be set!!!
  fcntl(fd, F_SETFL, O_ASYNC );   // Use signals - See man 2 open for explanation

//  show_file_descriptor_flags(fd);     // Now O_ASYNC is set
//  show_file_descriptor_owner_pid(fd); // and the owner is this process

  bzero(&newtio, sizeof(newtio)); // start with a clean slate
  newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
  newtio.c_iflag = IGNPAR;
  newtio.c_oflag = 0;
  newtio.c_cc[VTIME]  = 0;  // inter-character timer (unused in Linux)
  newtio.c_cc[VMIN]  = 1;  // blocking read until 1 character arrives
  cfmakeraw(&newtio);
  cfsetospeed(&newtio, BAUDRATE); // The speed settings don't seem to really matter.
  cfsetispeed(&newtio, BAUDRATE);
  tcflush(fd, TCIFLUSH);          // Make sure channel is cleared
  tcsetattr(fd, TCSANOW, &newtio);

// Just in case I have left something out that is set by default, here are all settings:
// # stty -a -F /dev/ttyUSB1
// speed 230400 baud; rows 0; columns 0; line = 0;
// intr = <undef>; quit = <undef>; erase = <undef>; kill = <undef>; eof = <undef>; eol = <undef>; eol2 = <undef>; swtch = <undef>;
// start = <undef>; stop = <undef>; susp = <undef>; rprnt = <undef>; werase = <undef>; lnext = <undef>; flush = <undef>;
// min = 1; time = 0;
// -parenb -parodd cs8 -hupcl -cstopb cread clocal crtscts
// -ignbrk -brkint ignpar -parmrk -inpck -istrip -inlcr -igncr -icrnl -ixon -ixoff -iuclc -ixany -imaxbel -iutf8
// -opost -olcuc -ocrnl -onlcr -onocr -onlret -ofill -ofdel nl0 cr0 tab0 bs0 vt0 ff0
// -isig -icanon -iexten -echo -echoe -echok -echonl -noflsh -xcase -tostop -echoprt -echoctl -echoke
//  check_binary_transparency();
  dlp_initialized = true;
}

void dlp_deinit(){
  close(fd);
//    printf("%d signals received\n",signals);
}

void send_dlp_command(char *device){
  int i,k;
  int res;
  int nbr_bytes_expected;
  static int test = 0;
  int burst_size;

  if (!dlp_initialized)
    dlp_init(device);
  // Double-check that this routine is not already being used by a thread
  test++;
  if (test != 1){
    printf("Entry and exit in send_dlp_command are not matched!!! (test = %d)\n",test);
    exit(-1);
  }

  // Prepare command string to send to DLP-2232-PB-G
  burst_size = 0;
  switch (outbuf[1]){
    case LineOut:
    case ConfigureAtoDs:
    case EEPROM_Write:
      outbuf[0] = 3;
      nbr_bytes_expected = 0;
      break;
    case WritePortA:
    case WritePortC:
    case WritePortD:
      outbuf[0] = 2;
      nbr_bytes_expected = 0;
      break;
    case MavicaInit:
      outbuf[0] = 1;
      nbr_bytes_expected = 1;
      break;
    case MavicaBytes:
      outbuf[0] = 2;
      if (!outbuf[2])
	nbr_bytes_expected = 8*256;
      else
      nbr_bytes_expected = 8*outbuf[2];
      break;
    case MavicaCommand:
      outbuf[0] = 3;
      if (!outbuf[2])
	nbr_bytes_expected = 5*256;
      else
      nbr_bytes_expected = 5*outbuf[2];
      break;
    case LineIn:
    case EEPROM_Read:
    case StartDS18S20:
    case Loopback:
      outbuf[0] = 2;
      nbr_bytes_expected = 1;
      break;
    case ReadPortA:
    case ReadPortC:
    case ReadPortD:
      outbuf[0] = 4;
      burst_size = get_burst_size(outbuf[2], outbuf[3]);
      nbr_bytes_expected = 1;
      break;
    case StartAtoD:
      outbuf[0] = 5;
      burst_size = get_burst_size(outbuf[3], outbuf[4]);
      nbr_bytes_expected = 2;
      break;
    case GetBoardId:
      outbuf[0] = 1;
      nbr_bytes_expected = 6;
      break;
    case ReadDS18S20Id:
      outbuf[0] = 2;
      nbr_bytes_expected = 8;
      break;
    case ReadDS18S20:
      outbuf[0] = 2;
      nbr_bytes_expected = 9;
      break;
    case SearchDS18S20s:
      outbuf[0] = 2;
      nbr_bytes_expected = -9;  // -9 signifies unknown number of 9-byte groups coming back
      break;
    case ReadMatchDS18S20:
      outbuf[0] = 10;
      nbr_bytes_expected = 9;
      break;
    case StartMatchDS18S20:
      outbuf[0] = 10;
      nbr_bytes_expected = 1;
      break;
    default:
      printf("Illegal DLP instruction code: %02X\n", outbuf[1]);
      exit(-1);
  }
  k = outbuf[0]+1;
  outbuf[k] = 0;
  for (i=0; i < k; i++)
    outbuf[k] ^= outbuf[i];

  // Send the command string
  res = write(fd, outbuf, k+1);
  if (res != k+1){
    printf("Unable to write command %02X to DLP-2232PB-G\n", outbuf[1]);
    exit(-1);
  }

  // Get the response string from the DLP-2232PB-G
  {
    int res;
    int i, j, done, group_size;
    unsigned char status_byte;
    fd_set rfds;
    struct timeval tv0 = {5,0};
    struct timeval tv;
    int retval;
    FD_ZERO(&rfds);
    FD_SET(fd,&rfds);
    tv.tv_sec = tv0.tv_sec;
    tv.tv_usec = tv0.tv_usec;

    int timed_read_USB(unsigned char * bytes, int nbr_bytes_expected){
//  select() will occasionally return an "interrupted system call" error and should be retried
//  This while and the check on EINTR solve this problem.
      while (1) {
        retval = select(fd+1, &rfds, NULL, NULL, &tv);
        if (retval == -1){
          if (errno == EINTR)
            continue;
          perror("select()");
          exit(-1);
        }
        else
          break;
      }
      if (retval){
        if ((res = read(fd, bytes, nbr_bytes_expected)) >= 0){
          return res;
        }
      }
      else {
        printf("Timeout on Camera connection after %lu seconds.\n",(unsigned long)tv0.tv_sec);
        exit(-1);
      }
      return 0;
    }

    if ((res = timed_read_USB(&status_byte, 1)) < 0)
      error(-1, 0, "Error reading status byte");

    switch (status_byte) {
      case 0x55:
        break;
      case 0xAA:
        printf("Parity error in command\n");
        break;
        return;
//        return(status_byte);
      case 0xAB:
        printf("Aborting command...\n");
        break;
        return;
//        return(status_byte);
      case 0xFF:
        printf("Communication error...\n");
        break;
        return;
//        return(status_byte);
      default:
        printf("Mystery status byte: %02X\n", status_byte);
        return;
//      return(status_byte);
        ;
    }
    if (nbr_bytes_expected){
      i = 0;
      if (nbr_bytes_expected < 0){
        done = false;
        nbr_bytes_expected = -nbr_bytes_expected;
        group_size = nbr_bytes_expected;
        while (! done){
          while (i < nbr_bytes_expected){
            i += timed_read_USB(&inbuf[i], nbr_bytes_expected - i); 
          }
          nbr_bytes_expected += group_size;
          if (inbuf[i-1] == 0x00 || inbuf[i-1] == 0x02 || inbuf[i-1] == 0x08){
            done = true;
            for (j = 1; j < group_size; j++){
              if (inbuf[i - 1 - j] != inbuf[i-1]){
                done = false;
                break;
              }
            }
          }
        }
      } else if (burst_size == 0 || burst_size == 1){
        while (i < nbr_bytes_expected){
          i += timed_read_USB(&inbuf[i], nbr_bytes_expected - i);
        }
      }
      else{
        while (i < nbr_bytes_expected*burst_size){
          i += timed_read_USB(&inbuf[i], nbr_bytes_expected*burst_size - i);
        }
      }
    }
//  if (verbose && burst_size !=0)
//    show_burst_data();
//  return(0);
//}
  }
  test--;
}

int echo_byte(unsigned char c){
  pthread_mutex_lock(&dlp_command_mutex);
  outbuf[1] = 0xAF;
  outbuf[2] = c;
  send_dlp_command();
  if (inbuf[0] != c){
    printf("Bad echo: Sent %02X, but received %02X\n", c, inbuf[0]);
    exit(-1);
  }
  pthread_mutex_unlock(&dlp_command_mutex);

  pthread_mutex_lock(&verbose_mutex);
  if (verbose)
    printf("0x%02X echoed correctly\n", c);
  pthread_mutex_unlock(&verbose_mutex);
  return c;
}

void check_id(char* required_id){
  char id[7];
  pthread_mutex_lock(&dlp_command_mutex);
  outbuf[1] = 0xA7;
  send_dlp_command();
  strncpy(id, (char*)inbuf, 6);
  id[6] = 0;
  pthread_mutex_unlock(&dlp_command_mutex);
  if (strncmp(required_id, id, 6)){
    printf("DLP unit id \"%s\" does not match \"%s\"\n", id, required_id);
    exit(-1);
  }
}

void identify(){
  char *dlp2232pbgId = "2232PB\0";
  char id[7];
  pthread_mutex_lock(&dlp_command_mutex);
  outbuf[1] = 0xA7;
  send_dlp_command();
  strncpy(id, (char*)inbuf, 6);
  id[6] = 0;
  if (strncmp(dlp2232pbgId, id, 6))
    dlp2232pbg = true;
  pthread_mutex_unlock(&dlp_command_mutex);
  pthread_mutex_lock(&verbose_mutex);
  if (verbose)
    printf("Unit returned identification of \"%s\"\n", id);
  pthread_mutex_unlock(&verbose_mutex);
}


// This function checks that all 256 possible bytes echo properly from the DLP-2232PB-G
// Incorrect termios settings can cause some bytes to be altered.
void check_binary_transparency(){
  int i;
  for (i=0; i<256; i++){
    echo_byte(i);
    if (inbuf[0] == i)
      printf(".");
    else
      printf("<%02X>",i);
  }
  printf("\n");
}

void signal_handler_IO(int status){
  signals++;
}

char portToLetter(enum dlp_port port){
  return ((port - PortA) >> 3) + 'A';
}

char* stateToWord(enum digital_pin_state state){
  if (state == Low)
    return "low";
  else
    return "high";
}

int get_burst_size(unsigned char block_count, unsigned char block_size){ 
  int burst_size;
  if (block_size == 0)
    burst_size = 256;
  else 
    burst_size = block_size;

  if (block_count == 0)
    burst_size *= 256;
  else
    burst_size *= block_count;
  return burst_size;
}

//void save_burst_data(){
//  FILE *f;
//  int i;
//  f = fopen("data.out","w");
//  for (i=0; i< burst_size; i++)
//    fprintf(f, "%05d: %5.2f\n",t[i],V[i]);
//  fclose(f);
//}

//void show_burst_data(){
//  int i,j;
//  for (i = 0; i < burst_size; i++){
//    if (type == Analog)
//      printf("%5d: %6.1f  ", i, V[i]);
//    else{
//      printf("%5d: ", i);
//      for (j = 7; j >= 0; j--){
//        printf("%c", '0' +((inbuf[i] & (1 << j)) !=0));
//        if (j == 4)
//          printf(" ");
//      }
//      printf("  ");
//    }
//    if (i%10 == 9)
//      printf("\n");
//  }
//  printf("\n");
//}
  
//  Based on Chapter 11 of the PIC16F87XA Data Sheet
void setup_a_to_d(enum analog_port_ref refs, unsigned int analog_ports, unsigned int PIC16F877A_divider){
  int i;
  char *ref_string;
  char *port_string;
  int max_analog_ports;

  if (dlp2232pbg)
    max_analog_ports = 6;
  else
    max_analog_ports = 8;

  pthread_mutex_lock(&dlp_command_mutex);
  outbuf[1] = ConfigureAtoDs; 
  if (analog_ports > max_analog_ports ){
    printf("%d analog ports specified, but maximum is %d.\n", analog_ports, max_analog_ports);
    exit(-1);
  }
  switch (refs){
    case Vdd_Vss:
      ref_string = "Vdd and Vss";
      break;
    case AN3_Vss:
      ref_string = "pin 3 and Vss";
      break;
    case AN3_AN2:
      ref_string = "pin 3 and pin 2";
      break;
    default:
      printf("Bad analog port reference value: %d\n",refs);
      exit(-1);
  }

  port_A_is_all_digital = false;
  for (i=0; i<8; i++)
    analog_port[i] = false;
  switch (refs + analog_ports){
    case 0x10:
    case 0x20:
    case 0x30:
      outbuf[2] = 0x06;
      port_string = "All pins on Port A are digital.";
      port_A_is_all_digital = true;
      break;
    case 0x11:
      outbuf[2] = 0x0E;
      port_string = "pin 0 is analog, pins 1-7 are digital";
      analog_port[0] = true;
      break;
    case 0x13:
      outbuf[2] = 0x04;
      port_string = "pins 0-2 are analog, pins 3-7 are digital";
      for (i=0; i<3; i++)
        analog_port[i] = true;
      break;
    case 0x15:
      outbuf[2] = 0x02;
      port_string = "pins 0-4 are analog, pins 5-7 are digital";
      for (i=0; i<5; i++)
        analog_port[i] = true;
      break;
    case 0x16:
      outbuf[2] = 0x09;
      port_string = "pins 0-5 are analog, pins 6 and 7 are digital";
      for (i=0; i<6; i++)
        analog_port[i] = true;
      break;
    case 0x18:      
      if (!dlp2232pbg) {
        outbuf[2] = 0x00;
        port_string = "All 8 pins are analog";
        for (i=0; i<8; i++)
          analog_port[i] = true;
      }
      break;
    case 0x22:
      outbuf[2] = 0x05;
      port_string = "pins 0 and 1 are analog, pins 2 and 4-7 are digital";
      analog_port[0] = true;
      analog_port[1] = true;
      analog_port[3] = true;
      break;
    case 0x24:
      outbuf[2] = 0x03;
      port_string = "pins 0-2 and 4 are analog, pins 5-7 are digital";
      for (i=0; i<5; i++)
        analog_port[i] = true;
      break;
    case 0x25:
      outbuf[2] = 0x0A;
      port_string = "pins 0-2 and 4-5 are analog, pins 6-7 are digital"; 
      for (i=0; i<6; i++)
        analog_port[i] = true;
      break;
    case 0x27:
      if (!dlp2232pbg) {
        outbuf[2] = 0x01;
        port_string = "pins 0-2 and 4-7 are analog";
        for (i=0; i<8; i++)
          analog_port[i] = true;
      }
      break;
    case 0x31:
      outbuf[2] = 0x0F;
      port_string = "pin 0 is analog, pins 1 and 4-7 are digital";
      analog_port[0] = true;
      analog_port[2] = true;
      analog_port[3] = true;
      break;
    case 0x32:
      outbuf[2] = 0x0D;
      port_string = "pins 0 and 1 are analog, pins 4-7 are digital";
      for (i=0; i<4; i++)
        analog_port[i] = true;
      break;
    case 0x33:
      outbuf[2] = 0x0C;
      port_string = "pins 0, 1, and 4 are analog, pins 5-7 are digital";
      for (i=0; i<5; i++)
        analog_port[i] = true;
      break;
    case 0x34:
      outbuf[2] = 0x0B;
      port_string = "pins 0, 1, 4, and 5 are analog, pins 6 and 7 are digital";
      for (i=0; i<6; i++)
        analog_port[i] = true;
      break;
    case 0x36:
      if (!dlp2232pbg) {
        outbuf[2] = 0x08;
        port_string = "pins 0, 1, and 4-7 are analog";
        for (i=0; i<8; i++)
          analog_port[i] = true;
      }
      break;
    default:
      printf("Unavailable combination of analog references (%s) and number of ports (%d)\n", ref_string, analog_ports);
      exit(-1);
  }
  if (verbose){
    if (outbuf[2] == 0x06)
      printf("%s\n", port_string);
    else
      printf("Port A %s, analog references from %s.\n",port_string, ref_string);
  }

  switch (PIC16F877A_divider){
    case 64:
      outbuf[2] |= 0xC0;
      outbuf[3] = 0x81;
      if (PIC16F877A_clock_frequency <= 20000000)
        break;
    case 32:
      outbuf[2] |= 0x80;
      outbuf[3] = 0x81;
      if (PIC16F877A_clock_frequency <= 20000000)
        break;
    case 16:
      outbuf[2] |= 0xC0;
      outbuf[3] = 0x41;
      if (PIC16F877A_clock_frequency <= 10000000)
        break;
    case 8:
      outbuf[2] |= 0x80;
      outbuf[3] = 0x41;
      if (PIC16F877A_clock_frequency <= 5000000)
        break;
    case 4:
      outbuf[2] |= 0xC0;
      outbuf[3] = 0x01;
      if (PIC16F877A_clock_frequency <= 2500000)
        break;
    case 2:
      outbuf[2] |= 0x80;
      outbuf[3] = 0x01;
      if (PIC16F877A_clock_frequency <= 1250000)
        break;
    case 0:  // Internal RC Clock - only recommended for sleep operation
            //                     if clock frequency > 1000000
      outbuf[2] |= 0x80;
      outbuf[3] = 0xC1;
    default:
      printf("PIC16F877A_divider setting of %d is too high for PIC16F877A_clock_frequency of %d\n",
      PIC16F877A_divider, PIC16F877A_clock_frequency);
      exit(-1);  
  }
  send_dlp_command();
  DAs_configured = true;
  pthread_mutex_unlock(&dlp_command_mutex);
  pthread_mutex_lock(&verbose_mutex);
  if (verbose){
    printf("D/A Converters set up with %d analog ports using %s as references and ", analog_ports, ref_string);
    if (PIC16F877A_divider)
      printf("a clock divider of %d.\n",PIC16F877A_divider);
    else
      printf("an RC clock.\n");
  }
  pthread_mutex_unlock(&verbose_mutex);
}

void get_voltage(enum analog_port port, unsigned char block_count, unsigned char block_size, unsigned char delay, double* V){
  int i;
  if (analog_port[port] != true){
    printf("Analog port %d is currently configured as a digital port.\n", port);
    exit(-1);
  }
  pthread_mutex_lock(&dlp_command_mutex);
  outbuf[1] = StartAtoD;
  if (DAs_configured == false){
    printf("Must configure D/As before reading voltages\n");
    exit(-4);
  }
  outbuf[2] = port;
  outbuf[3] = block_count; // nbr of blocks
  outbuf[4] = block_size;  // block size 0 for 256
  outbuf[5] = delay;
  send_dlp_command();
  for (i=0; i < 2*get_burst_size(block_count, block_size); i+=2){
    V[i/2]= ((double)inbuf[i+1]*256+inbuf[i])/256*5020/3.996;
  }
  pthread_mutex_unlock(&dlp_command_mutex);
  pthread_mutex_lock(&verbose_mutex);
  if (verbose)
//    printf("Analog voltages from Port A, pin %d:\n", port);
  pthread_mutex_unlock(&verbose_mutex);
}

void printPortPin(enum dlp_port port, enum port_pin pin){
  unsigned char outputString[15]="Port x, Pin x ";
  outputString[5] = 'A' + ((port - PortA) >> 3);
  outputString[12] = '0'+pin;
  outputString[14] = 0x00;
  printf("%s", outputString);
}


enum digital_pin_state read_pin_state(enum dlp_port port, enum port_pin pin){
  unsigned char response;
  pthread_mutex_lock(&dlp_command_mutex);
  outbuf[1] = LineIn;
  outbuf[2] = (unsigned char)(port + pin);
  send_dlp_command();
  response = inbuf[0];
  pthread_mutex_unlock(&dlp_command_mutex);
  pthread_mutex_lock(&verbose_mutex);
  if (verbose){
    printf("Pin %d of port %c is ", pin, portToLetter(port));
    if (response == 0)
      printf("low\n");
    else
      printf("high\n");
  }
  pthread_mutex_unlock(&verbose_mutex);
  switch ((int)response){
    case 0:
      return Low;
    case 1:
      return High;
    default:
      printf("Pin %d of port %c returned an improper value: %02X\n", pin, portToLetter(port), response);
      exit(-1);
  }
}

void set_pin_state(enum dlp_port port, enum port_pin pin, enum digital_pin_state state){
  if (port == PortA && pin > Pin5){
    printf("Pins 6 and 7 of Port A are not available on the DLP2232PB-G.\n");
    exit(-1);
  }
  pthread_mutex_lock(&dlp_command_mutex);
  outbuf[1] = LineOut;
  outbuf[2] = (unsigned char)(port + pin);
  outbuf[3] = state;
  send_dlp_command();
  pthread_mutex_unlock(&dlp_command_mutex);
  pthread_mutex_lock(&verbose_mutex);
  if (verbose){
    printf("Pin %d of port %c has been set %s\n", pin, portToLetter(port), stateToWord(state));
    if (port == PortA && pin == Pin4)
      printf("Remember: This pin is an open-drain output, not TTL.\n");
  }
  pthread_mutex_unlock(&verbose_mutex);
}

void* show_PIC16F877A_eeprom(){
  int i;
  unsigned char response[256];
  pthread_mutex_lock(&dlp_command_mutex);
  outbuf[1] = EEPROM_Read;
  for (i=0; i < 256; i++){
    outbuf[2] = i;
    send_dlp_command();
    response[i] = inbuf[0];
  }
  pthread_mutex_unlock(&dlp_command_mutex);

  pthread_mutex_lock(&verbose_mutex);
  printf("Current PIC16F877A EEPROM data displayed with\n  addresses relative to EEPROM start at 0x2100:");
  for (i = 0; i < 256; i++) {
    if (i % 16 == 0)
      printf("\n%02X: ", i);
    printf("%02X ",response[i]);
  }
  printf("\n");
  pthread_mutex_unlock(&verbose_mutex);
  return NULL;
}

int read_PIC16F877A_eeprom(unsigned char address){
  unsigned char response;
  pthread_mutex_lock(&dlp_command_mutex);
  outbuf[1] = EEPROM_Read;
  outbuf[2] = address;
  send_dlp_command();
  response = inbuf[0];
  pthread_mutex_unlock(&dlp_command_mutex);
  pthread_mutex_lock(&verbose_mutex);
  if (verbose)
    printf("PIC16F877A EEPROM address %02X holds %02X\n", address, response);
  pthread_mutex_unlock(&verbose_mutex);
  return response;
}

void write_PIC16F877A_eeprom(unsigned char address, unsigned char value){
  pthread_mutex_lock(&dlp_command_mutex);
  outbuf[1] = EEPROM_Write;
  outbuf[2] = address;
  outbuf[3] = value;
  send_dlp_command();
  pthread_mutex_unlock(&dlp_command_mutex);
  pthread_mutex_lock(&verbose_mutex);
  if (verbose)
    printf("PIC16F877A EEPROM address %02X has been set to %02X\n", address, value);
  pthread_mutex_unlock(&verbose_mutex);
}

void read_port(enum dlp_port port, unsigned char block_count, unsigned char block_size, unsigned char delay, int* state[], int channels){
  int pin;
  int i;
  if (port == PortA && port_A_is_all_digital == false)
    setup_a_to_d(Vdd_Vss, 0, 32);
  pthread_mutex_lock(&dlp_command_mutex);
  switch (port){
    case PortA:
      outbuf[1] = ReadPortA;
      break;
    case PortC:
      outbuf[1] = ReadPortC;
      break;
    case PortD:
      outbuf[1] = ReadPortD;
      break;
    default:
      printf("read_port is only for Ports A, C, and D of the PIC16F877A\n");
      exit(-1);
  }
  outbuf[2] = block_count;
  outbuf[3] = block_size;
  outbuf[4] = delay;
  send_dlp_command();
  for (i = 0; i < get_burst_size(block_count, block_size); i++){
    for (pin = 0; pin < channels; pin++){
      state[i][pin] = inbuf[i] >> pin & 1;
    }
  }
  pthread_mutex_unlock(&dlp_command_mutex);
  pthread_mutex_lock(&verbose_mutex);
  if (verbose)
//    printf("Digital pin readings from Port %c:\n", portToLetter(port));
  pthread_mutex_unlock(&verbose_mutex);
}

void set_port(enum dlp_port port, unsigned char bits){
  pthread_mutex_lock(&dlp_command_mutex);
  switch (port){
    case PortA:
      if (port_A_is_all_digital == false)
        setup_a_to_d(Vdd_Vss, 0, 32);
      outbuf[1] = WritePortA;
      break;
    case PortC:
      outbuf[1] = WritePortC;
      break;
    case PortD:
      outbuf[1] = WritePortD;
      break;
    default:
      printf("write_port is only for Ports A, C, and D of the PIC16F877A\n");
      exit(-1);
  }
  outbuf[2] = bits;
  send_dlp_command();
  pthread_mutex_lock(&dlp_command_mutex);
  pthread_mutex_lock(&verbose_mutex);
  if (verbose)
    printf("Port %c pins have been set to %02X\n", portToLetter(port), bits);
  pthread_mutex_unlock(&verbose_mutex);
}

