#include "mbed.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "math.h"
#include "mbed_rpc.h"
#include<stdlib.h>
#define UINT14_MAX        16383
// FXOS8700CQ I2C address
#define FXOS8700CQ_SLAVE_ADDR0 (0x1E<<1) // with pins SA0=0, SA1=0
#define FXOS8700CQ_SLAVE_ADDR1 (0x1D<<1) // with pins SA0=1, SA1=0
#define FXOS8700CQ_SLAVE_ADDR2 (0x1C<<1) // with pins SA0=0, SA1=1
#define FXOS8700CQ_SLAVE_ADDR3 (0x1F<<1) // with pins SA0=1, SA1=1
// FXOS8700CQ internal register addresses
#define FXOS8700Q_STATUS 0x00
#define FXOS8700Q_OUT_X_MSB 0x01
#define FXOS8700Q_OUT_Y_MSB 0x03
#define FXOS8700Q_OUT_Z_MSB 0x05
#define FXOS8700Q_M_OUT_X_MSB 0x33
#define FXOS8700Q_M_OUT_Y_MSB 0x35
#define FXOS8700Q_M_OUT_Z_MSB 0x37
#define FXOS8700Q_WHOAMI 0x0D
#define FXOS8700Q_XYZ_DATA_CFG 0x0E
#define FXOS8700Q_CTRL_REG1 0x2A
#define FXOS8700Q_M_CTRL_REG1 0x5B
#define FXOS8700Q_M_CTRL_REG2 0x5C
#define FXOS8700Q_WHOAMI_VAL 0xC7
I2C i2c( PTD9,PTD8);
RawSerial pc(USBTX, USBRX);
RawSerial xbee(D12, D11);
InterruptIn sw2(SW2);
RpcDigitalOut myled1(LED1,"myled1");
void LEDControl(Arguments *in, Reply *out);
RPCFunction rpcLED(&LEDControl, "LEDControl");
void xbee_rx_interrupt(void);
void xbee_rx(void);
void reply_messange(char *xbee_reply, char *messange);
void check_addr(char *xbee_reply, char *messenger);
double x, y;
int m_addr = FXOS8700CQ_SLAVE_ADDR1;
void FXOS8700CQ_readRegs(int addr, uint8_t * data, int len);
void FXOS8700CQ_writeRegs(uint8_t * data, int len);
void logger();
void log_acc();
void log_logger();
float Velocity[100] = {0.0};
float t[3]; 
float AccData[103][3];
float v[101];
int j = 0;


EventQueue logqueue;
EventQueue xbee_queue(32 * EVENTS_EVENT_SIZE);
Thread log_thread(osPriorityLow);
Thread xbee_thread(osPriorityLow);

int main(){
    pc.baud(9600);

    log_thread.start(callback(&logqueue, &EventQueue::dispatch_forever));
    char xbee_reply[4];

    xbee.baud(9600);
    xbee.printf("+++");
    xbee_reply[0] = xbee.getc();
    xbee_reply[1] = xbee.getc();
    if(xbee_reply[0] == 'O' && xbee_reply[1] == 'K'){
    pc.printf("enter AT mode.\r\n");
    xbee_reply[0] = '\0';
    xbee_reply[1] = '\0';
    }

    xbee.printf("ATMY <REMOTE_MY>\r\n");
    reply_messange(xbee_reply, "setting MY : <REMOTE_MY>");
    xbee.printf("ATDL <REMOTE_DL>\r\n");
    reply_messange(xbee_reply, "setting DL : <REMOTE_DL>");
    xbee.printf("ATID <PAN_ID>\r\n");
    reply_messange(xbee_reply, "setting PAN ID : <PAN_ID>");
    xbee.printf("ATWR\r\n");
    reply_messange(xbee_reply, "write config");
    xbee.printf("ATMY\r\n");
    check_addr(xbee_reply, "MY");
    xbee.printf("ATDL\r\n");
    check_addr(xbee_reply, "DL");
    xbee.printf("ATCN\r\n");
    reply_messange(xbee_reply, "exit AT mode");
    xbee.getc();
    pc.printf("start\r\n");

    xbee_thread.start(callback(&xbee_queue, &EventQueue::dispatch_forever));
    xbee.attach(xbee_rx_interrupt, Serial::RxIrq);
    Ticker xbeeTicker;
    xbeeTicker.attach(xbee_queue.event(&log_acc), 0.1f);
    sw2.fall(&log_logger);   // press sw2 to start measure and store the horizontal velocity

    while(1){wait(1);};
}

void log_logger(){
   logqueue.call(logger);
}

void logger(){
   Timer timer_wait;
   timer_wait.start();
   for (int i = 0; i < 100; i++){
      Velocity[i] = 9.8*0.1*sqrt(t[0]*t[0] + t[1]*t[1])*100;
      while(1){
        if (timer_wait.read() > 0.1){
        timer_wait.reset();
        break;
        }
      }
    }
}

void log_acc() {

   uint8_t who_am_i, data[2], res[6];
   int16_t acc16;

   // Enable the FXOS8700Q
   FXOS8700CQ_readRegs( FXOS8700Q_CTRL_REG1, &data[1], 1);
   data[1] |= 0x01;
   data[0] = FXOS8700Q_CTRL_REG1;
   FXOS8700CQ_writeRegs(data, 2);

   FXOS8700CQ_readRegs(FXOS8700Q_WHOAMI, &who_am_i, 1);

      FXOS8700CQ_readRegs(FXOS8700Q_OUT_X_MSB, res, 6);

      acc16 = (res[0] << 6) | (res[1] >> 2);
      if (acc16 > UINT14_MAX/2)
         acc16 -= UINT14_MAX;
      t[0] = ((float)acc16) / 4096.0f;

      acc16 = (res[2] << 6) | (res[3] >> 2);
      if (acc16 > UINT14_MAX/2)
         acc16 -= UINT14_MAX;
      t[1] = ((float)acc16) / 4096.0f;

      acc16 = (res[4] << 6) | (res[5] >> 2);
      if (acc16 > UINT14_MAX/2)
         acc16 -= UINT14_MAX;
      t[2] = ((float)acc16) / 4096.0f;
}


void FXOS8700CQ_readRegs(int addr, uint8_t * data, int len) {
   char t = addr;
   i2c.write(m_addr, &t, 1, true);
   i2c.read(m_addr, (char *)data, len);
}


void FXOS8700CQ_writeRegs(uint8_t * data, int len) {
   i2c.write(m_addr, (char *)data, len);
}

void xbee_rx_interrupt(void)

{

  xbee.attach(NULL, Serial::RxIrq); // detach interrupt

  xbee_queue.call(&xbee_rx);

}


void xbee_rx(void)

{

      static int i = 0;
      char buf[100] = {0};

      char outbuf[100] = {0};

      while(xbee.readable()){

         char c = xbee.getc();

         if(c!='\r' && c!='\n'){
               buf[i] = c;
               i++;
               buf[i] = '\0';
         }
         else{
               i = 0;
         }
         v[j] = strof(buf, NULL);
         RPC::call(buf, outbuf);


         pc.printf("%1.4f\r\n", Velocity[j]);  //send velocity data to mqtt publisher

         j++;

         if (j == 100){
               j = 0;
         }

    }

    xbee.attach(xbee_rx_interrupt, Serial::RxIrq); // reattach interrupt

}


void reply_messange(char *xbee_reply, char *messange){

  xbee_reply[0] = xbee.getc();

  xbee_reply[1] = xbee.getc();

  xbee_reply[2] = xbee.getc();

  if(xbee_reply[1] == 'O' && xbee_reply[2] == 'K'){

    pc.printf("%s\r\n", messange);

    xbee_reply[0] = '\0';

    xbee_reply[1] = '\0';

    xbee_reply[2] = '\0';

  }

}


void check_addr(char *xbee_reply, char *messenger){

  xbee_reply[0] = xbee.getc();

  xbee_reply[1] = xbee.getc();

  xbee_reply[2] = xbee.getc();

  xbee_reply[3] = xbee.getc();

  pc.printf("%s = %c%c%c\r\n", messenger, xbee_reply[1], xbee_reply[2], xbee_reply[3]);

  xbee_reply[0] = '\0';

  xbee_reply[1] = '\0';

  xbee_reply[2] = '\0';

  xbee_reply[3] = '\0';

}

