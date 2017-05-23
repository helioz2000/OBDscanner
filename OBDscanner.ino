/*************************************************************************
 * OBD Scanner
 * 
 * Freematics OBDII UART connector:
 * Red=+5V, Black=GND, White=RX, Green=TX
 * Baudrate is 115200
 * 
 * 
 * Circuit:
 * Arduino Mega uses Serial 1: Green->19 White->18
 * 
 *************************************************************************/

#include <SoftwareSerial.h>
#include <U8x8lib.h>
#include "OBD2UART.h"
#include "debug.h"

// On Arduino Leonardo, Micro, MEGA or DUE, hardware serial can be used for output
// as OBD-II UART adapter uses to Serial1
// On Arduino UNO and those have no Serial1, we use software serial for output
// as OBD-II UART adapter uses to Serial
//SoftwareSerial mySerial(A2, A3);
#define mySerial Serial

// ** Debug
#define DEBUG         // comment out to disable debugging
#ifdef DEBUG
#define DEBUG_LEVEL_DEFAULT L_INFO;
debugLevels currentDebugLevel;
#endif

// ** OLED
/* Constructor */
//U8X8_SSD1306_128X32_UNIVISION_HW_I2C u8x8(/* clock=*/ 21, /* data=*/ 20);
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* clock=*/ 21, /* data=*/ 20);

COBD obd;
bool hasMEMS;

#define BUTTON1 A1
#define BUTTON2 A2
#define BUTTON3 A3

byte selectedPage = 1;

#define NO_OF_PIDS 4
byte pids[NO_OF_PIDS];
int pid_values[NO_OF_PIDS];
byte pid_display[NO_OF_PIDS];

/*
 * ===============================================================
 * Ulitlity functions
 * ===============================================================
 */

#ifdef DEBUG
// print debug output on console interface
void debug(debugLevels level, char *sFmt, ...)
{
  if (level > currentDebugLevel) return;  // bypass if level is not high enough
  char acTmp[128];       // place holder for sprintf output
  va_list args;          // args variable to hold the list of parameters
  va_start(args, sFmt);  // mandatory call to initilase args 

  vsprintf(acTmp, sFmt, args);
  mySerial.print(acTmp);
  // mandatory tidy up
  va_end(args);
  return;
}
#endif

void testOut()
{
    static const char cmds[][6] = {"ATZ\r", "ATI\r", "ATH0\r", "ATRV\r", "0100\r", "010C\r", "0902\r"};
    char buf[128];

    for (byte i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++) {
        const char *cmd = cmds[i];
        debug(L_INFO, "Sending %s\n", cmd);
        u8x8.setCursor(0,2);
        u8x8.print("CMD: ");
        u8x8.print(cmd);
        if (obd.sendCommand(cmd, buf, sizeof(buf))) {
            char *p = strstr(buf, cmd);
            if (p)
                p += strlen(cmd);
            else
                p = buf;
            while (*p == '\r') p++;
            while (*p) {
              debug(L_INFO, "%c", *p);
              if (*p == '\r' && *(p + 1) != '\r')
                debug(L_INFO, "\n");
              p++;
            }
            debug(L_INFO, "\n");
        } else {
          debug(L_INFO, "Timeout\n");
        }
        delay(300);
    }
    debug(L_INFO,"\n");
    u8x8.clearLine(2);
}

void readMEMS()
{
  int acc[3];
  int gyro[3];
  int temp;

  if (!obd.memsRead(acc, gyro, 0, &temp)) return;

#ifdef DEBUG
  debug(L_INFO, "[%lu] ACC:%d/%d/%d", millis(), acc[0], acc[1], acc[2]);
  /*
  mySerial.print('[');
  mySerial.print(millis());
  mySerial.print(']');

  mySerial.print("ACC:");
  mySerial.print(acc[0]);
  mySerial.print('/');
  mySerial.print(acc[1]);
  mySerial.print('/');
  mySerial.print(acc[2]);
*/
  debug(L_INFO, " GYRO: %d/%d/%d", gyro[0], gyro[1], gyro[2]);
  /*
  mySerial.print(" GYRO:");
  mySerial.print(gyro[0]);
  mySerial.print('/');
  mySerial.print(gyro[1]);
  mySerial.print('/');
  mySerial.print(gyro[2]);
*/
  debug(L_INFO, " TEMP:%d.%dC\n", temp / 10, temp % 10);
/*
  mySerial.print(" TEMP:");
  mySerial.print((float)temp / 10, 1);
  mySerial.println("C");
*/
#endif
}

void setDisplay() {
  // sets the display mask
  u8x8.clear();
  switch (selectedPage) {
    case 1:
      pids[0]=PID_SPEED; pids[1]=PID_ENGINE_LOAD; 
      pids[2]=PID_RPM; pids[3]=PID_COOLANT_TEMP;
      pid_display[0]=3; pid_display[1]=3;
      pid_display[2]=4; pid_display[3]=3;
      u8x8.draw2x2String(0,0,"SPD:");
      u8x8.draw2x2String(0,2,"LD :   %");
      u8x8.draw2x2String(0,4,"RPM:");
      u8x8.draw2x2String(0,6,"TMP:   C");
      break;
    case 2:
      pids[0]=PID_COMMANDED_EGR; pids[1]=PID_BAROMETRIC; 
      pids[2]=PID_ENGINE_OIL_TEMP; pids[3]=PID_INTAKE_TEMP;
      pid_display[0]=3; pid_display[1]=4;
      pid_display[2]=3; pid_display[3]=3;
      u8x8.draw2x2String(0,0,"EDR:");
      u8x8.draw2x2String(0,2,"BAR:");
      u8x8.draw2x2String(0,4,"OIL:   C");
      u8x8.draw2x2String(0,6,"INT:   C");
      break;
    default:
      pids[0]=PID_SPEED; pids[1]=PID_ENGINE_LOAD; 
      pids[2]=PID_RPM; pids[3]=PID_COOLANT_TEMP;
      pid_display[0]=3; pid_display[1]=3;
      pid_display[2]=4; pid_display[3]=3;
      u8x8.draw2x2String(0,0,"SPD:");
      u8x8.draw2x2String(0,2,"LD :   %");
      u8x8.draw2x2String(0,4,"RPM:");
      u8x8.draw2x2String(0,6,"TMP:   C");
      break;
  }
}

void updateDisplay () {
  char strbuf[10];
  byte pids_read;
  unsigned long timing;
  
#ifdef DEBUG  
  debug(L_INFO, "update page %d\n", selectedPage);
  timing = millis();
#endif
  pids_read = obd.readPID(pids, NO_OF_PIDS, pid_values);
  
#ifdef DEBUG  
  debug(L_INFO, "pid read time: %ulmiilis\n", millis() - timing );
#endif

  if (pids_read == NO_OF_PIDS) {
    for (byte i = 0; i < NO_OF_PIDS ; i++) {
#ifdef DEBUG      
      debug(L_INFO, " %x=%d", (int)pids[i] | 0x100, pid_values[i]);
#endif
      switch(pid_display[i]) {
        case 0:
          // do nothing
          break;
        case 3:
          sprintf(strbuf, "%3d", pid_values[i]);
          break;
        case 4:
        default:
          sprintf(strbuf, "%4d", pid_values[i]);
      }
      if (pid_display[i] != 0)
        u8x8.draw2x2String(8,i*2,strbuf);
    }
  } else {
    debug(L_ERROR, "readPID() error: expected %d PIDs but got %d PIDs\n", NO_OF_PIDS, pids_read);
  }
}

void setup()
{
  char strbuf[32];
  mySerial.begin(115200);
  while (!mySerial);
#ifdef DEBUG  
  currentDebugLevel = DEBUG_LEVEL_DEFAULT;
#endif

  // OLED
  u8x8.begin();
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  u8x8.clear();
  u8x8.drawString(0,0,"OBD Scanner");

  // this will begin serial
  byte ver = obd.begin();

#ifdef DEBUG
  debug(L_INFO, "Freematics OBD-II Adapter ");
#endif
  if (ver > 0) {
    sprintf(strbuf, "Ver. %d.%d\n", ver / 10, ver % 10 );
    u8x8.drawString(0,1,strbuf);
#ifdef DEBUG
    debug(L_INFO, "Ver. %d.%d\n", ver / 10, ver % 10 );
#endif   
  } else {
    u8x8.draw2x2String(0,2,"OBD-II");
    u8x8.draw2x2String(0,4,"Adapter");
    u8x8.draw2x2String(0,6,"missing");
#ifdef DEBUG
    debug(L_INFO, "not detected");
#endif
    for (;;);
  }

  delay(1000);

  // send some commands for testing and show response for debugging purpose
  //testOut();

  hasMEMS = obd.memsInit();
#ifdef DEBUG
  debug(L_INFO, "MEMS:%s\n", hasMEMS ? "Yes" : "No" );
#endif  

  // initialize OBD-II adapter
  do {
    u8x8.drawString(0,4,"Init...");
#ifdef DEBUG    
    debug(L_INFO, "Init...\n");
#endif
  } while (!obd.init());

  char buf[64];
  // display VIN
  if (obd.getVIN(buf, sizeof(buf))) {
    u8x8.setCursor(0,3);
    u8x8.print("VIN:\n");
    u8x8.print(buf);
#ifdef DEBUG
    debug(L_INFO, "VIN: %s\n", buf);
#endif
  }
  
  unsigned int codes[6];
  byte dtcCount = obd.readDTC(codes, 6);
  if (dtcCount == 0) {
    u8x8.draw2x2String(0,6,"No DTC");
#ifdef DEBUG
    debug(L_INFO, "No DTC\n");
#endif
    delay(2000);
  } else {
#ifdef DEBUG
    debug(L_INFO, "%d DTC:", dtcCount);
    for (byte n = 0; n < dtcCount; n++) {
      debug(L_INFO, " %x", codes[n]);
    }
    debug(L_INFO, "\n");
    delay(5000);
#endif
  } 
  u8x8.clear();
  setDisplay();
}

void loop()
{
  updateDisplay();

  if(!digitalRead(BUTTON1)) {
    debug(L_INFO, "Button 1\n");
    if (selectedPage != 1) {
      selectedPage = 1;
      debug(L_INFO, "Page %d selected\n", selectedPage);
      setDisplay();
    }
  }

  if(!digitalRead(BUTTON2)) {
    if (selectedPage != 2) {
      
      selectedPage = 2;
      debug(L_INFO, "Page %d selected\n", selectedPage);
      setDisplay();
    }
  }
  /*
  readPIDSingle();
 
  readPIDMultiple();
  readBatteryVoltage();
  if (hasMEMS) {
    readMEMS();
  }
  */
}


