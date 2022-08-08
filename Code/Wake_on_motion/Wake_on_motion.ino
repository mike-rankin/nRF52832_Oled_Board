//
// Demo software for Mike's tiny nRF52 coin cell board
//
#include <OneBitDisplay.h>
#include <BitBang_I2C.h>

OBDISP obd;
#define RESET_PIN 23
#define SDA_PIN -1
#define SCL_PIN -1
#define USE_HW_I2C 1
#define OLED_ADDR 0x3c
#define FLIP180 1
#define INVERT 0
#define IMUADDR 0x19
// Interrupt pin is connected to D3 (PA.03)
#define INT_PIN 3
static uint8_t imu_addr;
static uint8_t ucBackBuffer[1024];
volatile int iIntFlag = 0;

void LIS3DHReadAccel(int16_t *X, int16_t *Y, int16_t *Z)
{
int i;
uint8_t ucTemp[8];

  i = I2CReadRegister(&obd.bbi2c, imu_addr, 0xa8, ucTemp, 6);
  if (i > 0)
  {
    *X = (ucTemp[1] << 8) + ucTemp[0];
    *Y = (ucTemp[3] << 8) + ucTemp[2];
    *Z = (ucTemp[5] << 8) + ucTemp[4];
  }
  
} /* LIST3DHReadAccel() */

void LIS3DHInit(byte bAddr)
{
uint8_t uc[4];

   imu_addr = bAddr;
   uc[0] = 0x20; // CTRL_REG1
   uc[1] = 0x4F; // Turn on the sensor with ODR = 50Hz, low power mode.
   I2CWrite(&obd.bbi2c, imu_addr, uc, 2);
// High res & BDU enabled
//   uc[0] = 0x23; // CTRL_REG4
//   uc[1] = 0x88;
//   I2CWrite(&obd.bbi2c, imu_addr, uc, 2);
   // DRDY on INT1
//   uc[0] = 0x22; // CTRL_REG3
//   uc[1] = 0x10;
//   I2CWrite(&obd.bbi2c, imu_addr, uc, 2);

 // enable adcs
//   uc[0] = 0x1f; // TEMP_CFG_REG
//   uc[1] = 0x80;
//   I2CWrite(&obd.bbi2c, imu_addr, uc, 2);
   
//   uc[0] = 0x21; // CTRL_REG2
//   uc[1] = 0x01; // High-pass filter (HPF) enabled with 0.2Hz cut-off frequency for INT1 (AOI1) interrupt generation only.
//   I2CWrite(&obd.bbi2c, imu_addr, uc, 2);
   uc[0] = 0x22; // CTRL_REG3
   uc[1] = 0x40; // ACC AOI1 interrupt signal is routed to INT1 pin.
   I2CWrite(&obd.bbi2c, imu_addr, uc, 2);
   uc[0] = 0x23; // CTRL_REG4
   uc[1] = 0x80; // Full Scale = +/-2 g with BDU and HR bit disabled.
   I2CWrite(&obd.bbi2c, imu_addr, uc, 2);
   uc[0] = 0x24; // CTRL_REG5
   uc[1] = 0x08; // INT1 pin is latched; need to read the INT1_SRC register to clear the interrupt signal.   
   I2CWrite(&obd.bbi2c, imu_addr, uc, 2); 
   // configurations for wakeup and motionless detection
   uc[0] = 0x32; // INT1_THS
   uc[1] = 0x04; // Threshold (THS) = 2LSBs * 15.625mg/LSB = 31.25mg.
   I2CWrite(&obd.bbi2c, imu_addr, uc, 2);
   uc[0] = 0x33; // INT1_DURATION
   uc[1] = 0x01; // Duration = 1LSBs * (1/10Hz) = 0.1s
   I2CWrite(&obd.bbi2c, imu_addr, uc, 2);
   uc[0] = 0x30; // INT1_CFG
   uc[1] = 0x02; //0x15; // Enable XLIE, YLIE and ZLIE (low events) interrupt generation
//   uc[1] = 0xaa; // Enable ZHIE, YHIE, XHIE (high events) interrupt generation
   I2CWrite(&obd.bbi2c, imu_addr, uc, 2);
} /* LIS3DHInit() */

void AccelInterrupt(void)
{
  iIntFlag = 1;
} /* AccelInterrupt() */

void setup() {
  int16_t x, y, z;
  int i, iCount = 0;
  char szTemp[32];
  uint8_t ucTemp[8];

  // put your setup code here, to run once:
  obdI2CInit(&obd, OLED_96x16, OLED_ADDR, FLIP180, INVERT, USE_HW_I2C, SDA_PIN, SCL_PIN, RESET_PIN, 400000L);
  LIS3DHInit(IMUADDR);
  obdSetBackBuffer(&obd, ucBackBuffer);
  obdFill(&obd, 0, 1);
  obdSetContrast(&obd, 5); // dim the display to save power
  pinMode(INT_PIN, INPUT); // interrupt pin from LIS3DH
//  attachInterrupt(INT_PIN, AccelInterrupt, RISING);
  I2CReadRegister(&obd.bbi2c, imu_addr, 0x31, ucTemp, 1);
  sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
  while (1)
  {
//    if (digitalRead(3)) {
//       I2CReadRegister(&obd.bbi2c, imu_addr, 0x31, ucTemp, 1);
//       sprintf(szTemp,"0x%02x 0x%02x", digitalRead(3), ucTemp[0]);
//    } else {
//      strcpy(szTemp, "Nope");
//    LIS3DHReadAccel(&x, &y, &z);
//    }
   delay(500);
//   __WFE();
//   __SEV();
//   __WFE();
//     __WFI();
//      sd_app_evt_wait(); // if softdevice enabled
//     waitForEvent();
      if (digitalRead(INT_PIN)) { //iIntFlag) {
      iIntFlag = 0;
      iCount++;
      obdPower(&obd, 1); // turn on display
      I2CReadRegister(&obd.bbi2c, imu_addr, 0x31, &ucTemp[0], 1); // reset INT1 line
      sprintf(szTemp,"%d bumps", iCount);
      obdWriteString(&obd, 0,0,0,szTemp, FONT_12x16, 0, 1);
      delay(2000);
      obdPower(&obd, 0); // turn off the display
    }
  }
} /* setup() */

void loop() {
} // loop
