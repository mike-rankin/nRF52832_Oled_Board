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
static uint8_t imu_addr;
static uint8_t ucBackBuffer[1024];
// frame counter for updating display every other pixel update
static int iFrame;

// Number of grains of sand - this is how many pixels in the first line of text
#define N_GRAINS     192
// Display width in pixels
#define WIDTH        96
// Display height in pixels
#define HEIGHT       16
// Maximum redraw rate, frames/second
#define MAX_FPS      120
 
// The 'sand' grains exist in an integer coordinate space that's 256X
// the scale of the pixel grid, allowing them to move and interact at
// less than whole-pixel increments.
#define MAX_X (WIDTH  * 256 - 1) // Maximum X coordinate in grain space
#define MAX_Y (HEIGHT * 256 - 1) // Maximum Y coordinate
struct Grain {
  int16_t  x,  y; // Position
  int16_t vx, vy; // Velocity
} grain[N_GRAINS];
uint32_t prevTime = 0;           // Used for frames-per-second throttle
uint8_t img[128 * 8]; // Copy of pixel map laid out the same way as the SSD1306

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
   uc[1] = 0x77; // Turn on the sensor with ODR = 400Hz normal mode.
   I2CWrite(&obd.bbi2c, imu_addr, uc, 2);
// High res & BDU enabled
   uc[0] = 0x23; // CTRL_REG4
   uc[1] = 0x88;
   I2CWrite(&obd.bbi2c, imu_addr, uc, 2);
   // DRDY on INT1
   uc[0] = 0x22; // CTRL_REG3
   uc[1] = 0x10;
   I2CWrite(&obd.bbi2c, imu_addr, uc, 2);

 // enable adcs
   uc[0] = 0x1f; // TEMP_CFG_REG
   uc[1] = 0x80;
   I2CWrite(&obd.bbi2c, imu_addr, uc, 2);

   return;
   
   uc[0] = 0x21; // CTRL_REG2
   uc[1] = 0x01; // High-pass filter (HPF) enabled with 0.2Hz cut-off frequency for INT1 (AOI1) interrupt generation only.
   I2CWrite(&obd.bbi2c, imu_addr, uc, 2);
   uc[0] = 0x22; // CTRL_REG3
   uc[1] = 0x40; // ACC AOI1 interrupt signal is routed to INT1 pin.
   I2CWrite(&obd.bbi2c, imu_addr, uc, 2);
   uc[0] = 0x23; // CTRL_REG4
   uc[1] = 0x88; // Full Scale = +/-2 g with BDU and HR bits enabled.
   I2CWrite(&obd.bbi2c, imu_addr, uc, 2);
   uc[0] = 0x24; // CTRL_REG5
//   uc[1] = 0x00; // INT1 pin is not latched, no need to read INT1_SRC to clear the int
   uc[1] = 0x08; // INT1 pin is latched; need to read the INT1_SRC register to clear the interrupt signal.   
   I2CWrite(&obd.bbi2c, imu_addr, uc, 2); 
   // configurations for wakeup and motionless detection
   uc[0] = 0x32; // INT1_THS
   uc[1] = 0x02; // Threshold (THS) = 2LSBs * 15.625mg/LSB = 31.25mg.
   I2CWrite(&obd.bbi2c, imu_addr, uc, 2);
   uc[0] = 0x33; // INT1_DURATION
   uc[1] = 0x01; // Duration = 1LSBs * (1/10Hz) = 0.1s
   I2CWrite(&obd.bbi2c, imu_addr, uc, 2);
   uc[0] = 0x30; // INT1_CFG
   uc[1] = 0x15; // Enable XLIE, YLIE and ZLIE (low events) interrupt generation
//   uc[1] = 0xaa; // Enable ZHIE, YHIE, XHIE (high events) interrupt generation
   I2CWrite(&obd.bbi2c, imu_addr, uc, 2);
} /* LIS3DHInit() */

void ResetDisplay(int bRandom)
{
  int i, j, x, y;
//  float f = GetTemp();
  char szTemp[32];
  uint8_t *pScreen;

  pScreen = ucBackBuffer;
  
  if (bRandom)
  {
    obdFill(&obd, 0, 1);
  //  sprintf(szTemp,"%dC", (int)f);
  //  oledWriteString(0,52,4,szTemp, FONT_NORMAL, 0, 1);
    memset(img, 0, sizeof(img)); // Clear our copy of the image array
    memset(pScreen, 0, sizeof(img));
  //  memcpy(img, pScreen, sizeof(img));
    for(i=0; i<N_GRAINS; i++) {  // For each sand grain...
      do {
        grain[i].x = random(WIDTH  * 256); // Assign random position within
        grain[i].y = random(HEIGHT * 256); // the 'grain' coordinate space
        // Check if corresponding pixel position is already occupied...
        for(j=0; (j<i) && (((grain[i].x / 256) != (grain[j].x / 256)) ||
                           ((grain[i].y / 256) != (grain[j].y / 256))); j++);
      } while(j < i); // Keep retrying until a clear spot is found
      x = grain[i].x / 256; y = grain[i].y / 256;
      img[((y / 8) * 128) + x] |= (1 << (y & 7)); // Mark it
      grain[i].vx = grain[i].vy = 0; // Initial velocity is zero
    }
  } // random grains
  else
  {
    int x, y;
    i = 0; // number of grains
    obdWriteString(&obd,0,0,1,(char *)"            ", FONT_8x8, 0,1); // erase second line - too many pixels
    memcpy(img, pScreen, sizeof(img)); // update our copy of the image array
    for (y=0; y<HEIGHT; y++)
    {
      for (x=0; x<WIDTH; x++)
      {
        if (pScreen[x + (y >> 3)*128] & (1 << (y & 7))) // pixel set?
        {
          grain[i].x = x*256; grain[i].y = y*256;
          grain[i].vx = grain[i].vy = 0; // Initial velocity is zero
          i++;
          if (i == N_GRAINS) return;
        }
      } // for x
    } // for y
//    Serial.println(i, DEC);
  }

} /* ResetDisplay() */

void setup() {
  // put your setup code here, to run once:
  obdI2CInit(&obd, OLED_96x16, OLED_ADDR, FLIP180, INVERT, USE_HW_I2C, SDA_PIN, SCL_PIN, RESET_PIN, 400000L);
  obdSetBackBuffer(&obd, ucBackBuffer);
  obdFill(&obd, 0, 1);
  obdSetContrast(&obd, 10); // dim the display to save power
  obdWriteString(&obd, 0,0,0,(char *)" Sand Grain ", FONT_8x8, 0, 1);
  obdWriteString(&obd, 0,0,1,(char *)" Simulation ", FONT_8x8, 0, 1);
  delay(3000);
  LIS3DHInit(IMUADDR);
  ResetDisplay(0);
}

void loop() {
   int16_t i, x, y, z;
  uint32_t t;
  int32_t v2; // Velocity squared
  int16_t ax, ay, az;
  signed int        oldidx, newidx;
  uint8_t    oldbit, newbit;
  signed int        newx, newy;
  signed int        x1, y1, x2, y2;
  byte butt;

  // Limit the animation frame rate to MAX_FPS.  Because the subsequent sand
  // calculations are non-deterministic (don't always take the same amount
  // of time, depending on their current states), this helps ensure that
  // things like gravity appear constant in the simulation.
  while(((t = micros()) - prevTime) < (1000000L / MAX_FPS));
  prevTime = t;

//  butt = digitalRead(BUTTON0);
//  if (butt == LOW && bOldButt0 == HIGH) // pressed
//  if (bOldButt1 == 0) // right button pressed
//  {
//    bOldButt1 = 1;
//    ResetDisplay(1);
//  }
//  if (bOldButt0 == 0) // left button pressed
//  {
//    bOldButt0 = 1;
//    obdFill(&obd, 0, 1);
//    obdWriteString(&obd, 0,0,0,(char *)" Sand Grain ", FONT_8x8, 0, 1);
//    ResetDisplay(0);
//  }
//  bOldButt0 = butt;
  // Display the current pixels (every other time through the loop)
  // Since we have velocities of less than 1 pixel, this allows for movement up to 2 pixels
  // per display update and still looks smooth since there are fractional velocities
  if ((iFrame & 1) == 0)
//#ifdef USE_SPI
//     uc1701DumpBuffer(img);
//#else
     obdDumpBuffer(&obd, img);
//#endif
  iFrame++;

  // Read accelerometer...
//  mpu6050ReadAccel(&x, &y, &z);
  LIS3DHReadAccel(&x, &y, &z);
#ifdef USE_SPI
  ax = -y / 512;
  ay = -x / 512;
#else
//  ax = -x / 512;      // Transform accelerometer axes
  ax = x / 512;
  ay = -y / 512;      // to grain coordinate space
#endif
  az = abs(z) / 2048; // Random motion factor
  az = (az >= 3) ? 1 : 4 - az;      // Clip & invert
  ax -= az;                         // Subtract motion factor from X, Y
  ay -= az;
 
  // Apply 2D accelerometer vector to grain velocities...
  //
  // Theory of operation:
  // if the 2D vector of the new velocity is too big (sqrt is > 256), this means it might jump
  // over pixels. We want to limit the velocity to 1 pixel as a maximum.
  // To avoid using floating point math (sqrt + 2 multiplies + 2 divides)
  // Instead of normalizing the velocity to keep the same direction, we can trim the new
  // velocity to 5/8 of it's value. This is a reasonable approximation since the maximum
  // velocity impulse from the accelerometer is +/-64 (16384 / 256) and it gets added every frame
  //
  for(i=0; i<N_GRAINS; i++) {
    grain[i].vx += ax;// + random(5); // Add a little random impulse to each grain
    grain[i].vy += ay;// + random(5);
    v2 = (int32_t)(grain[i].vx*grain[i].vx) + (int32_t)(grain[i].vy*grain[i].vy);
    if (v2 >= 65536) // too big, trim it
    {
      grain[i].vx = (grain[i].vx * 5)/8; // quick and dirty way to avoid doing a 'real' divide
      grain[i].vy = (grain[i].vy * 5)/8;
    }
  } // for i
 
  // Update the position of each grain, one at a time, checking for
  // collisions and having them react.  This really seems like it shouldn't
  // work, as only one grain is considered at a time while the rest are
  // regarded as stationary.  Yet this naive algorithm, taking many not-
  // technically-quite-correct steps, and repeated quickly enough,
  // visually integrates into something that somewhat resembles physics.
  // (I'd initially tried implementing this as a bunch of concurrent and
  // "realistic" elastic collisions among circular grains, but the
  // calculations and volument of code quickly got out of hand for both
  // the tiny 8-bit AVR microcontroller and my tiny dinosaur brain.)
  //
  // (x,y) to bytes mapping:
  // The SSD1306 has 8 rows of 128 bytes with the LSB of each byte at the top
  // In other words, bytes are oriented vertically with bit 0 as the top pixel
  // Part of my optimizations were writing the pixels into memory the same way they'll be
  // written to the display. This means calculating an offset and bit to test/set each pixel
  //
  for(i=0; i<N_GRAINS; i++) {
    newx = grain[i].x + grain[i].vx; // New position in grain space
    newy = grain[i].y + grain[i].vy;
    if(newx > MAX_X) {               // If grain would go out of bounds
      newx         = MAX_X;          // keep it inside, and
      grain[i].vx /= -2;             // give a slight bounce off the wall
    } else if(newx < 0) {
      newx         = 0;
      grain[i].vx /= -2;
    }
    if(newy > MAX_Y) {
      newy         = MAX_Y;
      grain[i].vy /= -2;
    } else if(newy < 0) {
      newy         = 0;
      grain[i].vy /= -2;
    }

    x1 = grain[i].x / 256; y1 = grain[i].y / 256;
    oldidx = ((y1/8) * 128/*WIDTH*/ + x1); // Prior pixel #
    oldbit = 1 << (y1 & 7);
    x2 = newx / 256; y2 = newy / 256;
    newidx = ((y2/8) * 128 /*WIDTH*/ + x2); // New pixel #
    newbit = 1 << (y2 & 7);
    if((oldidx != newidx || oldbit != newbit) && // If grain is moving to a new pixel...
        (img[newidx] & newbit) != 0) {       // but if that pixel is already occupied...
        // Try skidding along just one axis of motion if possible (start w/faster axis)
        if(abs(grain[i].vx) > abs(grain[i].vy)) { // X axis is faster
          x1 = newx / 256; y1 = grain[i].y / 256;
          newidx = ((y1 / 8) * 128/*WIDTH*/ + x1);
          newbit = 1 << (y1 & 7);
          if((img[newidx] & newbit) == 0) { // That pixel's free!  Take it!  But...
            newy         = grain[i].y; // Cancel Y motion
            grain[i].vy = (grain[i].vy /-2) + random(8);         // and bounce Y velocity
          } else { // X pixel is taken, so try Y...
            y1 = newy / 256; x1 = grain[i].x / 256;
            newidx = ((y1 / 8) * 128/*WIDTH*/ + x1);
            newbit = 1 << (y1 & 7);
            if((img[newidx] & newbit) == 0) { // Pixel is free, take it, but first...
              newx         = grain[i].x; // Cancel X motion
              grain[i].vx = (grain[i].vx /-2) + random(8);         // and bounce X velocity
            } else { // Both spots are occupied
              newx         = grain[i].x; // Cancel X & Y motion
              newy         = grain[i].y;
              grain[i].vx = (grain[i].vx /-2) + random(8);         // Bounce X & Y velocity
              grain[i].vy = (grain[i].vy /-2) + random(8);
            }
          }
        } else { // Y axis is faster
          y1 = newy / 256; x1 = grain[i].x / 256;
          newidx = ((y1 / 8) * 128/*WIDTH*/ + x1);
          newbit = 1 << (y1 & 7);
          if((img[newidx] & newbit) == 0) { // Pixel's free!  Take it!  But...
            newx         = grain[i].x; // Cancel X motion
            grain[i].vx = (grain[i].vx /-2) + random(8);        // and bounce X velocity
          } else { // Y pixel is taken, so try X...
            y1 = grain[i].y / 256; x1 = newx / 256;
            newidx = ((y1 / 8) * 128/*WIDTH*/ + x1);
            newbit = 1 << (y1 & 7);
            if((img[newidx] & newbit) == 0) { // Pixel is free, take it, but first...
              newy         = grain[i].y; // Cancel Y motion
              grain[i].vy = (grain[i].vy /-2) + random(8);        // and bounce Y velocity
            } else { // Both spots are occupied
              newx         = grain[i].x; // Cancel X & Y motion
              newy         = grain[i].y;
              grain[i].vx = (grain[i].vx /-2) + random(8);         // Bounce X & Y velocity
              grain[i].vy = (grain[i].vy /-2) + random(8);
            }
          }
        }
    }
    if (grain[i].x != newx || grain[i].y != newy)
    {
      y1 = newy / 256; x1 = newx / 256;
      newidx = ((y1 / 8) * 128/*WIDTH*/ + x1);
      newbit = 1 << (y1 & 7);
      grain[i].x  = newx; // Update grain position
      grain[i].y  = newy;
      img[oldidx] &= ~oldbit; // erase old pixel
      img[newidx] |= newbit;  // Set new pixel
    }
  }
} // loop
