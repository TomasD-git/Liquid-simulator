#include <Arduino.h>
#include <SPI.h>
#include <LovyanGFX.hpp>
#include "lgfx_user.hpp"

LGFX tft;
LGFX_Sprite fb(&tft);


// IMU PINS

#define PIN_IMU_SCK   4
#define PIN_IMU_MOSI  5
#define PIN_IMU_MISO  21
#define PIN_IMU_CS    15
#define PIN_IMU_INT1  14
#define PIN_BTN       9

#define HAVE_BMI270
#include "bmi270_config_file.h"


static SPIClass imuSPI(SPI);

static const int W = 240;
static const int H = 240;
static const int CX = 120;
static const int CY = 120;
static const float RING_R = 116.0f;

static const int NP = 90;
static const float PR = 8.5f;
static const float INTERACT = 22.0f;
static const float I2 = INTERACT * INTERACT;
static const float WALL = RING_R - PR - 1.5f;
static const float DAMP = 0.965f;
static const float G_SCALE = 3.2f;
static const float LP_ALPHA = 0.14f;

struct Particle { float x,y,vx,vy; };
static Particle p[NP];

static float imu_ax = 0;
static float imu_ay = 0;

#define BMI270_REG_CHIP_ID 0x00
#define BMI270_REG_ACC_DATA 0x0C
#define BMI270_REG_ACC_CONF 0x40
#define BMI270_REG_ACC_RANGE 0x41
#define BMI270_REG_PWR_CTRL 0x7D
#define BMI270_REG_PWR_CONF 0x7C
#define BMI270_REG_CMD 0x7E
#define BMI270_REG_INIT_CTRL 0x59
#define BMI270_REG_INIT_ADDR_0 0x5B
#define BMI270_REG_INIT_ADDR_1 0x5C
#define BMI270_REG_INIT_DATA 0x5E
#define BMI270_REG_INTERNAL_STATUS 0x21

static void imuWrite(uint8_t reg, uint8_t val) {
  digitalWrite(PIN_IMU_CS, LOW);
  imuSPI.transfer(reg & 0x7F);
  imuSPI.transfer(val);
  digitalWrite(PIN_IMU_CS, HIGH);
}

static uint8_t imuRead(uint8_t reg) {
  digitalWrite(PIN_IMU_CS, LOW);
  imuSPI.transfer(reg | 0x80);
  imuSPI.transfer(0x00);
  uint8_t v = imuSPI.transfer(0x00);
  digitalWrite(PIN_IMU_CS, HIGH);
  return v;
}

static void imuBurst(uint8_t reg, uint8_t* buf, uint8_t n) {
  digitalWrite(PIN_IMU_CS, LOW);
  imuSPI.transfer(reg | 0x80);
  imuSPI.transfer(0x00);
  for (int i = 0; i < n; i++) buf[i] = imuSPI.transfer(0x00);
  digitalWrite(PIN_IMU_CS, HIGH);
}

bool initBMI270() {
  imuSPI.begin(PIN_IMU_SCK, PIN_IMU_MISO, PIN_IMU_MOSI);
  imuSPI.setFrequency(8000000);
  pinMode(PIN_IMU_CS, OUTPUT);
  digitalWrite(PIN_IMU_CS, HIGH);
  delay(20);
  imuWrite(BMI270_REG_CMD, 0xB6);
  delay(20);
  if (imuRead(BMI270_REG_CHIP_ID) != 0x24) {
    Serial.println("BMI270 not found");
    return false;
  }
#ifdef HAVE_BMI270
  imuWrite(BMI270_REG_INIT_CTRL, 0x00);
  for (uint16_t off = 0; off < bmi270_config_file_size; off += 256) {
    uint16_t half = off >> 1;
    imuWrite(BMI270_REG_INIT_ADDR_0, half & 0xFF);
    imuWrite(BMI270_REG_INIT_ADDR_1, half >> 8);
    digitalWrite(PIN_IMU_CS, LOW);
    imuSPI.transfer(BMI270_REG_INIT_DATA & 0x7F);
    for (int i = 0; i < 256 && (off + i) < bmi270_config_file_size; i++)
      imuSPI.transfer(bmi270_config_file[off + i]);
    digitalWrite(PIN_IMU_CS, HIGH);
  }
#endif

  imuWrite(BMI270_REG_INIT_CTRL, 0x01);
  delay(100);

  imuWrite(BMI270_REG_ACC_CONF, 0xA8);
  imuWrite(BMI270_REG_ACC_RANGE, 0x00);
  imuWrite(BMI270_REG_PWR_CTRL, 0x04);
  imuWrite(BMI270_REG_PWR_CONF, 0x02);

  return true;
}
// READ IMU
void readIMU() {
  uint8_t b[6];
  imuBurst(BMI270_REG_ACC_DATA, b, 6);

  int16_t ax = (int16_t)(b[1] << 8 | b[0]);
  int16_t ay = (int16_t)(b[3] << 8 | b[2]);

  float fx = ax / 16384.0f;
  float fy = ay / 16384.0f;

  imu_ax += LP_ALPHA * (fx - imu_ax);
  imu_ay += LP_ALPHA * (fy - imu_ay);
}
// PARTICLES
void initParticles() {
  for (int i=0;i<NP;i++) {
    float r = sqrt((i+1.0)/NP) * WALL * 0.6f;
    float a = i * 2.399963f;
    p[i] = {
      CX + r*cos(a),
      CY + r*sin(a),
      0,0
    };
  }
}
// PHYSICS
void physics(float gx, float gy) {
  for (int i=0;i<NP;i++) {
    p[i].vx += gx * 0.016f;
    p[i].vy += gy * 0.016f;
    for (int j=i+1;j<NP;j++) {
      float dx = p[i].x - p[j].x;
      float dy = p[i].y - p[j].y;
      float d2 = dx*dx + dy*dy;
      if (d2 < I2 && d2 > 0.1f) {
        float d = sqrt(d2);
        float f = (INTERACT - d)/INTERACT * 0.5f;
        float nx = dx/d * f;
        float ny = dy/d * f;
        p[i].vx += nx; p[i].vy += ny;
        p[j].vx -= nx; p[j].vy -= ny;
      }
    }
  }
  for (int i=0;i<NP;i++) {
    p[i].vx *= DAMP;
    p[i].vy *= DAMP;
    p[i].x += p[i].vx;
    p[i].y += p[i].vy;
    float dx = p[i].x - CX;
    float dy = p[i].y - CY;
    float d = sqrt(dx*dx + dy*dy);
    if (d > WALL) {
      float nx = dx/d;
      float ny = dy/d;
      p[i].x = CX + nx * WALL;
      p[i].y = CY + ny * WALL;
      float dot = p[i].vx*nx + p[i].vy*ny;
      p[i].vx -= 1.8f * dot * nx;
      p[i].vy -= 1.8f * dot * ny;
    }
  }
}
// RENDER
void render() {
  fb.fillScreen(TFT_BLACK);
  fb.fillCircle(CX,CY,118, fb.color565(2,5,22));
  for (int i=0;i<NP;i++) {
    fb.fillCircle(p[i].x,p[i].y,8, fb.color565(0,180,255));
  }
  fb.pushSprite(0,0);
}
// SETUP
void setup() {
  Serial.begin(115200);
  tft.init();
  tft.setRotation(0);
  fb.setColorDepth(16);
  fb.createSprite(W,H);
  pinMode(PIN_IMU_CS, OUTPUT);
  digitalWrite(PIN_IMU_CS, HIGH);
  initParticles();
  if (initBMI270())
    Serial.println("IMU OK");
  else
    Serial.println("IMU FAIL → demo mode");
}
// LOOP
void loop() {
  readIMU();
  // Tune this if wrong orintation
  float gx = -imu_ax * G_SCALE;
  float gy =  imu_ay * G_SCALE;
  physics(gx, gy);
  render();
}