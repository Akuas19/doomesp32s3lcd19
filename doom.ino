#include <Arduino_GFX_Library.h>
#include <Wire.h>
#include <math.h>

#define LCD_BL 14
#define BTN_BOOT 0

// setup for Waveshare ESP32-S3-LCD-1.9
Arduino_DataBus *bus = new Arduino_ESP32SPI(
  11 /* DC */, 12 /* CS */, 10 /* SCK */, 13 /* MOSI */, -1 /* MISO */, FSPI
);

Arduino_GFX *gfx = new Arduino_ST7789(
  bus, 9 /* RST */, 1 /* Rotation: Landscape */, true /* IPS */,
  170 /* width */, 320 /* height */,
  35 /* col_offset1 */, 0 /* row_offset1 */,
  35 /* col_offset2 */, 0 /* row_offset2 */
);

Arduino_Canvas_Indexed *canvasGfx = new Arduino_Canvas_Indexed(320, 170, gfx);

// --- QMI8658 Hardware Driver ---
uint8_t imuAddr = 0x6B;
int sdaPin = -1;
int sclPin = -1;
bool imuFound = false;
int32_t axBias = 0; // Steering center point
int32_t ayBias = 0; // Gas pedal center point

void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(imuAddr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

// verify QMI8658 chip ID
bool verifyQMIChip(uint8_t addr) {
  Wire.beginTransmission(addr);
  Wire.write(0x00); // WHO_AM_I register
  if (Wire.endTransmission(true) != 0) return false;

  if (Wire.requestFrom((uint8_t)addr, (uint8_t)1) == 1) {
    uint8_t chipId = Wire.read();
    return (chipId == 0x05); 
  }
  return false;
}

bool testAndInitPinPair(int sda, int scl) {
  Wire.end();
  delay(10);
  Wire.begin(sda, scl, 100000);

  if (verifyQMIChip(0x6B)) { imuAddr = 0x6B; return true; }
  if (verifyQMIChip(0x6A)) { imuAddr = 0x6A; return true; }
  return false;
}

bool initQMI8658() {
  const int pairs[][2] = {
    {6, 7}, {7, 6}, {4, 5}, {5, 4},
    {15, 16}, {17, 18}, {1, 2}, {2, 1},
    {38, 39}, {47, 48}
  };

  for (int i = 0; i < 10; i++) {
    if (testAndInitPinPair(pairs[i][0], pairs[i][1])) {
      sdaPin = pairs[i][0];
      sclPin = pairs[i][1];
      imuFound = true;
      break;
    }
  }

  if (!imuFound) return false;

  writeReg(0x02, 0x40);
  writeReg(0x03, 0x23);
  writeReg(0x04, 0x53);
  writeReg(0x08, 0x03);

  return true;
}

bool readSensor(int16_t &ax, int16_t &ay) {
  if (!imuFound) return false;
  Wire.beginTransmission(imuAddr);
  Wire.write(0x35);
  if (Wire.endTransmission(true) != 0) return false;

  if (Wire.requestFrom((uint8_t)imuAddr, (uint8_t)4) == 4) {
    int16_t hwAx = (int16_t)(Wire.read() | (Wire.read() << 8)); // Physical X
    int16_t hwAy = (int16_t)(Wire.read() | (Wire.read() << 8)); // Physical Y
    
    // SWAP AXES FOR LANDSCAPE MODE
    ax = hwAy;
    ay = hwAx;
    
    return true;
  }
  return false;
}

void calibrateIMU() {
  canvasGfx->fillScreen(0x0000);
  canvasGfx->setTextSize(2);
  canvasGfx->setTextColor(0xFFE0);
  canvasGfx->setCursor(20, 55);
  canvasGfx->print("HOLD IN PLAYING POS"); 
  canvasGfx->setTextSize(1);
  canvasGfx->setTextColor(0xFFFF);
  canvasGfx->setCursor(50, 90);
  canvasGfx->print("Calibrating to your hands...");
  canvasGfx->flush();

  int32_t sumAx = 0;
  int32_t sumAy = 0;
  int validSamples = 0;

  for (int i = 0; i < 60; i++) {
    int16_t rawAx = 0, rawAy = 0;
    if (readSensor(rawAx, rawAy)) {
      sumAx += rawAx;
      sumAy += rawAy;
      validSamples++;
    }
    delay(15);
  }

  if (validSamples > 20) {
    axBias = sumAx / validSamples;
    ayBias = sumAy / validSamples;
  } else {
    imuFound = false;
  }
}

// --- 16x16 Map ---
#define MAP_W 16
#define MAP_H 16
const uint8_t worldMap[MAP_W][MAP_H] = {
  {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
  {1,0,0,0,0,0,1,0,0,0,0,0,0,0,0,1},
  {1,0,2,2,0,0,1,0,3,3,3,0,0,0,0,1},
  {1,0,2,0,0,0,0,0,3,0,3,0,0,0,0,1},
  {1,0,2,0,0,0,1,0,3,0,3,0,4,4,0,1},
  {1,0,0,0,0,0,1,0,0,0,0,0,4,4,0,1},
  {1,0,0,0,0,0,1,1,0,1,1,0,0,0,0,1},
  {1,1,0,1,1,0,0,0,0,0,0,0,0,0,0,1},
  {1,0,0,0,1,0,0,0,0,0,0,0,2,2,0,1},
  {1,0,0,0,1,0,4,0,0,4,0,0,2,2,0,1},
  {1,0,3,0,1,0,0,0,0,0,0,0,0,0,0,1},
  {1,0,3,0,0,0,4,0,0,4,0,0,0,0,0,1},
  {1,0,3,3,3,0,0,0,0,0,0,3,3,3,0,1},
  {1,0,0,0,0,0,0,0,0,0,0,3,0,3,0,1},
  {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}
};

float posX = 1.5f, posY = 1.5f;
float dirX = 1.0f, dirY = 0.0f;
float planeX = 0.0f, planeY = 0.66f;

float walkCycle = 0.0f;
int ammo = 50;
int health = 100;
int kills = 0;
int firingFrame = 0;
int hurtFlash = 0;
int fps = 0, frameCount = 0;
unsigned long lastFpsTime = 0;
unsigned long lastFrameTime = 0;

float zBuffer[320];

const uint8_t demonSprite[10][10] = {
  {0, 4, 0, 0, 0, 0, 0, 0, 4, 0},
  {0, 4, 1, 1, 1, 1, 1, 1, 4, 0},
  {1, 1, 2, 2, 2, 2, 2, 2, 1, 1},
  {1, 2, 3, 3, 2, 2, 3, 3, 2, 1},
  {1, 2, 3, 5, 2, 2, 5, 3, 2, 1},
  {1, 2, 2, 2, 2, 2, 2, 2, 2, 1},
  {0, 1, 5, 5, 5, 5, 5, 5, 1, 0},
  {0, 1, 4, 5, 4, 4, 5, 4, 1, 0},
  {0, 0, 1, 4, 5, 5, 4, 1, 0, 0},
  {0, 0, 0, 1, 1, 1, 1, 0, 0, 0}
};

struct Enemy {
  float x, y;
  int health;
  bool alive;
  int hurtTimer;
};

#define NUM_ENEMIES 4
Enemy enemies[NUM_ENEMIES] = {
  {3.5f, 4.5f, 60, true, 0},
  {10.5f, 2.5f, 60, true, 0},
  {8.5f, 8.5f, 60, true, 0},
  {14.5f, 10.5f, 60, true, 0}
};

void rotatePlayer(float angle) {
  float oldDirX = dirX;
  dirX = dirX * cosf(angle) - dirY * sinf(angle);
  dirY = oldDirX * sinf(angle) + dirY * cosf(angle);
  float oldPlaneX = planeX;
  planeX = planeX * cosf(angle) - planeY * sinf(angle);
  planeY = oldPlaneX * sinf(angle) + planeY * cosf(angle);
}

void fireWeapon() {
  if (firingFrame == 0 && ammo > 0) {
    firingFrame = 1;
    ammo--;
    for (int i = 0; i < NUM_ENEMIES; i++) {
      if (enemies[i].alive) {
        float ex = enemies[i].x - posX;
        float ey = enemies[i].y - posY;
        float dist = sqrtf(ex * ex + ey * ey);
        float dot = (ex * dirX + ey * dirY) / dist;
        if (dot > 0.82f && dist < 7.0f) {
          if (dist <= zBuffer[160] + 0.5f) {
            enemies[i].health -= 35;
            enemies[i].hurtTimer = 3;
            if (enemies[i].health <= 0) {
              enemies[i].alive = false;
              kills++;
            }
            break;
          }
        }
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(BTN_BOOT, INPUT_PULLUP);
  pinMode(LCD_BL, OUTPUT);
  digitalWrite(LCD_BL, LOW);

  gfx->begin();
  canvasGfx->begin();

  if (initQMI8658()) {
    calibrateIMU();
  }
  lastFrameTime = millis();
}

void loop() {
  unsigned long now = millis();
  float dt = (now - lastFrameTime) / 1000.0f;
  if (dt > 0.1f) dt = 0.1f;
  lastFrameTime = now;

  if (digitalRead(BTN_BOOT) == LOW) {
    fireWeapon();
  }

  // --- 1. Movement & Input ---
  if (imuFound) {
    int16_t rawAx = 0, rawAy = 0;
    if (readSensor(rawAx, rawAy)) {
      int16_t axCal = rawAx - axBias;
      int16_t ayCal = rawAy - ayBias;

      // STEERING: Left/Right tilt in landscape
      if (abs(axCal) > 1000) {
        float turnSpeed = ((float)axCal / 8192.0f) * 2.5f * dt;
        rotatePlayer(-turnSpeed); 
      }

      // WALKING: Forward/Back tilt in landscape moves you FORWARD
      if (abs(ayCal) > 1200) {
        float pitchRatio = fabsf((float)ayCal / 8192.0f);
        if (pitchRatio > 1.5f) pitchRatio = 1.5f;
        
        float speed = pitchRatio * 3.5f * dt;
        float moveX = dirX * speed;
        float moveY = dirY * speed;
        float pad = 0.25f;

        if (worldMap[int(posX + moveX + (moveX > 0 ? pad : -pad))][int(posY)] == 0) {
          posX += moveX;
        }
        if (worldMap[int(posX)][int(posY + moveY + (moveY > 0 ? pad : -pad))] == 0) {
          posY += moveY;
        }
        
        walkCycle += speed * 8.0f;
      }
    }
  } else {
    // Autopilot fallback 
    float checkDist = 0.65f;
    int frontX = int(posX + dirX * checkDist);
    int frontY = int(posY + dirY * checkDist);

    if (worldMap[frontX][frontY] == 0) {
      posX += dirX * 0.045f;
      posY += dirY * 0.045f;
      walkCycle += 0.15f;
    } else {
      rotatePlayer(-0.045f);
    }
  }

  // --- 2. Enemy AI ---
  for (int i = 0; i < NUM_ENEMIES; i++) {
    if (!enemies[i].alive) continue;
    float dx = posX - enemies[i].x;
    float dy = posY - enemies[i].y;
    float dist = sqrtf(dx * dx + dy * dy);

    if (dist > 0.6f && dist < 6.0f) {
      float spd = 0.018f;
      float nX = enemies[i].x + (dx / dist) * spd;
      float nY = enemies[i].y + (dy / dist) * spd;
      if (worldMap[int(nX)][int(enemies[i].y)] == 0) enemies[i].x = nX;
      if (worldMap[int(enemies[i].x)][int(nY)] == 0) enemies[i].y = nY;
    } else if (dist <= 0.6f) {
      health -= 1;
      hurtFlash = 2;
      if (health < 0) health = 100;
    }
    if (enemies[i].hurtTimer > 0) enemies[i].hurtTimer--;
  }

  // --- 3. Ceiling & Floor ---
  uint16_t ceilColor = (hurtFlash > 0) ? 0x8000 : 0x18C3;
  canvasGfx->fillRect(0, 0, 320, 69, ceilColor);
  canvasGfx->fillRect(0, 69, 320, 70, 0x2104);
  if (hurtFlash > 0) hurtFlash--;

  // --- 4. Wall Raycasting Loop ---
  const int screenW = 320;
  const int viewH = 138;

  for (int x = 0; x < screenW; x++) {
    float cameraX = 2.0f * x / float(screenW) - 1.0f;
    float rayDirX = dirX + planeX * cameraX;
    float rayDirY = dirY + planeY * cameraX;

    int mapX = int(posX);
    int mapY = int(posY);

    float sideDistX, sideDistY;
    float deltaDistX = (rayDirX == 0) ? 1e30f : fabsf(1.0f / rayDirX);
    float deltaDistY = (rayDirY == 0) ? 1e30f : fabsf(1.0f / rayDirY);
    float perpWallDist;

    int stepX, stepY;
    int hit = 0, side = 0;

    if (rayDirX < 0) {
      stepX = -1;
      sideDistX = (posX - mapX) * deltaDistX;
    } else {
      stepX = 1;
      sideDistX = (mapX + 1.0f - posX) * deltaDistX;
    }
    if (rayDirY < 0) {
      stepY = -1;
      sideDistY = (posY - mapY) * deltaDistY;
    } else {
      stepY = 1;
      sideDistY = (mapY + 1.0f - posY) * deltaDistY;
    }

    while (hit == 0) {
      if (sideDistX < sideDistY) {
        sideDistX += deltaDistX;
        mapX += stepX;
        side = 0;
      } else {
        sideDistY += deltaDistY;
        mapY += stepY;
        side = 1;
      }
      if (worldMap[mapX][mapY] > 0) hit = 1;
    }

    if (side == 0) perpWallDist = (sideDistX - deltaDistX);
    else           perpWallDist = (sideDistY - deltaDistY);

    if (perpWallDist < 0.1f) perpWallDist = 0.1f;
    zBuffer[x] = perpWallDist;

    int lineHeight = int(viewH / perpWallDist);
    int drawStart = -lineHeight / 2 + viewH / 2;
    if (drawStart < 0) drawStart = 0;
    int drawEnd = lineHeight / 2 + viewH / 2;
    if (drawEnd >= viewH) drawEnd = viewH - 1;

    uint16_t color;
    switch (worldMap[mapX][mapY]) {
      case 1: color = (side == 1) ? 0x8800 : 0xC000; break;
      case 2: color = (side == 1) ? 0x0010 : 0x031F; break;
      case 3: color = (side == 1) ? 0x0300 : 0x05E0; break;
      default:color = (side == 1) ? 0x9300 : 0xDE00; break;
    }
    if (perpWallDist > 4.5f) color = (color >> 1) & 0x7BEF;
    canvasGfx->drawFastVLine(x, drawStart, drawEnd - drawStart + 1, color);
  }

  // --- 5. enemy sprites ---
  for (int i = 0; i < NUM_ENEMIES; i++) {
    if (!enemies[i].alive) continue;
    float spriteX = enemies[i].x - posX;
    float spriteY = enemies[i].y - posY;

    float invDet = 1.0f / (planeX * dirY - dirX * planeY);
    float transformX = invDet * (dirY * spriteX - dirX * spriteY);
    float transformY = invDet * (-planeY * spriteX + planeX * spriteY);

    if (transformY > 0.2f) {
      int spriteScreenX = int((screenW / 2) * (1.0f + transformX / transformY));
      int spriteSize = abs(int(viewH / transformY));
      int drawStartY = max(0, -spriteSize / 2 + viewH / 2);
      int drawEndY = min(viewH - 1, spriteSize / 2 + viewH / 2);
      int drawStartX = max(0, -spriteSize / 2 + spriteScreenX);
      int drawEndX = min(screenW - 1, spriteSize / 2 + spriteScreenX);

      for (int stripe = drawStartX; stripe <= drawEndX; stripe++) {
        if (transformY < zBuffer[stripe]) {
          int texX = (stripe - (-spriteSize / 2 + spriteScreenX)) * 10 / spriteSize;
          if (texX < 0) texX = 0; else if (texX > 9) texX = 9;

          for (int y = drawStartY; y <= drawEndY; y++) {
            int texY = (y - (-spriteSize / 2 + viewH / 2)) * 10 / spriteSize;
            if (texY < 0) texY = 0; else if (texY > 9) texY = 9;

            uint8_t pixel = demonSprite[texY][texX];
            if (pixel != 0) {
              uint16_t color;
              if (enemies[i].hurtTimer > 0) {
                color = 0xFFFF;
              } else {
                switch(pixel) {
                  case 1: color = 0x8800; break;
                  case 2: color = 0xF800; break;
                  case 3: color = 0xFFE0; break;
                  case 4: color = 0xEF5D; break;
                  default: color = 0x1800; break;
                }
                if (transformY > 4.0f) color = (color >> 1) & 0x7BEF;
              }
              canvasGfx->drawPixel(stripe, y, color);
            }
          }
        }
      }
    }
  }

  // --- 6. weapon Sprite ---
  int gunBobX = int(cosf(walkCycle) * 4.0f);
  int gunBobY = int(fabsf(sinf(walkCycle)) * 5.0f);
  int gunBaseX = 160 + gunBobX;
  int gunBaseY = 138 + gunBobY;

  if (firingFrame > 0) {
    gunBaseY += 12;
    if (firingFrame < 3) {
      canvasGfx->fillCircle(gunBaseX, gunBaseY - 32, 18, 0xFFE0);
      canvasGfx->fillCircle(gunBaseX, gunBaseY - 32, 10, 0xFFFF);
    }
    firingFrame++;
    if (firingFrame > 4) firingFrame = 0;
  }

  canvasGfx->fillRect(gunBaseX - 10, gunBaseY - 22, 9, 24, 0x4208);
  canvasGfx->fillRect(gunBaseX + 1,  gunBaseY - 22, 9, 24, 0x4208);
  canvasGfx->fillRect(gunBaseX - 7,  gunBaseY - 26, 5, 5,  0x0000);
  canvasGfx->fillRect(gunBaseX + 3,  gunBaseY - 26, 5, 5,  0x0000);
  canvasGfx->fillRect(gunBaseX - 14, gunBaseY - 6,  28, 16, 0x6180);

  // --- 7. status Bar HUD ---
  canvasGfx->fillRect(0, 139, 320, 31, 0x3186);
  canvasGfx->drawFastHLine(0, 139, 320, 0x7BEF);
  canvasGfx->drawFastHLine(0, 140, 320, 0x0000);

  canvasGfx->setTextSize(1);
  canvasGfx->setTextColor(0xF800);
  canvasGfx->setCursor(10, 150);
  canvasGfx->print("HP ");
  canvasGfx->setTextColor(0xFFFF);
  canvasGfx->print(health);
  canvasGfx->print("%");

  canvasGfx->setTextColor(0xFFE0);
  canvasGfx->setCursor(80, 150);
  canvasGfx->print("AMMO ");
  canvasGfx->setTextColor(0xFFFF);
  canvasGfx->print(ammo);

  canvasGfx->setTextColor(0x07E0);
  canvasGfx->setCursor(160, 150);
  canvasGfx->print("KILLS ");
  canvasGfx->setTextColor(0xFFFF);
  canvasGfx->print(kills);

  canvasGfx->setTextColor(0x9CD3);
  canvasGfx->setCursor(240, 150);
  if (imuFound) {
    canvasGfx->print("TILT ");
    canvasGfx->print(sdaPin);
    canvasGfx->print(",");
    canvasGfx->print(sclPin);
  } else {
    canvasGfx->print("AUTOPILOT");
  }

  // --- 8. mini Radar Map ---
  for (int my = 0; my < MAP_H; my++) {
    for (int mx = 0; mx < MAP_W; mx++) {
      if (worldMap[mx][my] > 0) {
        canvasGfx->drawPixel(260 + mx * 3, 6 + my * 3, 0x7BEF);
      }
    }
  }
  for (int i = 0; i < NUM_ENEMIES; i++) {
    if (enemies[i].alive) {
      canvasGfx->drawPixel(260 + int(enemies[i].x * 3), 6 + int(enemies[i].y * 3), 0xFD20);
    }
  }
  canvasGfx->fillCircle(260 + int(posX * 3), 6 + int(posY * 3), 1, 0x07E0);

  canvasGfx->flush();

  frameCount++;
  if (millis() - lastFpsTime >= 1000) {
    fps = frameCount;
    frameCount = 0;
    lastFpsTime = millis();
  }
  delay(5);
}