```cpp
#include <Adafruit_NeoPixel.h>

#define LED_PIN 3
#define LED_COUNT 1
#define LDR_PIN A0

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

const int settleTime = 30;

// Calibration values
int rHigh, gHigh, bHigh;
int rLow, gLow, bLow;

// --- helper to light color and read ---
int readColor(int r, int g, int b)
{
  strip.setPixelColor(0, strip.Color(r, g, b));
  strip.show();
  delay(settleTime);
  return analogRead(LDR_PIN);
}

// --- countdown print ---
void countDown()
{
  Serial.println("3");
  delay(500);
  Serial.println("2");
  delay(500);
  Serial.println("1");
  delay(500);
}

void setup()
{
  Serial.begin(115200);
  strip.begin();
  strip.show();

  // ============================
  //        CALIBRATION
  // ============================

  Serial.println("Calibration starts...");
  delay(500);

  // ----- WHITE surface → HIGH -----
  Serial.println("Show WHITE object...");
  Serial.print("Starting in ");
  countDown();

  rHigh = readColor(255, 0, 0);
  gHigh = readColor(0, 255, 0);
  bHigh = readColor(0, 0, 255);
  strip.setPixelColor(0, 0, 0, 0);
  strip.show();

  Serial.println("HIGH values:");
  Serial.print("R: ");
  Serial.println(rHigh);
  Serial.print("G: ");
  Serial.println(gHigh);
  Serial.print("B: ");
  Serial.println(bHigh);
  Serial.println();

  delay(1000);

  // ----- BLACK surface → LOW -----
  Serial.println("Show BLACK object...");
  Serial.print("Starting in ");
  countDown();

  rLow = readColor(255, 0, 0);
  gLow = readColor(0, 255, 0);
  bLow = readColor(0, 0, 255);
  strip.setPixelColor(0, 0, 0, 0);
  strip.show();

  Serial.println("LOW values:");
  Serial.print("R: ");
  Serial.println(rLow);
  Serial.print("G: ");
  Serial.println(gLow);
  Serial.print("B: ");
  Serial.println(bLow);
  Serial.println();

  Serial.println("Calibration complete!");
  delay(1000);
}

void loop()
{
  // raw readings
  int rRaw = readColor(255, 0, 0);
  int gRaw = readColor(0, 255, 0);
  int bRaw = readColor(0, 0, 255);

  strip.setPixelColor(0, 0, 0, 0);
  strip.show();

  // mapped values (0–255)
  int r = map(rRaw, rLow, rHigh, 0, 255);
  int g = map(gRaw, gLow, gHigh, 0, 255);
  int b = map(bRaw, bLow, bHigh, 0, 255);

  // clamp
  r = constrain(r, 0, 255);
  g = constrain(g, 0, 255);
  b = constrain(b, 0, 255);

  Serial.print("R=");
  Serial.print(r);
  Serial.print("  G=");
  Serial.print(g);
  Serial.print("  B=");
  Serial.println(b);

  delay(200);
}


```