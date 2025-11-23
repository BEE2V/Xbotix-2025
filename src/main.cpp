#include <Adafruit_NeoPixel.h>

#define LED_PIN 3
#define LED_COUNT 1
#define LDR_PIN A0

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

const int settleTime = 30;

int redRef;   // LDR reading when showing a RED object
int greenRef; // LDR reading when showing a GREEN object

int readColor(int r, int g, int b)
{
  strip.setPixelColor(0, strip.Color(r, g, b));
  strip.show();
  delay(settleTime);
  return analogRead(LDR_PIN);
}

void countDown()
{
  Serial.println("3");
  delay(400);
  Serial.println("2");
  delay(400);
  Serial.println("1");
  delay(400);
}

void setup()
{
  Serial.begin(115200);
  strip.begin();
  strip.show();

  Serial.println("=== RED vs GREEN Calibration ===");

  // -------------------------
  //   CALIBRATE RED OBJECT
  // -------------------------
  Serial.println("Show RED object...");
  countDown();

  // read how object reflects RED light
  redRef = readColor(255, 0, 0);
  strip.clear();
  strip.show();

  Serial.print("Red reference = ");
  Serial.println(redRef);
  Serial.println();

  delay(700);

  // -------------------------
  //   CALIBRATE GREEN OBJECT
  // -------------------------
  Serial.println("Show GREEN object...");
  countDown();

  // read how object reflects GREEN light
  greenRef = readColor(0, 255, 0);
  strip.clear();
  strip.show();

  Serial.print("Green reference = ");
  Serial.println(greenRef);
  Serial.println();

  Serial.println("Calibration finished.");
  delay(1000);
}

void loop()
{

  // Read reflection for RED light
  int redValue = readColor(255, 0, 0);

  // Read reflection for GREEN light
  int greenValue = readColor(0, 255, 0);

  strip.clear();
  strip.show();

  // Compare: which light reflects more?
  // If object is red → strong red reflection
  // If object is green → strong green reflection
  String detected;

  if (abs(redValue - redRef) < abs(greenValue - greenRef))
  {
    detected = "RED";
  }
  else
  {
    detected = "GREEN";
  }

  Serial.print("Rval=");
  Serial.print(abs(redValue - redRef));
  Serial.print("  Gval=");
  Serial.print(abs(greenValue - greenRef));
  Serial.print("  → Detected: ");
  Serial.println(detected);

  delay(300);
}
