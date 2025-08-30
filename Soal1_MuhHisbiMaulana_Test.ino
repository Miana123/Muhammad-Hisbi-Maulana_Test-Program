// soal1_led_nonblocking.ino
// 5 LED dengan interval berbeda (non-blocking, millis)
// Pin bisa diubah sesuai kebutuhan board

struct LedTask {
  uint8_t pin;
  uint32_t intervalMs;
  uint32_t lastToggle;
  bool state;
};

LedTask leds[] = {
  {2,  270,  0, false},   // Merah
  {3,  440,  0, false},   // Kuning
  {4,  710,  0, false},   // Hijau
  {5,  1330, 0, false},   // Biru
  {6,  1850, 0, false}    // Putih
};

const size_t LED_COUNT = sizeof(leds) / sizeof(leds[0]);

void setup() {
  for (size_t i = 0; i < LED_COUNT; ++i) {
    pinMode(leds[i].pin, OUTPUT);
    digitalWrite(leds[i].pin, LOW);
    leds[i].lastToggle = millis();
    leds[i].state = false;
  }
}

void loop() {
  uint32_t now = millis();
  for (size_t i = 0; i < LED_COUNT; ++i) {
    LedTask &lt = leds[i];
    if (now - lt.lastToggle >= lt.intervalMs) {
      lt.state = !lt.state;
      digitalWrite(lt.pin, lt.state ? HIGH : LOW);
      lt.lastToggle = now;
    }
  }
}
