// PINs CONFIGURATION
#define RED_1 10
#define YELLOW_1 9
#define GREEN_1 8
#define RED_2 7
#define YELLOW_2 6
#define GREEN_2 5
#define PEDESTRIAN_BUTTON 4 

// CLASSE DOS LEDs
class TrafficLight {
  public: 
    enum Phase { RED, GREEN, YELLOW } ;

  private:
    uint8_t redPin, yellowPin, greenPin;
    uint32_t lastChangeTime = 0;
    uint32_t greenDuration;
    uint32_t redDuration;
    uint32_t yellowDuration;
    Phase currentPhase;

  public:
    TrafficLight(uint8_t red, uint8_t yellow, uint8_t green, uint32_t interval = 5000)
      : redPin(red), yellowPin(yellow), greenPin(green) {
      greenDuration = interval * 0.75;  // RAZÃO DE 75% DO TEMPO EM VERDE
      yellowDuration = interval * 0.25; // RAZÃO DE 25% DO TEMPO EM TRANSIÇÃO PARA VERMELHO
      redDuration = interval;
      pinMode(redPin, OUTPUT);
      pinMode(yellowPin, OUTPUT);
      pinMode(greenPin, OUTPUT);
      setPhase(RED);
    }

    void setPhase(Phase phase) {
      currentPhase = phase;
      lastChangeTime = millis();
      digitalWrite(redPin, phase == RED ? HIGH : LOW);
      digitalWrite(yellowPin, phase == YELLOW ? HIGH : LOW);
      digitalWrite(greenPin, phase == GREEN ? HIGH : LOW);
    }

    void startCycle(Phase initialPhase) {
      setPhase(initialPhase);
    }

    void update() {
      uint32_t now = millis();
      switch (currentPhase) {
        case GREEN:
          if (now - lastChangeTime >= greenDuration) {
            setPhase(YELLOW);
          }
          break;
        case YELLOW:
          if (now - lastChangeTime >= yellowDuration) {
            setPhase(RED);
          }
          break;
        case RED:
          if (now - lastChangeTime >= redDuration) {
            setPhase(GREEN); 
          }
          break;
      }
    }

    bool isRed() const {
      return currentPhase == RED;
    }

    void forceRed() {
      setPhase(RED);
    }
};

// CLASSE DE CONTROLE
class TrafficSystem {
  private:
    TrafficLight light1;
    TrafficLight light2;
    bool pedestrianHold = false;
    uint32_t holdStartTime = 0;
    const uint32_t holdDuration = 5000;

  public:
    TrafficSystem(uint8_t red1, uint8_t yellow1, uint8_t green1,
                  uint8_t red2, uint8_t yellow2, uint8_t green2)
      : light1(red1, yellow1, green1, 5000), 
        light2(red2, yellow2, green2, 5000) {
      init();
    }

    void init(){
      light1.startCycle(TrafficLight::GREEN);
      light2.startCycle(TrafficLight::GREEN;
    }

    void triggerPedestrianHold() {
      pedestrianHold = true;
      holdStartTime = millis();
      light1.forceRed();
      light2.forceRed();
    }

    void update() {
      if (pedestrianHold) {
        if (millis() - holdStartTime >= holdDuration) {
          pedestrianHold = false;
          init();
        }
        return;
      }
      light1.update();
      light2.update();
    }
};

// CRIA OBJETO
TrafficSystem trafficSystem(RED_1, YELLOW_1, GREEN_1, RED_2, YELLOW_2, GREEN_2);

// LOOP ARDUINO
void setup() {
   
} 

void loop() {
  if (digitalRead(PEDESTRIAN_BUTTON) == LOW) { 
    trafficSystem.triggerPedestrianHold();
    delay(100);
  }
  trafficSystem.update();
  delay(100); 
}