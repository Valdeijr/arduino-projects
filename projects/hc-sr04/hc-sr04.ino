long valorDoSensor(int trigPin, int echoPin) {
  pinMode(trigPin, OUTPUT);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  pinMode(echoPin, INPUT);
  long distance = pulseIn(echoPin, HIGH) * 0.01723;
  return distance;
}
  
void setup() {
  Serial.begin(9600);
}

void loop() {
  Serial.print(valorDoSensor(5,6));
  Serial.print("\n");
}