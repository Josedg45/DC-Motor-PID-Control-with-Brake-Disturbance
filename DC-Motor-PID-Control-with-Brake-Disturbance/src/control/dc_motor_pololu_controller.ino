// === Configuración de pines ===
const int pwmPin = 9;        // PWM al driver
const int dirPin = 8;        // Dirección del motor
const int encoderA = 2;      // Canal A del encoder
const int encoderB = 3;      // Canal B del encoder

// === Variables del encoder ===
volatile long encoderCount = 0;
int lastEncoded = 0;

// === Parámetros del motor ===
const int pulsesPerRev = 64;    // Pulsos por revolución del motor
const int gearRatio = 90;       // Relación de reducción
const float pulsesPerOutputRev = pulsesPerRev * gearRatio;

// === Variables de tiempo ===
unsigned long lastTime = 0;
const unsigned long sampleTime = 10; // [ms]

// === Variables de control ===
float rpm_raw = 0;
float rpm = 0;
float setpoint = 0; // RPM objetivo inicial

// === Media móvil ===
const int WINDOW_DEFAULT = 10;
const int WINDOW_MAX = 200;

int windowSamples = WINDOW_DEFAULT;
float ma_buffer[WINDOW_MAX];
int ma_index = 0;
float ma_sum = 0.0;
bool ma_initialized = false;

// === Controlador discreto ===
float q0 = 3.37592492110210;
float q1 = -4.73243722690591;
float q2 = 1.41598845554392;

float e_k = 0, e_k_1 = 0, e_k_2 = 0;
float u_k = 0, u_k_1 = 0;

// === PWM ===
int pwmValue = 0;

void setup() {
  pinMode(pwmPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(encoderA, INPUT_PULLUP);
  pinMode(encoderB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoderA), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderB), updateEncoder, CHANGE);

  Serial.begin(115200);  // Compatible con MATLAB

  analogWrite(pwmPin, 255 - pwmValue); // PWM invertido
  digitalWrite(dirPin, HIGH);

  for (int i = 0; i < WINDOW_MAX; ++i) ma_buffer[i] = 0.0;

  Serial.println("=== Control de Velocidad (Media Móvil) ===");
  Serial.println("Comandos:");
  Serial.println("  S<number>   -> Cambia el setpoint (ej: S150)");
  Serial.println("  M<number>   -> Cambia ventana media móvil (ej: M10)");
  Serial.println("----------------------------------------------------");
}

void loop() {
  // --- Lectura de comandos seriales ---
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'S') {
      float newSetpoint = Serial.parseFloat();
      if (newSetpoint >= 0 && newSetpoint <= 500) {
        setpoint = newSetpoint;
        Serial.print("Nuevo setpoint: ");
        Serial.println(setpoint);
      }
    } else if (cmd == 'M') {
      int newW = Serial.parseInt();
      if (newW < 0) newW = 0;
      if (newW > WINDOW_MAX) newW = WINDOW_MAX;
      setMovingAverageWindow(newW);
      Serial.print("Nuevo windowSamples: ");
      Serial.println(windowSamples);
    }
    while (Serial.available()) Serial.read(); // limpiar buffer
  }

  // --- Cálculo de velocidad ---
  unsigned long now = millis();
  if (now - lastTime >= sampleTime) {
    noInterrupts();
    long count = encoderCount;
    encoderCount = 0;
    interrupts();

    float revs = (float)count / pulsesPerOutputRev;
    float rps = revs / ((float)sampleTime / 1000.0);
    rpm_raw = rps * 60.0;

    // === Filtro de media móvil ===
    if (windowSamples <= 0) {
      rpm = rpm_raw;
    } else {
      if (!ma_initialized) {
        for (int i = 0; i < windowSamples; ++i) ma_buffer[i] = rpm_raw;
        ma_sum = rpm_raw * (float)windowSamples;
        ma_index = 0;
        ma_initialized = true;
      } else {
        ma_sum -= ma_buffer[ma_index];
        ma_buffer[ma_index] = rpm_raw;
        ma_sum += ma_buffer[ma_index];
        ma_index++;
        if (ma_index >= windowSamples) ma_index = 0;
      }
      rpm = ma_sum / (float)windowSamples;
    }

    // --- Control discreto ---
    e_k = setpoint - rpm;
    u_k = q0 * e_k + q1 * e_k_1 + q2 * e_k_2 + u_k_1;

    if (u_k > 255) u_k = 255;
    if (u_k < 0)   u_k = 0;

    pwmValue = (int)u_k;
    if (setpoint != 0) analogWrite(pwmPin, 255 - pwmValue);
    else pwmValue = 0;

    e_k_2 = e_k_1;
    e_k_1 = e_k;
    u_k_1 = u_k;

    // --- Envío a MATLAB (CSV) ---
    // Formato: setpoint, control(u), rpm
    Serial.print(setpoint);
    Serial.print(",");
    Serial.print(u_k);   // señal de control continua (no pwmValue entero)
    Serial.print(",");
    Serial.println(rpm);

    lastTime = now;
  }
}

void updateEncoder() {
  int MSB = digitalRead(encoderA);
  int LSB = digitalRead(encoderB);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
    encoderCount++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
    encoderCount--;

  lastEncoded = encoded;
}

void setMovingAverageWindow(int newW) {
  if (newW < 0) newW = 0;
  if (newW > WINDOW_MAX) newW = WINDOW_MAX;
  if (newW == windowSamples) return;

  if (newW == 0) {
    ma_initialized = false;
    ma_sum = 0.0;
    ma_index = 0;
  } else {
    for (int i = 0; i < newW; ++i) ma_buffer[i] = rpm;
    ma_sum = rpm * (float)newW;
    ma_index = 0;
    ma_initialized = true;
  }
  windowSamples = newW;
}
