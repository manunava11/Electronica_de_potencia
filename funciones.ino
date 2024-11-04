// Función para enviar una señal digital
void enviarDigital(int pin, bool estado) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, estado ? HIGH : LOW);
}

// Función para medir una señal digital
bool medirDigital(int pin) {
    pinMode(pin, INPUT);
    return digitalRead(pin) == HIGH;
}

// Función para enviar una señal analógica (PWM)
void enviarAnalogica(int pin, int valor) {
    pinMode(pin, OUTPUT);
    analogWrite(pin, valor); // valor entre 0 y 255 para PWM
}

// Función para medir una señal analógica
int medirAnalogica(int pin) {
    pinMode(pin, INPUT);
    return analogRead(pin); // valor entre 0 y 1023
}

float medirFrecuencia(int pin) {
    unsigned long t1, t2;
    
    // Esperar a que la señal suba 
    while (digitalRead(pin) == LOW);
    t1 = micros(); // tiempo del primer flanco ascendente

    // Esperar a que la señal vuelva a subir
    while (digitalRead(pin) == HIGH);
    while (digitalRead(pin) == LOW);
    t2 = micros(); // tiempo del segundo flanco ascendente

    // Calcular el período y luego la frecuencia
    float periodo = (t2 - t1) / 1e6; // en segundos
    return 1 / periodo; // en Hz
}


void ajustarAnguloDisparo(int pin, int angulo) {
    int delayTiempo = map(angulo, 0, 180, 0, 10000); // Convertir ángulo a microsegundos
    digitalWrite(pin, HIGH);
    delayMicroseconds(delayTiempo);
    digitalWrite(pin, LOW);
}

float calcularRMS(int pin, int muestras, float deltaT) {
    float sumaCuadrados = 0;
    for (int i = 0; i < muestras; i++) {
        int valor = analogRead(pin);
        float voltaje = (valor * 5.0) / 1023.0; // Convertir a voltios
        sumaCuadrados += voltaje * voltaje;
        delayMicroseconds(deltaT * 1e6);
    }
    return sqrt(sumaCuadrados / muestras);
}

float calcularEficiencia(float potenciaSalida, float potenciaEntrada) {
    // mandarle ya de entrada las Vodc*Vodc/R   y  Integral(v*i)
    if (potenciaEntrada == 0) return 0;
    return (potenciaSalida / potenciaEntrada) * 100.0; 
}

// Definición de pines
const int pinFrecuencia = 2;  // Pin para medir frecuencia
const int pinIntegral = A0;   // Pin para muestrear la señal analógica e integrar

// Parámetros para la integración
float deltaT = 0.01; // Paso de muestreo en segundos (10 ms)
int muestras = 100;  // Número de muestras para la integración


float medirFrecuencia(int pin) {
    unsigned long t1, t2;

    // Esperar a que la señal suba (flanco ascendente)
    while (digitalRead(pin) == LOW);
    t1 = micros(); // tiempo del primer flanco ascendente

    // Esperar a que la señal vuelva a subir
    while (digitalRead(pin) == HIGH);
    while (digitalRead(pin) == LOW);
    t2 = micros(); // tiempo del segundo flanco ascendente

    // Calcular el período y luego la frecuencia
    float periodo = (t2 - t1) / 1e6; // en segundos
    return 1 / periodo; // en Hz
}

float integrarSenal(int pin, float deltaT, int muestras) {
   float integral = 0.0;

    for (int i = 0; i < muestras; i++) {
        int valorActual = analogRead(pin);
        float voltaje = (valorActual * 5.0) / 1023.0; // Convertir a voltios
        // Suma de Riemann: suma del área de cada rectángulo
        integral += voltaje * deltaT;

        delayMicroseconds(deltaT * 1e6); // Espera deltaT en microsegundos
    }

    return integral; 
}

void setup() {
    Serial.begin(9600);
    pinMode(pinFrecuencia, INPUT);
    pinMode(pinIntegral, INPUT);
}

void loop() {
 
    float frecuencia = medirFrecuencia(pinFrecuencia);
    Serial.print("Frecuencia: ");
    Serial.print(frecuencia);
    Serial.println(" Hz");

    // Integral

    float integral = integrarSenal(pinIntegral, deltaT, muestras);

    Serial.print("Integral acumulada: ");
    Serial.print(integral);
    Serial.println(" V*s");

    delay(100); 
}
