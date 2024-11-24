const int pinEntradaRS = 13; // entra R-S
float muestreoFrec;
const int pinesSalida[] = {33, 26, 21, 15, 23, 32}; // FORMA T1 T2 T3 T4 T5 T6
//const int pinesSalida[] = {33, 32, 26, 15, 21, 23};  // T1 T3 T5 T4 T6 T2
const int pin3v3 = 14;
float frecuenciaMedida;
int i = 0;

int frec_clock=8000000;
hw_timer_t *Timer0_Cfg = NULL;
hw_timer_t *timer = NULL;

volatile bool triggered = false;
float periodo;
float alfaAng = 0;
uint64_t alfa = 0;
const float Pi = 3.14159;

#define SENSOR_PIN 36
#define ADC_RESOLUTION 4095
#define VREF 3.3
#define SAMPLING_INTERVAL 100

#define POT_PIN 25 // Pin ADC de la ESP32 conectado al cursor del potenciómetro
#define VCC 3.3    // Voltaje de referencia (3.3V en ESP32)
#define MAX_ANGLE 60.0  // Máximo ángulo en grados
#define PERIOD_MS 20.0  // Tiempo de un período completo (360°) en milisegundos

volatile bool calculateIntegral = false;
volatile float integralValue = 0;
unsigned int sampleCount = 0;


void IRAM_ATTR ISR() {
    detachInterrupt(digitalPinToInterrupt(pinEntradaRS));  // Deshabilitamos la interrupción
    if (!triggered) {
        triggered = true;
        timerStart(timer);  // Iniciamos el temporizador una sola vez
        Serial.println("Timer started");
    }

    // Actualizamos los pines
    digitalWrite(pinesSalida[0], LOW);
    digitalWrite(pinesSalida[1], HIGH);
    digitalWrite(pinesSalida[5], HIGH);
    
    // Modificamos el valor de i
    i = 1;

}

float medirFrecuencia() {
    float valorActual = analogRead(pinEntradaRS) * (3.3 / 4095.0);
    unsigned long tiempoInicio;
    bool flag = false;
    while (valorActual > 0) {
        valorActual = analogRead(pinEntradaRS) * (3.3 / 4095.0);
    }
    while (valorActual == 0) {
        valorActual = analogRead(pinEntradaRS) * (3.3 / 4095.0);
        if (!flag) {
            tiempoInicio = micros();
            flag = true;
        }
        if (valorActual > 0) {
            unsigned long tiempoFinal = micros();
            float periodo = (tiempoFinal - tiempoInicio) * 2;
            return 1000000.0 / periodo;
        }
    }
    return 0;
}


void IRAM_ATTR onTimer() {
    // Actualizamos la lógica de pines en cada evento del temporizador
    switch (i) {
        case 1: // estan 2 y 3
            digitalWrite(pinesSalida[5], LOW);
            digitalWrite(pinesSalida[3], HIGH);
            Serial.println(i);
            i++;
            calculateIntegral = true;
            break;
        case 2: // estan 3 y 4
            digitalWrite(pinesSalida[1], LOW);
            digitalWrite(pinesSalida[2], HIGH);
            Serial.println(i);
            i++;
            calculateIntegral = true;
            break;
        case 3: // estan 4 y 5
            digitalWrite(pinesSalida[3], LOW);
            digitalWrite(pinesSalida[4], HIGH);
            Serial.println(i);
            i++;
            calculateIntegral = true;
            break;
        case 4:// estan 5 y 6
            digitalWrite(pinesSalida[2], LOW);
            digitalWrite(pinesSalida[0], HIGH);
            Serial.println(i);
            i++;
            calculateIntegral = true;
            break;
        case 5:// estan 6 y 1
            digitalWrite(pinesSalida[4], LOW);
            digitalWrite(pinesSalida[5], HIGH);
            Serial.println(i);
            attachInterrupt(digitalPinToInterrupt(pinEntradaRS), ISR, RISING);
            triggered=false;
            timerStop(timer);  // Aseguramos que el temporizador esté detenido 
            i = 0;
            calculateIntegral = true;
            break;
        case 0:
            digitalWrite(pinesSalida[0], LOW);
            digitalWrite(pinesSalida[4], LOW);
            digitalWrite(pinesSalida[5], LOW);
            Serial.println(i);
            i++;
            calculateIntegral = true;
            break;
    }
}

void setup() {
    pinMode(pinEntradaRS, INPUT);
    pinMode(pin3v3, OUTPUT);
    for (int l = 0; l < 6; l++) {
        pinMode(pinesSalida[l], OUTPUT);
    }
    digitalWrite(pin3v3, HIGH);
    pinMode(POT_PIN, INPUT); // Inicializar el pin como entrada
    analogReadResolution(12); // Resolución de 12 bits para el ADC (0 a 4095)
    Serial.begin(115200);

    /*int j;
    for (j = 0; j < 10; j++) {
        muestreoFrec += medirFrecuencia();
    }
    frecuenciaMedida = muestreoFrec / j;
    periodo = 1 / frecuenciaMedida;
    Serial.print("Frecuencia: ");
    Serial.print(frecuenciaMedida);
    Serial.println(" Hz");*/

    attachInterrupt(digitalPinToInterrupt(pinEntradaRS), ISR, RISING);

    // Configuramos el temporizador correctamente
    frec_clock=1000000;
  
    
    timer = timerBegin(frec_clock);  // Temporizador 0, prescaler 80 (1 MHz)
    timerAttachInterrupt(timer, &onTimer);  // Asociamos la función onTimer
    timerAlarm(timer, 3333, true,0);  // en microsegundos (3.3333ms)
    timerStop(timer);  // Aseguramos que el temporizador esté detenido inicialmente
}

void variarAlfa() {
  int rawValue = analogRead(POT_PIN); // Leer el valor analógico del potenciómetro
  float voltage = (rawValue / 4095.0) * VCC; // Convertir el valor a voltaje

  // Calcular el ángulo (0° a 60°) basado en el voltaje medido
  float angle = (voltage / VCC) * MAX_ANGLE;

  // Calcular el tiempo correspondiente (20 ms para 360°)
  float time_ms = (angle / 360.0) * PERIOD_MS;
  alfa = time_ms*1000; // en microsegundos

}

void variarTimer(int alfa)
{
    uint64_t newTimerPeriod = 3333 + (uint64_t)(alfa); 
    timerDetachInterrupt(timer);
    timerEnd(timer);
    timer = timerBegin(frec_clock);  // Temporizador 0, prescaler 80 (1 MHz)
    timerAttachInterrupt(timer, &onTimer);  // Asociamos la función onTimer
    timerAlarm(timer, newTimerPeriod, true,0);  // en microsegundos (3.3333ms)
    Serial.println("Se cambio el timer a: ");
    Serial.print(newTimerPeriod);
    Serial.println("");
    
}
void loop() {
  delay(200);
  if(triggered)
  {
  variarAlfa();
  variarTimer(alfa);
  }
    /*if (calculateIntegral) {
        calculateIntegral = false;
        Serial.print("Integral calculada: ");
        Serial.println(integralValue);
        integralValue = 0;
    }*/
}

/*void sampleAndIntegrate() {
    unsigned long startTime = micros();  // Tiempo de inicio
    unsigned long currentTime;
    float sum = 0;
    sampleCount = 0;

    while (!calculateIntegral) {  // Muestreo hasta que la interrupción se dispare
        currentTime = micros();  // Tiempo actual
        if (currentTime - startTime >= SAMPLING_INTERVAL) {
            startTime = currentTime;  // Actualizar el tiempo de referencia
            int adcValue = analogRead(SENSOR_PIN);  // Leer señal analógica
            float voltage = (adcValue * VREF) / ADC_RESOLUTION;  // Convertir a voltaje
            sum += voltage * (SAMPLING_INTERVAL / 1000000.0);  // Suma incremental para la integral
            sampleCount++;
        }
    }
    integralValue = sum;  // Guardar el resultado final de la integral
}

float calculateRMS() {
    unsigned long sumOfSquares = 0;  // Suma de los cuadrados
    for (unsigned int i = 0; i < sampleCount; i++) {
        int adcValue = analogRead(SENSOR_PIN);  // Leer valor ADC
        float voltage = (adcValue * VREF) / ADC_RESOLUTION;  // Convertir ADC a voltaje
        sumOfSquares += voltage * voltage;  // Sumar el cuadrado del voltaje
        delayMicroseconds(200);  // Retardo entre muestras (frecuencia adecuada para CA)
    }
    return sqrt(sumOfSquares / sampleCount);  // Calcular RMS
}*/
