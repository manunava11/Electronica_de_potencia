const int pinEntradaRS = 22; // entra R-S
float muestreoFrec;
const int pinesSalida[] = {34, 35, 26, 15, 21, 23};
float frecuenciaMedida;
int i = 0;

hw_timer_t *timer = NULL;
volatile bool triggered = false;
const int pinAlfa = 12;
float tensionAlfa;
float periodo;
float alfaAng = 0;
uint64_t alfa = 0;
const float Pi = 3.14159;

#define SENSOR_PIN 36
#define ADC_RESOLUTION 4095
#define VREF 3.3
#define SAMPLING_INTERVAL 100

volatile bool calculateIntegral = false;
volatile float integralValue = 0;
unsigned int sampleCount = 0;

void IRAM_ATTR onTimer() {
    switch (i) {
        case 1:
            digitalWrite(pinesSalida[0], LOW);
            digitalWrite(pinesSalida[2], HIGH);
            i++;
            calculateIntegral = true;
            break;
        case 2:
            digitalWrite(pinesSalida[1], LOW);
            digitalWrite(pinesSalida[3], HIGH);
            i++;
            calculateIntegral = true;
            break;
        case 3:
            digitalWrite(pinesSalida[2], LOW);
            digitalWrite(pinesSalida[4], HIGH);
            i++;
            calculateIntegral = true;
            break;
        case 4:
            digitalWrite(pinesSalida[3], LOW);
            digitalWrite(pinesSalida[5], HIGH);
            i++;
            calculateIntegral = true;
            break;
        case 5:
            digitalWrite(pinesSalida[4], LOW);
            digitalWrite(pinesSalida[0], HIGH);
            i = 0;
            calculateIntegral = true;
            break;
        case 0:
            digitalWrite(pinesSalida[1], HIGH);
            digitalWrite(pinesSalida[5], LOW);
            i++;
            calculateIntegral = true;
            break;
    }
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

void variarAlfa() {
    tensionAlfa = analogRead(pinEntradaRS) * (3.3 / 4095.0);
    alfaAng = 243.24 * (tensionAlfa - 2.49);
    alfa = alfaAng * 2 * Pi / 360;
    Serial.println(alfa);
}

void setup() {
    for (int l = 0; l < 6; l++) {
        pinMode(pinesSalida[l], OUTPUT);
    }
    Serial.begin(115200);
    pinMode(pinEntradaRS, INPUT);

    int j;
    for (j = 0; j < 10; j++) {
        muestreoFrec += medirFrecuencia();
    }
    frecuenciaMedida = muestreoFrec / j;
    periodo = 1 / frecuenciaMedida;
    Serial.print("Frecuencia: ");
    Serial.print(frecuenciaMedida);
    Serial.println(" Hz");

    timer = timerBegin(80);
    timerAttachInterrupt(timer, &onTimer);
    timerAlarm(timer, 3333 + alfa, true,0);
    timerStop(timer);
}

void loop() {
    variarAlfa();
    timerAlarm(timer, 3333 + alfa, true,0);

    /*if (calculateIntegral) {
        calculateIntegral = false;
        Serial.print("Integral calculada: ");
        Serial.println(integralValue);
        integralValue = 0;
    }*/
}

void ISR() {
    if (!triggered) {
        triggered = true;
        timerStart(timer);
    }
    digitalWrite(pinesSalida[0], HIGH);
    digitalWrite(pinesSalida[1], HIGH);
    digitalWrite(pinesSalida[5], LOW);
    i = 1;
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
