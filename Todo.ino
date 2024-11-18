const int pinEntradaRS = 4;        // entra R-S
float muestreoFrec;
const int pinesSalida[] = {13,14,15,16,17,18};
float frecuenciaMedida;
int i=0;

hw_timer_t *timer = NULL;  // Temporizador de hardware
volatile bool triggered = false;  // Bandera para indicar la interrupción inicial
const int pinAlfa;
float tensionAlfa;
float periodo;
float alfaAng=0;
float alfa=0;
const float Pi = 3.14159;

#define SENSOR_PIN 36  // Pin analógico para la señal
#define ADC_RESOLUTION 4095  // Resolución ADC (12 bits)
#define VREF 3.3  // Voltaje de referencia de la ESP32
#define SAMPLING_INTERVAL 100  // Intervalo de muestreo en microsegundos

volatile bool calculateIntegral = false;  // Bandera para indicar cuándo calcular la integral
volatile float integralValue = 0;  // Resultado de la integral
unsigned int sampleCount = 0;  // Número de muestras tomadas

void setup() {
    pinMode(pinSalida, OUTPUT);
    Serial.begin(115200);
    pinMode(pinEntradaRS, INPUT);
    attachInterrupt(digitalPinToInterrupt(pinEntradaRT), ISR, RISING);
    for(int j=0;j<10;j++)
    {
      muestreoFrec+=medirFrecuencia();
    }
    frecuenciaMedida = muestreoFrec/j;
    periodo = 1/frecuenciaMedida;
    Serial.print("Frecuencia: ");
    Serial.print(frecuenciaMedida);
    Serial.println(" Hz");
    timer = timerBegin(0, 80, true);  // Temporizador 0, prescaler 80 (1 tick = 1 us)
    timerAttachInterrupt(timer, &onTimer, true);  // Vincular el temporizador a la función 'onTimer'
    // se podria poner 1000*periodo/6 
    timerAlarmWrite(timer, 3333+alfa, true);  // Configurar alarma cada 3333 us (3.33 ms)
    timerAlarmDisable(timer);  // Desactivar el temporizador al inicio
   
}

void loop() {

  medirAlfa();
  timerAlarmWrite(timer, 3333+alfa, true);  // Configurar alarma cada 3333 us (3.33 ms);

  if (calculateIntegral) {
        calculateIntegral = false;  // Reiniciar bandera
        Serial.print("Integral calculada: ");
        Serial.println(integralValue);
        integralValue = 0;  // Reiniciar la integral para el próximo ciclo
    }
  /*float rmsVoltage = calculateRMS();  // Medir voltaje RMS
    float currentRMS = (rmsVoltage - offset) / ACS712_SENSITIVITY;  // Convertir a corriente (A)

    Serial.print("RMS Voltage: ");
    Serial.print(rmsVoltage);
    Serial.print(" V, RMS Current: ");
    Serial.print(currentRMS);
    Serial.println(" A");*/
}


// ISR para la interrupción por flanco ascendente
void ISR() {
    if (!triggered) {  // Solo se activa en la primera interrupción
        triggered = true;  // Marca que ocurrió la interrupción inicial
        timerAlarmEnable(timer);  // Activa el temporizador para iniciar la ejecución periódica
	  digitalWrite(pinesSalida[0],HIGH);
	  digitalWrite(pinesSalida[1],HIGH);
	  digitalWrite(pinesSalida[5],LOW);
    // poner en HIGH los que van
    i=1;
    }
}

// Función llamada periódicamente por el temporizador
void IRAM_ATTR onTimer() {
/*T1 T2
T2 T3
T3 T4
T4 T5
T5 T6
T6 T1*/
    // Código que deseas ejecutar periódicamente
    // Aquí pondrás la tarea que quieres repetir cada 3.33 ms
    // hacer un switch con i para poner en high lo que va.
    // tambien en cada corrida poner low los anteriores
    switch (i) {
        case 1:
            // Acción para el caso 0
            Serial.println("Caso 0");
		digitalWrite(pinesSalida[0],LOW); // apago t1 dejo t2
	      digitalWrite(pinesSalida[2],HIGH); // prendo t3
		i++;
   		calculateIntegral = true;  // Marca que hay que calcular la integral
            break;

        case 2:
            // Acción para el caso 1
            Serial.println("Caso 1");
		digitalWrite(pinesSalida[1],LOW); // apago t2 dejo t3
	      digitalWrite(pinesSalida[3],HIGH); // prendo t4
		i++;
   		calculateIntegral = true;  // Marca que hay que calcular la integral
            break;

        case 3:
            // Acción para el caso 2
            Serial.println("Caso 2");
		digitalWrite(pinesSalida[2],LOW); // apago t3 dejo t4
	      digitalWrite(pinesSalida[4],HIGH); // prendo t5
		i++;
   		calculateIntegral = true;  // Marca que hay que calcular la integral
            break;

        case 4:
            // Acción para el caso 3
            Serial.println("Caso 3");
		digitalWrite(pinesSalida[3],LOW); // apago t4 dejo t5
	      digitalWrite(pinesSalida[5],HIGH); // prendo t6
		i++;
    		calculateIntegral = true;  // Marca que hay que calcular la integral
            break;

        case 5:
            // Acción para el caso 4
            Serial.println("Caso 4");
		digitalWrite(pinesSalida[4],LOW); // apago t5 dejo t6
	      digitalWrite(pinesSalida[0],HIGH); // prendo t1
		i=0;
    		calculateIntegral = true;  // Marca que hay que calcular la integral
            break;

        case 0:
            // Acción para el caso 5
            Serial.println("Caso 5");
		digitalWrite(pinesSalida[1],HIGH); // prendo t2
	      digitalWrite(pinesSalida[5],LOW); // apago t6 y dejo t1
 		i++;
   		calculateIntegral = true;  // Marca que hay que calcular la integral
            break;
}


// Función para medir la frecuencia de la señal
float medirFrecuencia() {
    float valorActual;
    valorActual= analogRead(pinEntrada) * (3.3 / 4095.0);  // Lectura inicial
    // Contar dos cruces por cero para medir el periodo completo, con un timeout
    unsigned long tiempoInicio;
    boolean flag = false;
    while(valorActual>0)
    { // Espero a que llegue un flanco descendente
      valorActual= analogRead(pinEntrada) * (3.3 / 4095.0);  // Lectura inicial
    }
    while (valorActual==0) {
        valorActual = analogRead(pinEntrada) * (3.3 / 4095.0);  // Lectura analógica
        if(!flag){
          tiempoInicio = micros();
          flag = true;
        }
       // Serial.println(valorActual);
        if (valorActual > 0) 
        {
            unsigned long tiempoFinal = micros();
            float periodo = (tiempoFinal - tiempoInicio)*2;  // Tiempo de un ciclo completo en microsegundos
            return 1000000.0 / periodo;  // Frecuencia en Hz
        }
    }
   return 0;
}
void variarAlfa(){
  tensionAlfa = analogRead(pinEntrada) * (3.3 / 4095.0);  // Lectura inicial
  alfaAng = 243.24*(tensionAlfa-2.49;
  alfa = alfaAng *2*Pi/360;
  
  }

// Función para muestrear y calcular la integral
void sampleAndIntegrate() {
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
}
