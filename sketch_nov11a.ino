const int pinEntrada = 34;        // Pin analógico donde se mide la señal (Ejemplo: GPIO34)
const int pinSalida = 13;         // Pin digital que se activará en el cruce por cero
int alfa = 0;
const float umbralCero = 2.5;     // Umbral para el cruce por cero (ajustar si es necesario)
const int pinesSalida[] = {13,14,15,16,17,18,19};
int i=0;
int lim_alfa = 90;
const int pinAlfa = ;
const int pinAlfaLectura = ;
const int 
void setup() {
    pinMode(pinSalida, OUTPUT);
    Serial.begin(115200);
    float frecuenciaMedida = medirFrecuencia();
    Serial.print("Frecuencia: ");
    Serial.print(frecuenciaMedida);
    Serial.println(" Hz");
    
   
}

void loop() {
    detectarCrucePorCero(100);             // Activar el pin de salida por 500 ms en cada cruce por cero
    medirAlfa();
}

// Función para generar una señal de onda sinusoidal aproximada en el pin PWM


// Función para detectar el cruce por cero y activar el pin de salida
void detectarCrucePorCero(unsigned long tiempoActivacion) {
    static float valorPrevio = 0;
    float valorActual = analogRead(pinEntrada) * (3.3 / 4095.0);  // Ajuste para ESP32: 12 bits (0-4095) y 3.3V
    valorPrevio = valorActual;
    // Detecta cruce por cero (cuando el valor cambia de positivo a negativo o viceversa)
    boolean cruce = false;
    while (!cruce) {
        valorActual = analogRead(pinEntrada) * (3.3 / 4095.0);  // Lectura analógica
        if (valorPrevio * valorActual <= 0) {
            digitalWrite(pinesSalida[i], HIGH);  // Activa el pin de salida
            Serial.println("Led ");
            Serial.print(i);
            delayMicroseconds(tiempoActivacion * 1000);  // Tiempo en alto especificado en microsegundos
            digitalWrite(pinesSalida[i], LOW);   // Desactiva el pin de salida
            cruce = true;
            i++;
            if (i == 6) {  // Si se pasa del último pin, reinicia
                i = 0;
            }
        }
        valorPrevio = valorActual;
    }
}

/*void detectarCrucePorCero(unsigned long tiempoActivacion) {
    static float valorPrevio = 0;
    float valorActual = analogRead(pinEntrada) * (3.3 / 4095.0);  // Ajuste para ESP32: 12 bits (0-4095) y 3.3V
    valorPrevio = valorActual;
    // Detecta cruce por cero (cuando el valor cambia de positivo a negativo o viceversa)
    boolean cruce = false;
    while (!cruce) {
        valorActual = analogRead(pinEntrada) * (3.3 / 4095.0);  // Lectura analógica
        if (valorPrevio * valorActual <= 0) {
            digitalWrite(pinSalida, HIGH);  // Activa el pin de salida
            Serial.println("Led alto");
            delayMicroseconds(tiempoActivacion * 1000);  // Tiempo en alto especificado en microsegundos
            digitalWrite(pinSalida, LOW);   // Desactiva el pin de salida
            cruce = true;
        }
        valorPrevio = valorActual;
    }
}*/

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

void medirAlfa(){
  digitalWrite(pinAlfa, HIGH);
  float vo = analogRead(pinAlfaLectura)* (3.3 / 4095.0);
  alfa = lim_alfa-(lim_alfa/3.3)*vo
  digitalWrite(pinAlfa, LOW);
}
