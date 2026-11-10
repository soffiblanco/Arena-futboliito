#include <Arduino.h>
#include <Servo.h>
#include <Arduino_FreeRTOS.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

// CONFIGURACION GENERAL
const bool HABILITAR_GOLES = true;
const bool HABILITAR_GOLPES = true;

// Registros 74HC595
const byte DATOS = 2;
const byte RELOJ = 3;
const byte LATCH = 4;

// Sensores de gol
const byte GOL1 = 5;
const byte GOL2 = 6;

// Interruptor
const byte INTERRUPTOR = 7;

// Servos
const byte PIN_SERVO1 = 8;
const byte PIN_SERVO2 = 9;

// Portero 1
const byte PUL1 = 10;
const byte DIR1 = 11;
const byte IZQ1 = A2;
const byte DER1 = A3;

// Portero 2
const byte PUL2 = 13;
const byte DIR2 = A4;
const byte IZQ2 = A5;
const byte DER2 = A0;

// Direcciones comprobadas
const bool DERECHA1 = HIGH;
const bool DERECHA2 = HIGH;

// Final presionado = HIGH
const byte FINAL_ACTIVO = HIGH;

// Calibracion original
const unsigned long PULSO_CAL = 500;
const unsigned long PULSO_JUEGO = 50;
const unsigned long PAUSA_DIRECCION = 500000UL;

const unsigned long MAX_PASOS = 10000;
const unsigned long MAX_ETAPA_MS = 30000;

// Seguimiento
const unsigned long TIMEOUT_CAMARA = 600;
const long MARGEN_PASOS = 80;
const long TOLERANCIA = 12;

const bool INVERTIR_X_P1 = false;
const bool INVERTIR_X_P2 = false;

// CONFIGURACION DE GOLPES

// Ajustar segun montaje fisico
const int REPOSO1 = 90;
const int GOLPE1 = 55;

const int REPOSO2 = 90;
const int GOLPE2 = 125;

// Duracion del golpe y regreso
const unsigned long DURACION_GOLPE = 170;

// Espera entre golpes
const unsigned long ESPERA_GOLPE = 500;

// Porcentaje del alto de la imagen
// donde cada portero puede golpear
const int ZONA_GOLPE = 30;

// Margen para considerar portero alineado
const long MARGEN_ALINEACION = 10000;

// Portero 1 arriba, portero 2 abajo
const bool INVERTIR_Y = false;

// ESTADOS

enum Estado {
  APAGADO,
  CALIBRANDO,
  JUGANDO,
  FINALIZADO,
  ERROR_SISTEMA
};

enum Etapa {
  BUSCAR_IZQUIERDA,
  BUSCAR_DERECHA,
  IR_CENTRO,
  SEGUIR,
  LISTO,
  ERROR_MOTOR
};

Estado estado = APAGADO;

// MOTORES

struct Motor {
  byte pul, dir, izq, der;

  bool derechaNivel;
  bool sentidoDerecha;
  bool pulsoAlto;

  Etapa etapa;

  unsigned long pasos;
  unsigned long recorrido;
  unsigned long mitad;

  unsigned long ultimoCambio;
  unsigned long esperaHasta;
  unsigned long inicioEtapa;

  long posicion;
  long objetivo;
};

Motor m1 = {
  PUL1, DIR1, IZQ1, DER1,
  DERECHA1, false, false,
  LISTO,
  0, 0, 0, 0, 0, 0,
  0, 0
};

Motor m2 = {
  PUL2, DIR2, IZQ2, DER2,
  DERECHA2, false, false,
  LISTO,
  0, 0, 0, 0, 0, 0,
  0, 0
};

// SERVOS

Servo servo1;
Servo servo2;

struct Golpeador {
  Servo *servo;
  int reposo;
  int golpe;

  bool golpeando;
  bool golpeAnterior;

  unsigned long inicioGolpe;
  unsigned long ultimoGolpe;
};

Golpeador g1 = {
  &servo1, REPOSO1, GOLPE1,
  false, false, 0, 0
};

Golpeador g2 = {
  &servo2, REPOSO2, GOLPE2,
  false, false, 0, 0
};

// VIDAS

int vidas1 = 10;
int vidas2 = 10;
int ganador = 0;

bool registrado1 = false;
bool registrado2 = false;

unsigned long libre1 = 0;
unsigned long libre2 = 0;

unsigned long inicioGol1 = 0;
unsigned long inicioGol2 = 0;

const unsigned long REARME_GOL = 300;
const unsigned long CONFIRMACION_GOL = 30;

// Celebracion
unsigned long ultimoParpadeo = 0;
byte cambiosCelebracion = 0;
bool celebracionTerminada = false;

// CAMARA

unsigned long ultimoDato = 0;
bool camaraValida = false;

int pelotaX = 0;
int pelotaY = 0;
int anchoROI = 0;
int altoROI = 0;

char linea[48];
byte longitud = 0;

// MARCADOR

void leds(byte tercero, byte segundo, byte primero) {
  digitalWrite(LATCH, LOW);

  shiftOut(DATOS, RELOJ, MSBFIRST, tercero);
  shiftOut(DATOS, RELOJ, MSBFIRST, segundo);
  shiftOut(DATOS, RELOJ, MSBFIRST, primero);

  digitalWrite(LATCH, HIGH);
}

void patrones(unsigned int a, unsigned int b) {
  a &= 0x03FF;
  b &= 0x03FF;

  leds(
    (b >> 6) & 15,
    ((a >> 8) & 3) | ((b & 63) << 2),
    a & 255
  );
}

void actualizarVidas() {
  patrones(
    (1U << vidas1) - 1,
    (1U << vidas2) - 1
  );
}

// PULSOS DE MOTORES

void parar(Motor &m) {
  digitalWrite(m.pul, LOW);
  m.pulsoAlto = false;
  m.ultimoCambio = micros();
}

void sentido(Motor &m, bool derecha) {
  parar(m);

  m.sentidoDerecha = derecha;

  digitalWrite(
    m.dir,
    derecha ? m.derechaNivel : !m.derechaNivel
  );

  m.esperaHasta = micros() + PAUSA_DIRECCION;
}

void etapa(Motor &m, Etapa e) {
  m.etapa = e;
  m.pasos = 0;
  m.inicioEtapa = millis();

  if (e == BUSCAR_IZQUIERDA || e == IR_CENTRO) {
    sentido(m, false);
  }

  if (e == BUSCAR_DERECHA) {
    sentido(m, true);
  }
}

void preparar(Motor &m) {
  pinMode(m.pul, OUTPUT);
  pinMode(m.dir, OUTPUT);

  pinMode(m.izq, INPUT_PULLUP);
  pinMode(m.der, INPUT_PULLUP);

  parar(m);
}

bool paso(Motor &m, unsigned long tiempo) {
  unsigned long ahora = micros();

  if (m.pulsoAlto) {
    if (ahora - m.ultimoCambio >= tiempo) {
      digitalWrite(m.pul, LOW);
      m.pulsoAlto = false;
      m.ultimoCambio = ahora;
    }
    return false;
  }

  if ((long)(ahora - m.esperaHasta) < 0) {
    return false;
  }

  if (ahora - m.ultimoCambio >= tiempo) {
    digitalWrite(m.pul, HIGH);

    m.pulsoAlto = true;
    m.ultimoCambio = ahora;
    m.pasos++;

    return true;
  }

  return false;
}

// SEGURIDAD

void detenerServos() {
  if (servo1.attached()) {
    servo1.write(REPOSO1);
  }

  if (servo2.attached()) {
    servo2.write(REPOSO2);
  }

  g1.golpeando = false;
  g2.golpeando = false;
}

void fallo(byte numero, const char *motivo) {
  parar(m1);
  parar(m2);
  detenerServos();

  m1.etapa = ERROR_MOTOR;
  m2.etapa = ERROR_MOTOR;

  estado = ERROR_SISTEMA;

  Serial.print("ERROR PORTERO ");
  Serial.print(numero);
  Serial.print(": ");
  Serial.println(motivo);

  Serial.println("DESCONECTAR 12V Y REVISAR");
}

// CALIBRACION ORIGINAL

void iniciarCalibracion() {
  m1.recorrido = 0;
  m2.recorrido = 0;

  m1.mitad = 0;
  m2.mitad = 0;

  m1.posicion = 0;
  m2.posicion = 0;

  etapa(m1, BUSCAR_IZQUIERDA);
  etapa(m2, BUSCAR_IZQUIERDA);

  estado = CALIBRANDO;

  Serial.println(
    "CALIBRANDO: IZQUIERDA -> DERECHA -> CENTRO"
  );
}

void calibrar(Motor &m, byte numero) {
  if (m.etapa == LISTO || m.etapa == ERROR_MOTOR) {
    return;
  }

  bool izq = digitalRead(m.izq) == FINAL_ACTIVO;
  bool der = digitalRead(m.der) == FINAL_ACTIVO;

  if (izq && der) {
    fallo(numero, "DOS FINALES ACTIVOS");
    return;
  }

  if (millis() - m.inicioEtapa > MAX_ETAPA_MS) {
    fallo(numero, "TIEMPO AGOTADO");
    return;
  }

  if (m.etapa == BUSCAR_IZQUIERDA) {
    if (izq) {
      parar(m);

      Serial.print("P");
      Serial.print(numero);
      Serial.println(" IZQUIERDA OK");

      etapa(m, BUSCAR_DERECHA);
    }
    else if (m.pasos >= MAX_PASOS) {
      fallo(numero, "NO ENCONTRO IZQUIERDA");
    }
    else {
      paso(m, PULSO_CAL);
    }
  }

  else if (m.etapa == BUSCAR_DERECHA) {
    if (der) {
      parar(m);

      m.recorrido = m.pasos;
      m.mitad = m.recorrido / 2;

      Serial.print("P");
      Serial.print(numero);
      Serial.print(" RECORRIDO ");
      Serial.println(m.recorrido);

      if (m.recorrido < 100) {
        fallo(numero, "RECORRIDO CORTO");
        return;
      }

      etapa(m, IR_CENTRO);
    }
    else if (m.pasos >= MAX_PASOS) {
      fallo(numero, "NO ENCONTRO DERECHA");
    }
    else {
      paso(m, PULSO_CAL);
    }
  }

  else if (m.etapa == IR_CENTRO) {
    if (izq) {
      fallo(numero, "IZQUIERDA DURANTE CENTRADO");
      return;
    }

    if (m.pasos >= m.mitad) {
      parar(m);

      m.posicion =
        (long)m.recorrido - (long)m.pasos;

      m.objetivo = m.posicion;
      m.etapa = LISTO;

      Serial.print("P");
      Serial.print(numero);
      Serial.println(" CENTRADO OK");
    }
    else {
      paso(m, PULSO_CAL);
    }
  }
}

// INICIO Y APAGADO

void prepararGoles() {
  registrado1 = digitalRead(GOL1) == LOW;
  registrado2 = digitalRead(GOL2) == LOW;

  libre1 = 0;
  libre2 = 0;

  inicioGol1 = 0;
  inicioGol2 = 0;
}

void iniciarJuego() {
  vidas1 = 10;
  vidas2 = 10;
  ganador = 0;

  cambiosCelebracion = 0;
  celebracionTerminada = false;

  actualizarVidas();
  prepararGoles();

  camaraValida = false;

  detenerServos();

  g1.ultimoGolpe = millis();
  g2.ultimoGolpe = millis();

  g1.golpeAnterior = false;
  g2.golpeAnterior = false;

  iniciarCalibracion();

  Serial.println("JUEGO ENCENDIDO");
}

void apagar() {
  parar(m1);
  parar(m2);

  detenerServos();

  m1.etapa = LISTO;
  m2.etapa = LISTO;

  estado = APAGADO;
  camaraValida = false;

  leds(0, 0, 0);

  Serial.println("JUEGO APAGADO");
}

// RECEPCION DE PYTHON

void procesarLinea(char *s) {
  if (strncmp(s, "X:", 2) != 0) {
    return;
  }

  int x, y, w, h;
  char extra;

  if (
    sscanf(
      s + 2,
      "%d,%d,%d,%d%c",
      &x, &y, &w, &h, &extra
    ) != 4
  ) {
    return;
  }

  if (
    w <= 0 || h <= 0 ||
    w > 4096 || h > 4096 ||
    x < 0 || x > w ||
    y < 0 || y > h
  ) {
    return;
  }

  pelotaX = x;
  pelotaY = y;
  anchoROI = w;
  altoROI = h;

  ultimoDato = millis();
  camaraValida = true;
}

void leerSerial() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (longitud) {
        linea[longitud] = '\0';
        procesarLinea(linea);
        longitud = 0;
      }
    }
    else if (longitud < sizeof(linea) - 1) {
      linea[longitud++] = c;
    }
    else {
      longitud = 0;
    }
  }
}

// SEGUIMIENTO DE PELOTA

long objetivoPelota(Motor &m, bool invertir) {
  long minimo = MARGEN_PASOS;
  long maximo = (long)m.recorrido - MARGEN_PASOS;

  if (maximo <= minimo) {
    return m.posicion;
  }

  long x = invertir
    ? anchoROI - pelotaX
    : pelotaX;

  return minimo + (x * (maximo - minimo)) / anchoROI;
}

void seguimiento(
  Motor &m,
  byte numero,
  bool invertir
) {
  if (m.etapa != SEGUIR) {
    return;
  }

  bool izq = digitalRead(m.izq) == FINAL_ACTIVO;
  bool der = digitalRead(m.der) == FINAL_ACTIVO;

  if (izq && der) {
    fallo(numero, "DOS FINALES ACTIVOS");
    return;
  }

  if (m.pulsoAlto) {
    paso(m, PULSO_JUEGO);
    return;
  }

  if (
    !camaraValida ||
    millis() - ultimoDato > TIMEOUT_CAMARA
  ) {
    parar(m);
    return;
  }

  m.objetivo = objetivoPelota(m, invertir);

  long diferencia = m.objetivo - m.posicion;

  if (labs(diferencia) <= TOLERANCIA) {
    parar(m);
    return;
  }

  bool derecha = diferencia > 0;

  if (
    (derecha && der) ||
    (!derecha && izq)
  ) {
    fallo(numero, "FINAL ACTIVADO");
    return;
  }

  if (
    (m.posicion <= 0 && !derecha) ||
    (m.posicion >= (long)m.recorrido && derecha)
  ) {
    fallo(numero, "FUERA DE RECORRIDO");
    return;
  }

  if (m.sentidoDerecha != derecha) {
    sentido(m, derecha);
  }

  if (paso(m, PULSO_JUEGO)) {
    m.posicion += derecha ? 1 : -1;
  }
}

// GOLPES AUTOMATICOS

void actualizarGolpe(
  Golpeador &g,
  Motor &m,
  bool zona
) {
  unsigned long ahora = millis();

  // Si esta golpeando, esperar y regresar
  if (g.golpeando) {
    if (ahora - g.inicioGolpe >= DURACION_GOLPE) {
      g.servo->write(g.reposo);
      g.golpeando = false;

      Serial.println("SERVO REGRESO");
    }
    return;
  }

  // Si la pelota sale de la zona,
  // permitir un nuevo golpe
  if (!zona) {
    g.golpeAnterior = false;
    return;
  }

  // Comprobar alineacion
  bool alineado =
    labs(m.objetivo - m.posicion) <= MARGEN_ALINEACION;

  // Comprobar tiempo entre golpes
  bool disponible =
    ahora - g.ultimoGolpe >= ESPERA_GOLPE;

  // Golpear una vez por entrada a la zona,
  // incluso si el portero se alinea despues
  if (
    zona &&
    !g.golpeAnterior &&
    alineado &&
    disponible
  ) {
    Serial.println("GOLPE ACTIVADO");

    g.servo->write(g.golpe);

    g.golpeando = true;
    g.inicioGolpe = ahora;
    g.ultimoGolpe = ahora;

    g.golpeAnterior = true;
  }
}


void actualizarServos() {
  if (!HABILITAR_GOLPES) return;

  unsigned long ahora = millis();

  if (g1.golpeando && ahora - g1.inicioGolpe >= DURACION_GOLPE) {
    servo1.write(REPOSO1);
    g1.golpeando = false;
  }

  if (g2.golpeando && ahora - g2.inicioGolpe >= DURACION_GOLPE) {
    servo2.write(REPOSO2);
    g2.golpeando = false;
  }

  if (!camaraValida || ahora - ultimoDato > TIMEOUT_CAMARA) {
    g1.golpeAnterior = false;
    g2.golpeAnterior = false;
    return;
  }

  long y = INVERTIR_Y ? (long)altoROI - pelotaY : pelotaY;

  // Multiplicacion LONG para evitar desbordamiento
  long limite1 = ((long)altoROI * ZONA_GOLPE) / 100;
  long limite2 = ((long)altoROI * (100 - ZONA_GOLPE)) / 100;

  bool zona1 = y <= limite1;
  bool zona2 = y >= limite2;

  actualizarGolpe(g1, m1, zona1);
  actualizarGolpe(g2, m2, zona2);
}


// GOLES Y VIDAS

void registrarGol(byte numero) {
  int &vidas = numero == 1 ? vidas1 : vidas2;

  if (vidas <= 0) {
    return;
  }

  vidas--;
  actualizarVidas();

  Serial.print("GOL CONTRA PORTERO ");
  Serial.print(numero);
  Serial.print(" | VIDAS: ");
  Serial.println(vidas);

  if (vidas == 0) {
    ganador = numero == 1 ? 2 : 1;
    estado = FINALIZADO;

    parar(m1);
    parar(m2);
    detenerServos();

    cambiosCelebracion = 0;
    ultimoParpadeo = 0;
    celebracionTerminada = false;

    Serial.print("GANADOR PORTERO ");
    Serial.println(ganador);
  }
}

void revisarGol(
  byte pin,
  byte numero,
  bool &registrado,
  unsigned long &libre,
  unsigned long &inicio
) {
  unsigned long ahora = millis();
  bool detectado = digitalRead(pin) == LOW;

  if (registrado) {
    if (!detectado) {
      if (!libre) {
        libre = ahora;
      }

      if (ahora - libre >= REARME_GOL) {
        registrado = false;
        libre = 0;
        inicio = 0;
      }
    }
    else {
      libre = 0;
    }
    return;
  }

  if (detectado) {
    if (!inicio) {
      inicio = ahora;
    }

    if (ahora - inicio >= CONFIRMACION_GOL) {
      registrado = true;
      inicio = 0;
      libre = 0;

      registrarGol(numero);
    }
  }
  else {
    inicio = 0;
  }
}

// CELEBRACION DEL GANADOR

void celebrar() {
  if (celebracionTerminada) {
    return;
  }

  unsigned long ahora = millis();

  if (
    cambiosCelebracion > 0 &&
    ahora - ultimoParpadeo < 200
  ) {
    return;
  }

  ultimoParpadeo = ahora;

  unsigned int patron =
    (cambiosCelebracion % 2 == 0)
      ? 0x03FF
      : 0;

  if (ganador == 1) {
    patrones(patron, 0);
  }
  else {
    patrones(0, patron);
  }

  cambiosCelebracion++;

  if (cambiosCelebracion >= 12) {
    if (ganador == 1) {
      patrones(0x03FF, 0);
    }
    else {
      patrones(0, 0x03FF);
    }

    celebracionTerminada = true;

    Serial.println("FIN DE CELEBRACION");
    Serial.println("APAGAR Y ENCENDER PARA REINICIAR");
  }
}

// SETUP

void tareaTiempoReal(void *parametros);

void setup() {
  Serial.begin(9600);

  pinMode(DATOS, OUTPUT);
  pinMode(RELOJ, OUTPUT);
  pinMode(LATCH, OUTPUT);

  pinMode(GOL1, INPUT_PULLUP);
  pinMode(GOL2, INPUT_PULLUP);

  pinMode(INTERRUPTOR, INPUT_PULLUP);

  preparar(m1);
  preparar(m2);

  leds(0, 0, 0);

  if (HABILITAR_GOLPES) {
    servo1.attach(PIN_SERVO1);
    servo2.attach(PIN_SERVO2);

    servo1.write(REPOSO1);
    servo2.write(REPOSO2);
  }

  Serial.println("======================");
  Serial.println("ARENA DE FUTBOLITO");
  Serial.println("SEGUIMIENTO + GOLPES + VIDAS");
  Serial.println("======================");
  Serial.println("ESPERANDO INTERRUPTOR ON");

  // Pila de 210 bytes (Arduino_FreeRTOS AVR usa bytes).
  // Si no se puede crear la tarea, el LED integrado parpadea.
  BaseType_t resultado = xTaskCreate(
    tareaTiempoReal, "ARENA", 210, NULL, 1, NULL
  );
  if (resultado != pdPASS) {
    pinMode(LED_BUILTIN, OUTPUT);
    while (true) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(250);
    }
  }
}

// ========================================================
// METODOS DE TIEMPO REAL
// ========================================================
// Los dos actuadores lineales conservan el generador de pulsos
// no bloqueante del codigo original.
void moverCarritos() {
  seguimiento(m1, 1, INVERTIR_X_P1);
  if (estado == JUGANDO) {
    seguimiento(m2, 2, INVERTIR_X_P2);
  }
}

// Los servos son los actuadores de golpe.
// actualizarServos() ya controla zona, duracion y reposo
// sin utilizar delay().
void moverGolpeadores() {
  actualizarServos();
}

void actualizarSensoresGol() {
  if (!HABILITAR_GOLES) return;
  revisarGol(GOL1, 1, registrado1, libre1, inicioGol1);
  if (estado == JUGANDO) {
    revisarGol(GOL2, 2, registrado2, libre2, inicioGol2);
  }
}

// Un ciclo completo del sistema original, sin cambiar su logica.
void cicloJuego() {
  leerSerial();

  bool encendido = digitalRead(INTERRUPTOR) == LOW;
  if (!encendido) {
    if (estado != APAGADO) apagar();
    return;
  }

  if (estado == APAGADO) iniciarJuego();
  if (estado == ERROR_SISTEMA) return;

  if (estado == FINALIZADO) {
    celebrar();
    return;
  }

  if (estado == CALIBRANDO) {
    calibrar(m1, 1);
    if (estado != CALIBRANDO) return;
    calibrar(m2, 2);
    if (estado != CALIBRANDO) return;

    if (m1.etapa == LISTO && m2.etapa == LISTO) {
      m1.etapa = SEGUIR;
      m2.etapa = SEGUIR;
      m1.objetivo = m1.posicion;
      m2.objetivo = m2.posicion;
      prepararGoles();
      estado = JUGANDO;
      Serial.println(F("AMBOS LISTOS - PARTIDA INICIADA"));
    }
    return;
  }

  if (estado == JUGANDO) {
    moverCarritos();
    if (estado != JUGANDO) return;
    moverGolpeadores();
    if (estado != JUGANDO) return;
    actualizarSensoresGol();
  }
}

// FreeRTOS: tarea periodica que coordina todos los actuadores.
// Una sola tarea reduce la presion de SRAM en Arduino UNO (2 KB).
// Entre activaciones se cede la CPU al planificador.
void tareaTiempoReal(void *parametros) {
  (void)parametros;
  TickType_t ultimoTick = xTaskGetTickCount();
  for (;;) {
    // Durante un tick se atienden varias veces los pulsos
    // para no depender de un vTaskDelay por cada paso.
    unsigned long inicio = micros();
    do {
      cicloJuego();
    } while ((unsigned long)(micros() - inicio) < 1000UL);

    vTaskDelayUntil(&ultimoTick, 1);
  }
}

// Arduino_FreeRTOS para AVR inicia el scheduler al terminar setup().
// El loop no realiza tareas del juego.
void loop() {}
