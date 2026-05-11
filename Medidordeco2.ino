#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ===== CONFIGURACION WIFI =====
const char* ssid = "MedidordeCo2";
const char* password = "123456789";
IPAddress apIP(192, 168, 4, 1);
IPAddress apGateway(192, 168, 4, 1);
IPAddress apSubnet(255, 255, 255, 0);

// ===== PINES =====
#define MQ135_PIN 34
#define LED_VERDE 25
#define LED_AZUL 26
#define LED_ROJO 27

// ===== OBJETOS =====
LiquidCrystal_I2C lcd(0x27, 16, 2);
WiFiServer server(80);

struct TramoICA {
  int cLo;
  int cHi;
  int iLo;
  int iHi;
  const char* estado;
};

int lecturaBase = 0;
bool calibracionValida = false;
String diagnosticoSensor = "Esperando calibracion";
unsigned long ultimaActualizacionLCD = 0;
const unsigned long INTERVALO_LCD_MS = 1500;
const int RAW_DELTA_GAS_MAX = 1200;
const int GAS_ESTIMADO_MAX = 3000;
const int ADC_SATURADO = 4090;
const int MARGEN_SATURACION = 200;
const int ADC_MINIMO_VALIDO = 20;
const int UMBRAL_LED_VERDE_MAX = 166;
const int UMBRAL_LED_AZUL_MAX = 334;
const unsigned long PRECALENTAMIENTO_MINIMO_MS = 2UL * 60UL * 1000UL;
const unsigned long PRECALENTAMIENTO_RECOMENDADO_MS = 5UL * 60UL * 1000UL;
const unsigned long PRECALENTAMIENTO_IDEAL_MS = 10UL * 60UL * 1000UL;
const int DELTA_ESTABLE_MAX = 15;
const int LECTURAS_ESTABLES_NECESARIAS = 60;
const unsigned long INTERVALO_ESTABILIDAD_MS = 500;
const unsigned long TIEMPO_MAX_ESTABILIZACION_MS = PRECALENTAMIENTO_IDEAL_MS;

const TramoICA TRAMOS_ICA[] = {
  {0, 350, 0, 50, "BUENA"},
  {351, 800, 51, 100, "MODERADA"},
  {801, 1200, 101, 150, "SENSIBLE"},
  {1201, 1600, 151, 200, "DANINA"},
  {1601, 2200, 201, 300, "MUY DANINA"},
  {2201, 3000, 301, 500, "PELIGROSA"}
};

int leerPromedioSensor(int muestras, int pausaMs) {
  long suma = 0;
  for (int i = 0; i < muestras; i++) {
    suma += analogRead(MQ135_PIN);
    delay(pausaMs);
  }
  return static_cast<int>(suma / max(1, muestras));
}

bool lecturaSaturada(int valorADC) {
  return valorADC >= (ADC_SATURADO - MARGEN_SATURACION);
}

bool lecturaMuyBaja(int valorADC) {
  return valorADC <= ADC_MINIMO_VALIDO;
}

String minutosRedondeados(unsigned long milisegundos) {
  unsigned long minutos = (milisegundos + 59999UL) / 60000UL;
  return String(minutos) + " min";
}

bool esperarEstabilizacionSensor() {
  Serial.println("Estabilizando MQ135 antes de tomar lecturaBase...");
  Serial.println("Metodo: comparar lecturas consecutivas hasta que el cambio sea pequeno durante un rato.");
  Serial.print("Minimo absoluto: ");
  Serial.println(minutosRedondeados(PRECALENTAMIENTO_MINIMO_MS));
  Serial.print("Recomendado: ");
  Serial.println(minutosRedondeados(PRECALENTAMIENTO_RECOMENDADO_MS));
  Serial.print("Ideal: ");
  Serial.println(minutosRedondeados(PRECALENTAMIENTO_IDEAL_MS));
  Serial.print("Delta estable maximo: ");
  Serial.println(DELTA_ESTABLE_MAX);

  unsigned long inicio = millis();
  int lecturaAnterior = leerPromedioSensor(4, 10);
  int lecturasEstables = 0;

  while (millis() - inicio < TIEMPO_MAX_ESTABILIZACION_MS) {
    int lecturaActual = leerPromedioSensor(4, 10);
    int diferencia = abs(lecturaActual - lecturaAnterior);

    if (!lecturaSaturada(lecturaActual) && !lecturaMuyBaja(lecturaActual) && diferencia <= DELTA_ESTABLE_MAX) {
      lecturasEstables++;
    } else {
      lecturasEstables = 0;
    }

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Estabilizando");
    lcd.setCursor(0, 1);
    lcd.print("Est ");
    lcd.print(lecturasEstables);
    lcd.print("/");
    lcd.print(LECTURAS_ESTABLES_NECESARIAS);

    Serial.print("RAW=");
    Serial.print(lecturaActual);
    Serial.print(" DIF=");
    Serial.print(diferencia);
    Serial.print(" ESTABLES=");
    Serial.println(lecturasEstables);

    if (lecturasEstables >= LECTURAS_ESTABLES_NECESARIAS) {
      diagnosticoSensor = "Sensor estable. Base automatica tomada cuando la variacion RAW se mantuvo pequena.";
      return true;
    }

    lecturaAnterior = lecturaActual;
    delay(INTERVALO_ESTABILIDAD_MS);
  }

  diagnosticoSensor = "No se logro estabilidad antes del tiempo maximo. Revisar sensor, alimentacion, GND y salida analogica.";
  return false;
}

bool calibrarLecturaBase() {
  lecturaBase = leerPromedioSensor(40, 50);

  if (lecturaSaturada(lecturaBase)) {
    diagnosticoSensor = "Calibracion invalida: lecturaBase esta saturada. Revisar AO > 3.3V, divisor de voltaje, GND comun y precalentamiento.";
    return false;
  }

  if (lecturaMuyBaja(lecturaBase)) {
    diagnosticoSensor = "Calibracion invalida: lecturaBase esta casi en 0. Revisar cable AO, GND, alimentacion del modulo y pin GPIO 34.";
    return false;
  }

  diagnosticoSensor = "Base automatica valida. El ICA se calcula con el aumento RAW respecto a lecturaBase.";
  return true;
}

int convertirRawAGasEstimado(int valorADC) {
  if (lecturaSaturada(valorADC)) {
    return GAS_ESTIMADO_MAX;
  }

  if (!calibracionValida) {
    return 0;
  }

  int deltaADC = valorADC - lecturaBase;
  if (deltaADC <= 0) {
    return 0;
  }

  long gasEstimado = map(deltaADC, 0, RAW_DELTA_GAS_MAX, 0, GAS_ESTIMADO_MAX);
  return constrain(static_cast<int>(gasEstimado), 0, GAS_ESTIMADO_MAX);
}

int interpolarICA(int concentracion, const TramoICA& tramo) {
  float proporcion = float(concentracion - tramo.cLo) / float(tramo.cHi - tramo.cLo);
  float indice = tramo.iLo + proporcion * float(tramo.iHi - tramo.iLo);
  return constrain(static_cast<int>(round(indice)), tramo.iLo, tramo.iHi);
}

int calcularICAEstimado(int gasEstimado) {
  for (const TramoICA& tramo : TRAMOS_ICA) {
    if (gasEstimado <= tramo.cHi) {
      return interpolarICA(gasEstimado, tramo);
    }
  }
  return 500;
}

String obtenerEstadoICA(int ica) {
  if (ica <= 50) return "BUENA";
  if (ica <= 100) return "MODERADA";
  if (ica <= 150) return "SENSIBLE";
  if (ica <= 200) return "DANINA";
  if (ica <= 300) return "MUY DANINA";
  return "PELIGROSA";
}

String colorDesdeRGB(int rojo, int verde, int azul) {
  char color[8];
  snprintf(color, sizeof(color), "#%02X%02X%02X", rojo, verde, azul);
  return String(color);
}

int interpolarCanal(int inicio, int fin, int valor, int valorMaximo) {
  return inicio + ((fin - inicio) * valor) / valorMaximo;
}

String obtenerColorICA(int ica) {
  int valor = constrain(ica, 0, 500);

  if (valor <= 250) {
    int rojo = interpolarCanal(45, 2, valor, 250);
    int verde = interpolarCanal(198, 132, valor, 250);
    int azul = interpolarCanal(83, 199, valor, 250);
    return colorDesdeRGB(rojo, verde, azul);
  }

  int tramo = valor - 250;
  int rojo = interpolarCanal(2, 230, tramo, 250);
  int verde = interpolarCanal(132, 57, tramo, 250);
  int azul = interpolarCanal(199, 70, tramo, 250);
  return colorDesdeRGB(rojo, verde, azul);
}

String obtenerRecomendacionICA(int ica) {
  if (ica <= 50) return "Condiciones favorables. Mantener ventilacion natural y seguimiento periodico.";
  if (ica <= 100) return "Nivel aceptable. Conviene reforzar ventilacion y revisar el ambiente con frecuencia.";
  if (ica <= 150) return "Personas sensibles pueden verse afectadas. Reducir tiempo de exposicion y aumentar ventilacion.";
  if (ica <= 200) return "Calidad comprometida. Se recomienda intervenir el ambiente y limitar la permanencia prolongada.";
  if (ica <= 300) return "Nivel muy danino. Priorizar evacuacion parcial, ventilacion intensa y revision del sensor.";
  return "Nivel peligroso. Se recomienda desocupar el espacio y aplicar medidas inmediatas.";
}

int calcularPorcentajeICA(int ica) {
  long porcentaje = map(ica, 0, 500, 0, 100);
  return constrain(static_cast<int>(porcentaje), 0, 100);
}

void encenderLedICA(bool verde, bool azul, bool rojo) {
  digitalWrite(LED_VERDE, verde ? HIGH : LOW);
  digitalWrite(LED_AZUL, azul ? HIGH : LOW);
  digitalWrite(LED_ROJO, rojo ? HIGH : LOW);
}

void actualizarSalidas(int ica, const String& estadoICA) {
  if (ica <= UMBRAL_LED_VERDE_MAX) {          // 0 - 166
    encenderLedICA(true, false, false);
  } else if (ica <= UMBRAL_LED_AZUL_MAX) {    // 167 - 334
    encenderLedICA(false, true, false);
  } else {                                    // 335 - 500
    encenderLedICA(false, false, true);
  }

  if (millis() - ultimaActualizacionLCD >= INTERVALO_LCD_MS) {
    ultimaActualizacionLCD = millis();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Indice ICA: " + String(ica) + "      ");
    lcd.setCursor(0, 1);
    lcd.print(estadoICA + "        ");
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_VERDE, OUTPUT);
  pinMode(LED_AZUL, OUTPUT);
  pinMode(LED_ROJO, OUTPUT);
  analogSetPinAttenuation(MQ135_PIN, ADC_11db);

  // Iniciar LCD
  Wire.begin(21, 22);
  lcd.init();
  lcd.backlight();
  lcd.print("Iniciando...");

  // Crear red WiFi propia en la ESP32
  WiFi.setSleep(false);
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(apIP, apGateway, apSubnet);
  bool apIniciado = WiFi.softAP(ssid, password, 6, false, 4);

  if (!apIniciado) {
    Serial.println("Error iniciando la red WiFi");
    lcd.clear();
    lcd.print("Error WiFi AP");
    while (true) {
      delay(1000);
    }
  }

  server.begin();
  Serial.println("\nPunto de acceso iniciado");
  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());
  delay(4000);

  // Base automatica despues de detectar que el MQ135 se estabilizo.
  bool sensorEstable = esperarEstabilizacionSensor();
  calibracionValida = sensorEstable && calibrarLecturaBase();
  Serial.print("Lectura base: ");
  Serial.println(lecturaBase);
  Serial.print("Estado calibracion: ");
  Serial.println(calibracionValida ? "VALIDA" : "INVALIDA");
  Serial.println(diagnosticoSensor);

  lcd.clear();
  lcd.print("IP: ");
  lcd.setCursor(0, 1);
  lcd.print(WiFi.softAPIP());
  delay(3000);
}

void loop() {
  // 1. LECTURA Y CALCULO
  int valorADC = leerPromedioSensor(8, 5);
  int deltaADC = calibracionValida ? max(0, valorADC - lecturaBase) : 0;
  int gasEstimado = convertirRawAGasEstimado(valorADC);
  int icaEstimado = calcularICAEstimado(gasEstimado);
  bool sensorSaturado = lecturaSaturada(valorADC);

  // 2. CONTROL DE HARDWARE
  String estadoICA = obtenerEstadoICA(icaEstimado);
  String colorICA = obtenerColorICA(icaEstimado);
  String recomendacionICA = obtenerRecomendacionICA(icaEstimado);
  int porcentajeICA = calcularPorcentajeICA(icaEstimado);

  if (!calibracionValida) {
    icaEstimado = sensorSaturado ? 500 : 0;
    estadoICA = sensorSaturado ? "SATURADO" : "CALIB INVALIDA";
    colorICA = sensorSaturado ? obtenerColorICA(500) : "#64748b";
    recomendacionICA = diagnosticoSensor;
  } else if (sensorSaturado) {
    estadoICA = "SATURADO";
    colorICA = obtenerColorICA(500);
    recomendacionICA = "La salida analogica del sensor esta al limite del ADC. Revisar cableado, voltaje del modulo y tiempo de precalentamiento.";
  }
  porcentajeICA = calcularPorcentajeICA(icaEstimado);

  actualizarSalidas(icaEstimado, estadoICA);

  static unsigned long ultimoLog = 0;
  if (millis() - ultimoLog >= 2000) {
    ultimoLog = millis();
    Serial.print("RAW=");
    Serial.print(valorADC);
    Serial.print(" BASE=");
    Serial.print(lecturaBase);
    Serial.print(" DELTA=");
    Serial.print(deltaADC);
    Serial.print(" GAS=");
    Serial.print(gasEstimado);
    Serial.print(" ICA=");
    Serial.print(icaEstimado);
    Serial.print(" WIFI_IP=");
    Serial.print(WiFi.softAPIP());
    Serial.print(" CLIENTES=");
    Serial.print(WiFi.softAPgetStationNum());
    Serial.print(" CAL=");
    Serial.println(calibracionValida ? "OK" : "INVALIDA");
  }

  // 3. SERVIDOR WEB NATIVO
  WiFiClient client = server.available();
  if (client) {
    String currentLine = "";
    String requestLine = "";
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        if (c == '\n') {
          if (currentLine.length() == 0) {
            if (requestLine.startsWith("GET /data")) {
              client.println("HTTP/1.1 200 OK");
              client.println("Content-type:application/json");
              client.println("Connection: close");
              client.println();
              client.println("{");
              client.println("\"ica\":" + String(icaEstimado) + ",");
              client.println("\"raw\":" + String(valorADC) + ",");
              client.println("\"base\":" + String(lecturaBase) + ",");
              client.println("\"delta\":" + String(deltaADC) + ",");
              client.println("\"gas\":" + String(gasEstimado) + ",");
              client.println("\"porcentaje\":" + String(porcentajeICA) + ",");
              client.println("\"estado\":\"" + estadoICA + "\",");
              client.println("\"color\":\"" + colorICA + "\",");
              client.println("\"recomendacion\":\"" + recomendacionICA + "\"");
              client.println("}");
              break;
            }

            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();

            client.println("<html><head><meta charset='UTF-8'>");
            client.println("<meta name='viewport' content='width=device-width, initial-scale=1'>");
            client.println("<title>Panel de Calidad del Aire</title>");
            client.println("<style>");
            client.println(":root{--ink:#0f172a;--muted:#475569;--surface:#ffffff;--surface-soft:#f8fafc;--line:#dbe4f0;--brand:#0f766e;--brand-deep:#0b3b59;--accent:" + colorICA + ";--accent-soft:rgba(255,255,255,0.14);}");
            client.println("*{box-sizing:border-box;scroll-behavior:smooth;}");
            client.println("body{margin:0;font-family:Verdana,Arial,sans-serif;background:radial-gradient(circle at top left,#dcfce7 0%,#e0f2fe 35%,#f8fafc 100%);color:var(--ink);min-height:100vh;padding:20px;}");
            client.println(".shell{width:min(1180px,100%);margin:0 auto;}");
            client.println(".nav,.section,.hero-panel,.status-panel,.chart-panel,.legend-panel,.info-panel,.actions-panel{background:rgba(255,255,255,0.9);backdrop-filter:blur(12px);border:1px solid rgba(219,228,240,0.8);box-shadow:0 18px 55px rgba(15,23,42,0.12);}");
            client.println(".nav{display:flex;flex-wrap:wrap;align-items:center;justify-content:space-between;gap:16px;padding:18px 22px;border-radius:24px;margin-bottom:18px;}");
            client.println(".brand{display:flex;align-items:center;gap:14px;}");
            client.println(".brand-badge{width:46px;height:46px;border-radius:14px;background:linear-gradient(135deg,var(--brand-deep),var(--brand));display:grid;place-items:center;color:#fff;font-weight:bold;box-shadow:0 14px 32px rgba(11,59,89,0.25);}");
            client.println(".brand h1{font-size:20px;margin:0 0 4px;} .brand p{margin:0;font-size:13px;color:var(--muted);}");
            client.println(".nav-links{display:flex;flex-wrap:wrap;gap:10px;}");
            client.println(".nav-links a,.action-btn{display:inline-flex;align-items:center;justify-content:center;padding:11px 16px;border-radius:999px;text-decoration:none;font-size:13px;font-weight:bold;transition:transform .2s ease,box-shadow .2s ease;}");
            client.println(".nav-links a{background:#e6f4f1;color:var(--brand-deep);border:1px solid #c9ebe4;}");
            client.println(".action-btn.primary{background:linear-gradient(135deg,var(--brand-deep),var(--brand));color:#fff;box-shadow:0 12px 28px rgba(15,118,110,0.25);}");
            client.println(".action-btn.secondary{background:#eff6ff;color:var(--brand-deep);border:1px solid #cfe1ff;}");
            client.println(".nav-links a:hover,.action-btn:hover{transform:translateY(-1px);}");
            client.println(".hero-grid,.dashboard-grid,.meta-grid,.actions-grid{display:grid;gap:18px;}");
            client.println(".hero-grid{grid-template-columns:1.35fr .95fr;margin-bottom:18px;}");
            client.println(".hero-panel{padding:28px;border-radius:28px;position:relative;overflow:hidden;}");
            client.println(".hero-panel:before{content:'';position:absolute;inset:auto -80px -90px auto;width:220px;height:220px;background:radial-gradient(circle,#99f6e4 0%,rgba(153,246,228,0) 70%);}");
            client.println(".eyebrow{display:inline-flex;align-items:center;gap:8px;padding:8px 12px;border-radius:999px;background:#ecfeff;color:var(--brand-deep);font-size:12px;letter-spacing:.12em;text-transform:uppercase;font-weight:bold;}");
            client.println(".hero-title{font-size:clamp(30px,4.8vw,54px);line-height:1.02;margin:18px 0 12px;max-width:10ch;}");
            client.println(".hero-copy{margin:0;max-width:58ch;color:var(--muted);line-height:1.65;}");
            client.println(".hero-actions{display:flex;flex-wrap:wrap;gap:12px;margin-top:22px;}");
            client.println(".hero-note{margin-top:18px;font-size:13px;color:#64748b;}");
            client.println(".status-panel{padding:26px;border-radius:28px;background:linear-gradient(180deg,#0f172a 0%,#132238 100%);color:#e2e8f0;}");
            client.println(".status-top{display:flex;justify-content:space-between;gap:12px;align-items:flex-start;}");
            client.println(".status-label{font-size:13px;text-transform:uppercase;letter-spacing:.14em;color:#94a3b8;}");
            client.println(".status-badge{display:inline-flex;align-items:center;gap:10px;padding:10px 14px;border-radius:999px;background:var(--accent-soft);font-size:13px;font-weight:bold;}");
            client.println(".dot{width:12px;height:12px;border-radius:50%;background:var(--accent);box-shadow:0 0 18px var(--accent);}");
            client.println(".status-value{font-size:clamp(68px,11vw,118px);font-weight:800;line-height:.95;color:var(--accent);margin:18px 0 10px;}");
            client.println(".status-unit{font-size:18px;color:#cbd5e1;margin-bottom:18px;}");
            client.println(".meter{height:14px;border-radius:999px;background:rgba(255,255,255,0.08);overflow:hidden;border:1px solid rgba(255,255,255,0.08);}");
            client.println(".meter-fill{height:100%;border-radius:999px;background:linear-gradient(90deg,#22c55e 0%,#facc15 35%,#f97316 60%,#ef4444 82%,#6a040f 100%);width:" + String(porcentajeICA) + "%;}");
            client.println(".meter-row{display:flex;justify-content:space-between;font-size:13px;color:#94a3b8;margin-top:8px;}");
            client.println(".dashboard-grid{grid-template-columns:1.1fr .9fr;margin-bottom:18px;}");
            client.println(".section{padding:22px;border-radius:24px;}");
            client.println(".section h2{margin:0 0 14px;font-size:19px;}");
            client.println(".section p{margin:0;color:var(--muted);line-height:1.55;}");
            client.println(".kpi-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(170px,1fr));gap:14px;}");
            client.println(".kpi{background:var(--surface-soft);border:1px solid var(--line);border-radius:20px;padding:18px;}");
            client.println(".kpi span{display:block;font-size:12px;text-transform:uppercase;letter-spacing:.08em;color:#64748b;margin-bottom:8px;}");
            client.println(".kpi strong{display:block;font-size:28px;color:var(--ink);margin-bottom:8px;}");
            client.println(".kpi small{display:block;color:var(--muted);line-height:1.45;}");
            client.println(".chart-panel{padding:22px;border-radius:24px;background:linear-gradient(180deg,#f8fafc 0%,#eefbf7 100%);}");
            client.println(".chart-card{padding:18px;border-radius:20px;background:#fff;border:1px solid var(--line);margin-top:14px;}");
            client.println(".chart-track{display:flex;align-items:flex-end;gap:10px;height:180px;margin-top:16px;}");
            client.println(".bar{flex:1;border-radius:16px 16px 6px 6px;background:linear-gradient(180deg,#7dd3fc 0%,var(--accent) 100%);min-width:28px;position:relative;}");
            client.println(".bar span{position:absolute;left:50%;transform:translateX(-50%);bottom:-28px;font-size:12px;color:#64748b;white-space:nowrap;}");
            client.println(".bar.current:after{content:'Actual';position:absolute;top:10px;left:50%;transform:translateX(-50%);font-size:11px;color:#fff;font-weight:bold;}");
            client.println(".legend-list,.meta-list{display:grid;gap:12px;margin-top:14px;}");
            client.println(".legend-item,.meta-item{display:flex;justify-content:space-between;gap:12px;align-items:center;padding:14px 16px;border-radius:18px;background:var(--surface-soft);border:1px solid var(--line);}");
            client.println(".legend-item strong,.meta-item strong{font-size:14px;}");
            client.println(".legend-chip{padding:7px 12px;border-radius:999px;color:#fff;font-size:12px;font-weight:bold;}");
            client.println(".meta-grid{grid-template-columns:1fr 1fr;margin-bottom:18px;}");
            client.println(".recommendation{border-left:4px solid var(--accent);padding:16px 18px;background:#fff;border-radius:18px;margin-top:14px;box-shadow:inset 0 0 0 1px rgba(219,228,240,0.8);}");
            client.println(".recommendation strong{display:block;margin-bottom:6px;}");
            client.println(".actions-grid{grid-template-columns:repeat(auto-fit,minmax(220px,1fr));}");
            client.println(".action-card{padding:20px;border-radius:22px;background:#fff;border:1px solid var(--line);}");
            client.println(".action-card h3{margin:0 0 10px;font-size:17px;}");
            client.println(".action-card p{margin:0 0 16px;color:var(--muted);line-height:1.55;}");
            client.println(".footer{display:flex;flex-wrap:wrap;gap:12px;justify-content:space-between;font-size:13px;color:#64748b;padding:12px 2px 2px;}");
            client.println("@media (max-width:900px){.hero-grid,.dashboard-grid,.meta-grid{grid-template-columns:1fr;}.hero-title{max-width:none;}}");
            client.println("@media (max-width:640px){body{padding:12px;}.nav,.hero-panel,.status-panel,.section,.chart-panel{padding:18px;}.status-value{font-size:72px;}.chart-track{height:140px;}.nav-links{width:100%;}.nav-links a{flex:1;}}");
            client.println("</style></head>");
            client.println("<body><main class='shell'>");
            client.println("<nav class='nav'>");
            client.println("<div class='brand'><div class='brand-badge'>AQ</div><div><h1>Panel de Calidad del Aire</h1><p>Monitoreo ambiental en tiempo real con ESP32 y MQ135</p></div></div>");
            client.println("<div class='nav-links'><a href='#resumen'>Resumen</a><a href='#riesgo'>Riesgo</a><a href='#sistema'>Sistema</a></div>");
            client.println("</nav>");
            client.println("<section class='hero-grid'>");
            client.println("<article class='hero-panel' id='resumen'>");
            client.println("<div class='eyebrow'>Monitoreo Ambiental</div>");
            client.println("<h2 class='hero-title'>Calidad del aire en tiempo real para espacios interiores</h2>");
            client.println("<p class='hero-copy'>Este panel presenta un resumen ejecutivo del ambiente medido por el sensor. El sistema normaliza la lectura RAW respecto a la calibracion base y la transforma en un ICA estimado para facilitar la toma de decisiones.</p>");
            client.println("<div class='hero-actions'><a class='action-btn primary' href='#sistema'>Ver sistema</a><a class='action-btn secondary' href='#acciones'>Acciones sugeridas</a></div>");
            client.println("<div class='hero-note'>Actualizacion automatica cada 1 segundo sobre la red WiFi propia del dispositivo.</div>");
            client.println("</article>");
            client.println("<aside class='status-panel'>");
            client.println("<div class='status-top'><div><div class='status-label'>Indice ICA estimado</div></div><div class='status-badge' id='estadoBadge'><span class='dot'></span>" + estadoICA + "</div></div>");
            client.println("<div class='status-value' id='icaValor'>" + String(icaEstimado) + "</div>");
            client.println("<div class='status-unit'>Escala de evaluacion: 0 a 500</div>");
            client.println("<div class='meter'><div class='meter-fill' id='meterFill'></div></div>");
            client.println("<div class='meter-row'><span>0 saludable</span><span>500 critico</span></div>");
            client.println("</aside>");
            client.println("</section>");
            client.println("<section class='dashboard-grid'>");
            client.println("<article class='section'>");
            client.println("<h2>Resumen operativo</h2>");
            client.println("<div class='kpi-grid'>");
            client.println("<div class='kpi'><span>Lectura RAW</span><strong id='rawValor'>" + String(valorADC) + "</strong><small>Valor directo entregado por el ADC antes de la normalizacion.</small></div>");
            client.println("<div class='kpi'><span>Lectura base</span><strong id='baseValor'>" + String(lecturaBase) + "</strong><small>Referencia tomada cuando el MQ135 se detecto estable.</small></div>");
            client.println("<div class='kpi'><span>Delta RAW</span><strong id='deltaValor'>" + String(deltaADC) + "</strong><small>Aumento real usado para evitar saltos causados por una base alta.</small></div>");
            client.println("<div class='kpi'><span>Gas estimado</span><strong id='gasValor'>" + String(gasEstimado) + "</strong><small>Conversion estimada a partir de la referencia base del sensor.</small></div>");
            client.println("<div class='kpi'><span>Estado actual</span><strong id='estadoValor'>" + estadoICA + "</strong><small>Clasificacion del riesgo segun el ICA estimado.</small></div>");
            client.println("<div class='kpi'><span>Red activa</span><strong>Online</strong><small>Punto de acceso local listo para consulta desde celulares o portatiles.</small></div>");
            client.println("</div>");
            client.println("<div class='recommendation'><strong>Recomendacion inmediata</strong><span id='recomendacionValor'>" + recomendacionICA + "</span></div>");
            client.println("</article>");
            client.println("<aside class='chart-panel'>");
            client.println("<h2>Tendencia visual</h2>");
            client.println("<p>Vista comparativa simplificada para dar una lectura mas parecida a un tablero web profesional.</p>");
            client.println("<div class='chart-card'><div class='chart-track'>");
            client.println("<div class='bar' style='height:28%;'><span>Base</span></div>");
            client.println("<div class='bar' style='height:44%;'><span>Seguro</span></div>");
            client.println("<div class='bar' style='height:62%;'><span>Vigilancia</span></div>");
            client.println("<div class='bar current' id='barraActual' style='height:" + String(max(18, porcentajeICA)) + "%;'><span>Actual</span></div>");
            client.println("</div></div>");
            client.println("</aside>");
            client.println("</section>");
            client.println("<section class='meta-grid'>");
            client.println("<article class='section legend-panel' id='riesgo'>");
            client.println("<h2>Niveles de riesgo</h2>");
            client.println("<div class='legend-list'>");
            client.println("<div class='legend-item'><strong>Verde</strong><span class='legend-chip' style='background:#2dc653;'>0 - 166</span></div>");
            client.println("<div class='legend-item'><strong>Azul</strong><span class='legend-chip' style='background:#0284c7;'>167 - 334</span></div>");
            client.println("<div class='legend-item'><strong>Rojo</strong><span class='legend-chip' style='background:#e63946;'>335 - 500</span></div>");
            client.println("</div>");
            client.println("</article>");
            client.println("<article class='section info-panel' id='sistema'>");
            client.println("<h2>Informacion del sistema</h2>");
            client.println("<div class='meta-list'>");
            client.println("<div class='meta-item'><span>IP del ESP32</span><strong>" + WiFi.softAPIP().toString() + "</strong></div>");
            client.println("<div class='meta-item'><span>SSID</span><strong>" + String(ssid) + "</strong></div>");
            client.println("<div class='meta-item'><span>Formula base</span><strong>ICA por interpolacion lineal</strong></div>");
            client.println("<div class='meta-item'><span>Metodo de base</span><strong>Estabilidad automatica por delta RAW</strong></div>");
            client.println("<div class='meta-item'><span>Criterio estable</span><strong>Delta <= " + String(DELTA_ESTABLE_MAX) + " por " + String(LECTURAS_ESTABLES_NECESARIAS) + " lecturas</strong></div>");
            client.println("<div class='meta-item'><span>Guia de calibracion</span><strong>Min " + minutosRedondeados(PRECALENTAMIENTO_MINIMO_MS) + " / Recom " + minutosRedondeados(PRECALENTAMIENTO_RECOMENDADO_MS) + " / Ideal " + minutosRedondeados(PRECALENTAMIENTO_IDEAL_MS) + "</strong></div>");
            client.println("<div class='meta-item'><span>Diagnostico sensor</span><strong>" + diagnosticoSensor + "</strong></div>");
            client.println("<div class='meta-item'><span>Intervalo web</span><strong>1 s</strong></div>"); // <-- CAMBIADO DE 3 s A 1 s
            client.println("<div class='meta-item'><span>Clientes conectados</span><strong>" + String(WiFi.softAPgetStationNum()) + "</strong></div>");
            client.println("</div>");
            client.println("</article>");
            client.println("</section>");
            client.println("<section class='section actions-panel' id='acciones'>");
            client.println("<h2>Opciones y acciones</h2>");
            client.println("<div class='actions-grid'>");
            client.println("<article class='action-card'><h3>Revision rapida</h3><p>Consultar el estado general del espacio y validar si la condicion es apta para operacion normal.</p><a class='action-btn secondary' href='#resumen'>Abrir resumen</a></article>");
            client.println("<article class='action-card'><h3>Gestion preventiva</h3><p>Usar la recomendacion del panel para decidir ventilacion, pausa o ajustes operativos.</p><a class='action-btn secondary' href='#riesgo'>Ver riesgo</a></article>");
            client.println("<article class='action-card'><h3>Soporte tecnico</h3><p>Verificar conectividad local, IP del equipo y estado del sistema para mantenimiento basico.</p><a class='action-btn secondary' href='#sistema'>Ver sistema</a></article>");
            client.println("</div>");
            client.println("</section>");
            client.println("<div class='footer'><div>Formula usada: ICA=((Ihi-Ilo)/(BPhi-BPlo))*(Cp-BPlo)+Ilo</div><div>Si el celular dice sin internet, mantenerse conectado y abrir http://192.168.4.1</div></div>");
            client.println("<script>");
            client.println("function setText(id,value){const el=document.getElementById(id);if(el)el.textContent=value;}");
            client.println("async function actualizarDatos(){try{const r=await fetch('/data',{cache:'no-store'});const d=await r.json();document.documentElement.style.setProperty('--accent',d.color);setText('icaValor',d.ica);setText('rawValor',d.raw);setText('baseValor',d.base);setText('deltaValor',d.delta);setText('gasValor',d.gas);setText('estadoValor',d.estado);setText('recomendacionValor',d.recomendacion);const badge=document.getElementById('estadoBadge');if(badge)badge.innerHTML='<span class=\"dot\"></span>'+d.estado;const meter=document.getElementById('meterFill');if(meter)meter.style.width=d.porcentaje+'%';const bar=document.getElementById('barraActual');if(bar)bar.style.height=Math.max(18,d.porcentaje)+'%';}catch(e){}}");
            client.println("setInterval(actualizarDatos,1000);");
            client.println("</script>");
            client.println("</main></body></html>");
            client.println();
            break;
          } else {
            if (requestLine.length() == 0) {
              requestLine = currentLine;
            }
            currentLine = "";
          }
        } else if (c != '\r') {
          currentLine += c;
        }
      }
    }
    client.stop();
  }
}
