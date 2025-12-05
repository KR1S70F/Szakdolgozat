#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <driver/i2s.h>
#include <arduinoFFT.h>
#include "FS.h"
#include "SPIFFS.h"

// 1. WiFi Hitelesítő Adatok
const char* ssid = "Kristof"; 
const char* password = "tesztfft";

// 2. I2S/INMP441 Pin Konfiguráció (ESP32 Tűk)
#define I2S_PORT I2S_NUM_0
#define I2S_SCK_PIN 18   // Bit Clock
#define I2S_WS_PIN 19    // Word Select / Left-Right Clock
#define I2S_SD_PIN 21    // Serial Data In (INMP441 Data Out)
#define I2S_SAMPLE_RATE 44100
#define I2S_BUFFER_SIZE 1024




// ----------------------------------------------------------------------------------
// --- FFT Konfiguráció ---
// ----------------------------------------------------------------------------------

const uint16_t SAMPLES = 1024; // Az FFT mérete (2 hatványa)
const double SAMPLING_FREQUENCY = 44100; // Mintavételi frekvencia (Hz)
// const double FREQUENCY_RESOLUTION = SAMPLING_FREQUENCY / SAMPLES; // ~43 Hz

double vReal[SAMPLES]; // Valós számok tömbje
double vImag[SAMPLES]; // Képzetes számok tömbje (mindig 0.0)

ArduinoFFT FFT = ArduinoFFT(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);

// ----------------------------------------------------------------------------------
// --- 3. MENTÉSI ÉS WEBSERVER KONFIGURÁCIÓ ---
// ----------------------------------------------------------------------------------

// Mentési konfiguráció (SPIFFS)
const char* FILENAME = "/fft_data.csv"; 
const int LOG_INTERVAL = 500;           // Mentés minden 500. ciklusonként
int logCounter = 0;                     
char logBuffer[20480];                  // 20 KB puffer
int bufferIndex = 0;

// Szerver és WebSocket objektumok
WebSocketsServer webSocket = WebSocketsServer(81); 
WebServer server(80); // HTTP WebServer

// MOZGÓÁTLAG SZŰRŐ MÉRETE (ablakszélesség)
// Kísérletezéssel állítandó be (pl. 4 és 16 között)
const int MA_FILTER_SIZE = 8; 

// Puffer a szűrt adatok tárolására, a vReal és vImag után
double filtered_vReal[SAMPLES];


// EXPONENCIÁLIS MOZGÓÁTLAG (EMA) SZŰRŐ KONFIGURÁCIÓ

// Alpha (simító faktor): 0 és 1 közötti érték. 
// Kisebb ALPHA = ERŐSEBB simítás
const double ALPHA = 0.1; 

// Állapotváltozó: Az előző szűrt kimenet tárolása. 
// Ennek globálisnak kell lennie, hogy a loop() hívások között megmaradjon az értéke.
double ema_previous_output = 0.0;

// ----------------------------------------------------------------------------------
// --- WebSocket Konfiguráció ---
// ----------------------------------------------------------------------------------

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  // A kliens események kezelése (itt csak logolás történik)
  switch (type) {
    case WStype_CONNECTED:
      Serial.printf("[%u] Kliens csatlakozott. IP: %s\n", num, webSocket.remoteIP(num).toString().c_str());
      break;
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Kliens lecsatlakozott.\n", num);
      break;
    default:
      break;
  }
}
void handleRoot() {
  String html = "<h1>ESP32 FFT Adatmentés</h1>";
  html += "<p>Kattintson ide a mentett CSV adatok letöltéséhez:</p>";
  html += "<p><a href='/download'>fft_data.csv letöltése</a></p>";
  html += "<p>A valós idejű grafikon a WebSocket porton fut (81). Keressen ra a forraskodban megadott IP-cimen.</p>";

  server.send(200, "text/html", html);
}

void handleDownload() {
  if (SPIFFS.exists(FILENAME)) {
    File file = SPIFFS.open(FILENAME, "r");
    server.streamFile(file, "text/csv");
    file.close();
  } else {
    server.send(404, "text/plain", "404: A mentesi fajl ('fft_data.csv') nem talalhato.");
  }
}

void handleNotFound() {
  server.send(404, "text/plain", "404: File Not Found");
}

// ----------------------------------------------------------------------------------
// --- SETUP ---
// ----------------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(1000);

  // SPIFFS Inicializálás
  if (!SPIFFS.begin(true)) {
    Serial.println("Hiba történt a SPIFFS mountolása közben!");
    return;
  }
  Serial.println("SPIFFS sikeresen inicializálva.");

  // SPIFFS Buffer Fejlécének Előkészítése
  bufferIndex += sprintf(logBuffer + bufferIndex, "Timestamp,");
  for (int i = 0; i < SAMPLES / 2; i++) {
    bufferIndex += sprintf(logBuffer + bufferIndex, "Frekvencia_%d%s", i, (i == SAMPLES / 2 - 1) ? "" : ",");
  }
  bufferIndex += sprintf(logBuffer + bufferIndex, "\n");

  // WiFi csatlakozás
  Serial.print("Csatlakozás a WiFi-hez...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi csatlakoztatva.");
  Serial.print("ESP32 IP címe: ");
  Serial.println(WiFi.localIP());
  Serial.println("Kliens csatlakoztatásához nyissa meg az IP-címet a böngészőben, 81-es porton.");

  // I2S Konfiguráció
  const i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = I2S_SAMPLE_RATE,
    .bits_per_sample= I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format=I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format=I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags=ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len=I2S_BUFFER_SIZE,
    .use_apll = false
  };

  const i2s_pin_config_t pin_config = {
      .bck_io_num = I2S_SCK_PIN,
      .ws_io_num = I2S_WS_PIN,
      .data_out_num = -1,
      .data_in_num = I2S_SD_PIN
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config);
  i2s_set_clk(I2S_PORT, (int)SAMPLING_FREQUENCY, I2S_BITS_PER_SAMPLE_24BIT, I2S_CHANNEL_MONO); // Monó módot állítunk be

  // WebSocket indítása
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  server.on("/", handleRoot);
  server.on("/download", handleDownload);
  server.onNotFound(handleNotFound);
  server.begin(); 
  Serial.println("Web Server elindult a 80-as porton.");
}

// ----------------------------------------------------------------------------------
// --- LOOP ---
// ----------------------------------------------------------------------------------

void loop() {
  webSocket.loop(); // Websocket kliensek kezelése
  server.handleClient();

  // Deklarációk a loop() belsejében
  size_t bytes_read;
  uint32_t i2s_read_buffer[SAMPLES]; 

  // 1. ADATGYŰJTÉS
  // Megpróbáljuk beolvasni az I2S adatot
  i2s_read(I2S_PORT, (char*)i2s_read_buffer, sizeof(i2s_read_buffer), &bytes_read, portMAX_DELAY);

  if (bytes_read > 0) { 
    
    // 2. MINTÁK ELŐKÉSZÍTÉSE ÉS ÁTADÁSA AZ FFT-NEK
    for (int i = 0; i < SAMPLES; i++) {
      // Az I2S 24 bites mintái a 32 bites szó felső részén helyezkednek el.
      // Eltoljuk 8 bittel jobbra, hogy a 24 bit a 0. pozícióra kerüljön.
      int32_t sample = i2s_read_buffer[i] >> 8; 
      vReal[i] = (double)sample;
      vImag[i] = 0.0;
    }

   /* 
    // --------------------------------------------------------------------
    // 2.5 MOZGÓÁTLAG SZŰRŐ ALKALMAZÁSA (TIME DOMAIN)
    // --------------------------------------------------------------------
    for (int i = 0; i < SAMPLES; i++) {
        double sum = 0.0;
        int count = 0;
    
        // Az ablak visszamenőlegesen gyűjti az adatokat az i-j pontokból
        for (int j = 0; j < MA_FILTER_SIZE; j++) {
            int index = i - j;
        
            // Csak az érvényes, már beolvasott indexeket használjuk
            if (index >= 0) {
                sum += vReal[index];
                count++;
            }
        }
    
        // Szűrt átlag beállítása
        if (count > 0) {
            filtered_vReal[i] = sum / count;
        } else {
            filtered_vReal[i] = vReal[i]; 
        }
    }

    // Az eredeti vReal tömb felülírása a szűrt értékekkel
    for (int i = 0; i < SAMPLES; i++) {
        vReal[i] = filtered_vReal[i];
    }
    
    */
    // --------------------------------------------------------------------
    // 2.5 EXPONENCIÁLIS MOZGÓÁTLAG (EMA) SZŰRŐ ALKALMAZÁSA (TIME DOMAIN)
    // Formula: Yt = ALPHA * Xt + (1 - ALPHA) * Yt-1
    // --------------------------------------------------------------------
    
    for (int i = 0; i < SAMPLES; i++) {
        double Xt = vReal[i]; // A jelenlegi nyers bemenet
        double Yt;           // Az új szűrt kimenet

        if (i == 0) {
            // A ciklus első mintájánál az előző állapotot a globális változóból vesszük
            Yt = ALPHA * Xt + (1.0 - ALPHA) * ema_previous_output;
        } else {
            // A további mintáknál az előző, már szűrt értéket használjuk a vReal[i-1]-ből
            Yt = ALPHA * Xt + (1.0 - ALPHA) * vReal[i - 1]; 
        }
        
        // Az eredeti mintát felülírjuk a szűrt értékkel (inplace filtering)
        vReal[i] = Yt;
    }
    
    // Frissítjük a globális állapotot a következő loop() híváshoz
    // (a jelenlegi adathalmaz utolsó szűrt mintáját mentjük el)
    ema_previous_output = vReal[SAMPLES - 1]; 
    

    // 3. FFT SZÁMÍTÁS
    FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD); // Hamming ablak (spektrális szivárgás csökkentése)
    FFT.compute(FFT_FORWARD);
    FFT.complexToMagnitude(); // Komplex számokból Amplitúdó (Magnitude)

    const int NOISE_THRESHOLD = 1000000; //

   for (int i = 0; i < SAMPLES / 2; i++) { 
        if (vReal[i] < NOISE_THRESHOLD) {
          vReal[i] = 0.0; // Ha a zajszint alatt van, lenullázzuk
      }
    }
    
    
    // 4. ADATOK ELKÜLDÉSE WEBSOCKETEN KERESZTÜL
    String fftData = "";
    // Csak a spektrum felét kell vizsgálni (Nyquist-tétel: SAMPLES/2)
    for (int i = 0; i < SAMPLES / 2; i++) { 
        // Skálázás nélkül küldjük, a kliens oldali JS skálázza a grafikonhoz
        int magnitude = (int)vReal[i]; 
        fftData += String(magnitude);
        if (i < SAMPLES / 2 - 1) {
            fftData += ","; // Vesszővel elválasztjuk az értékeket
        }
    }
    
    // Küldés minden csatlakozott kliensnek (TXT formátumban)
    webSocket.broadcastTXT(fftData);

    // 5. ADATMENTÉS ÉS PUFFERELÉS (SPIFFS)
    logCounter++;
    if (logCounter >= LOG_INTERVAL) {
      
      // Adatok beírása a pufferbe
      bufferIndex += sprintf(logBuffer + bufferIndex, "%lu,", millis());
      
      for (int i = 0; i < SAMPLES / 2; i++) {
        int magnitude = (int)vReal[i]; 
        bufferIndex += sprintf(logBuffer + bufferIndex, "%d%s", magnitude, (i == SAMPLES / 2 - 1) ? "" : ",");
      }
      bufferIndex += sprintf(logBuffer + bufferIndex, "\n"); 
      
      logCounter = 0; 
      
      // ELLENŐRZÉS: Ha a puffer majdnem tele van, kiírjuk a fájlba.
      if (bufferIndex > sizeof(logBuffer) - 1024) { 
        File file = SPIFFS.open(FILENAME, FILE_APPEND);
        if (file) {
            file.write((uint8_t*)logBuffer, bufferIndex); 
            file.close();
            Serial.printf("SPIFFS-re írva: %d bájt mentve.\n", bufferIndex);
        } else {
            Serial.println("Hiba a mentési fájlba íráskor!");
        }
        
        bufferIndex = 0; // Puffer resetelése
      }
    }
  }
  yield();
  
}