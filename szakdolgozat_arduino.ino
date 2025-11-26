#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <driver/i2s.h>
#include <arduinoFFT.h>

// ----------------------------------------------------------------------------------
// --- BEÁLLÍTÁSOK (KÉRJÜK, EZT SZABJA TESTRE!) ---
// ----------------------------------------------------------------------------------

// 1. WiFi Hitelesítő Adatok
const char* ssid = "Kristof"; 
const char* password = "77772222";

// 2. I2S/INMP441 Pin Konfiguráció (ESP32 Tűk)
#define I2S_PORT I2S_NUM_0
#define I2S_SCK_PIN 33   // Bit Clock
#define I2S_WS_PIN 25    // Word Select / Left-Right Clock
#define I2S_SD_PIN 32    // Serial Data In (INMP441 Data Out)
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
// --- WebSocket Konfiguráció ---
// ----------------------------------------------------------------------------------

WebSocketsServer webSocket = WebSocketsServer(81); // 81-es port a WebSocketnek

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

// ----------------------------------------------------------------------------------
// --- SETUP ---
// ----------------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(1000);

  // 1. WiFi csatlakozás
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

  // 2. I2S Konfiguráció
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

  // 3. WebSocket indítása
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
}

// ----------------------------------------------------------------------------------
// --- LOOP ---
// ----------------------------------------------------------------------------------

void loop() {
  webSocket.loop(); // Websocket kliensek kezelése

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

    // 3. FFT SZÁMÍTÁS
    FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD); // Hamming ablak (spektrális szivárgás csökkentése)
    FFT.compute(FFT_FORWARD);
    FFT.complexToMagnitude(); // Komplex számokból Amplitúdó (Magnitude)
    
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
  }
}