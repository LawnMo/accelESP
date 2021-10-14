#include <Arduino.h>
#include <SPI.h>
#include <ESP8266WiFi.h>
#include <WifiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>

#define STASSID "YOURSSIDHERE"
#define STAPSK "YOURWIFIPSKHERE"

// D5 = SCK
// D6 = MISO => SDO
// D7 = MOSI => SDA
#define CS D8
#define INT D1
#define WAI_WRITE 0x0f
#define WAI_VALUE 0x33
#define REG_CTRL1 0x20
#define REG_OUTXL 0x28
#define REG_FIFO_CTRL 0x2e
#define REG_FIFO_SRC 0x2f
#define ENABLE_X_Y_Z 0x07
#define RESOLUTION 12

alignas(2) volatile uint8_t transfer_buffer[2 + (6 * 32)];
volatile uint8_t* const data_buffer = transfer_buffer + 2;
uint8_t ctrl_reg1;
volatile uint32_t total_numread;
uint32_t total_handled;
bool lis3dh_found = false;

SPISettings spi_setting = SPISettings(2000000, MSBFIRST, SPI_MODE0);
ESP8266WebServer server(80);

#define TRANSACT_SPI(x) do {         \
  SPI.beginTransaction(spi_setting); \
  digitalWrite(CS, LOW);             \
  x;                                 \
  digitalWrite(CS, HIGH);            \
  SPI.endTransaction();              \
} while (0)

void read_registers(uint8_t base_reg, size_t num_to_read) {
  transfer_buffer[1] = base_reg | 0xC0;
  TRANSACT_SPI(SPI.transfer((uint8_t *)transfer_buffer + 1, 1 + num_to_read));
}

uint8_t read_register(uint8_t reg) {
  read_registers(reg, 1);
  return data_buffer[0];
}

void write_registers(uint8_t base_reg, size_t num_to_write) {
  if (base_reg < 0x1e) return;
  transfer_buffer[1] = base_reg | 0x40;
  TRANSACT_SPI(SPI.transfer((uint8_t *)transfer_buffer + 1, 1 + num_to_write));
}

void write_register(uint8_t reg, uint8_t value) {
  data_buffer[0] = value;
  write_registers(reg, 1);
}

void setupSPI() {
  pinMode(CS, OUTPUT);
  digitalWrite(CS, HIGH);
  SPI.begin();
}

void setupLIS3DH() {
  uint8_t wai = read_register(WAI_WRITE);
  lis3dh_found = wai == WAI_VALUE;
  if (wai != WAI_VALUE) {
    Serial.println(F("LIS3DH missing"));
  } else {
    Serial.println(F("LIS3DH found"));
  }

  ctrl_reg1 = 0;
  uint8_t ctrl_reg4 = 1 << 7; // block data update
  ctrl_reg4 |= 1 << 3; // high res
  uint8_t odr = 0x9; // 1344Hz

  ctrl_reg1 |= odr << 4;

  data_buffer[0] = 0; // power down mode
  data_buffer[1] = 0; // ctrl2 high pass not used
  data_buffer[2] = 1u << 2; // ctrl3 enable fifo watermark interrupt
  data_buffer[3] = ctrl_reg4;
  data_buffer[4] = 1u << 6; // ctrl5 enable fifo
  data_buffer[5] = 0; // ctrl6 int2 disabled, active high interrupts
  write_registers(REG_CTRL1, 6);
  write_register(REG_FIFO_CTRL, (2u << 6) | (20 - 1));

}

volatile bool fifo_read_error = false;
volatile uint32_t first_time = 0;
volatile uint16_t read_pos = 0;
volatile uint16_t write_pos = 0;
void start_collecting() {
  ctrl_reg1 |= ENABLE_X_Y_Z;
  while ((read_register(REG_FIFO_SRC) & (1 << 5)) == 0) {
    read_registers(REG_OUTXL, 6); // drain fifo
  }
  total_numread = 0;
  total_handled = 0;
  write_register(REG_CTRL1, ctrl_reg1);
  first_time = 0;
  fifo_read_error = false;
  write_pos = 0;
  read_pos = 0;
}

#define RING_BUFFER_LEN 16380
#define RING_BUFFER_LEN_U8 (RING_BUFFER_LEN * 2)
volatile uint16_t ring_buffer[RING_BUFFER_LEN];
volatile uint8_t *const ring_buffer_u8 = (uint8_t *)ring_buffer;
volatile uint16_t fifo_overflows = 0;
volatile uint16_t fifo_datarate = 0;
// perform FIFO read in an interrupt handler. Polling INT and reading the FIFO inline overflows
// the remote FIFO. Store into a local ring buffer to delay overflow caused by FP math converting to G
void IRAM_ATTR read_lis3dh_fifo() {
  if (first_time == 0)
    first_time = millis();
  uint8_t fifo_status = read_register(REG_FIFO_SRC);
  uint8_t num_to_read = fifo_status & 0x1f;
  if (num_to_read == 0 && (fifo_status & 0x20) == 0)
    num_to_read = 32;
  if (num_to_read != 0) {
    read_registers(REG_OUTXL, 6 * num_to_read);
  } else {
    fifo_read_error = true;
    return;
  }
  
  if ((fifo_status & 0x40) != 0)
    ++fifo_overflows;

  total_numread += num_to_read;
  const uint32_t interval = millis() - first_time;
  fifo_datarate = (total_numread == 0 || interval == 0) ? 0 : (total_numread * 1000)/interval;
  if (write_pos + num_to_read * 6 > RING_BUFFER_LEN_U8) {
    uint16_t first = RING_BUFFER_LEN_U8 - write_pos;
    uint16_t remaining = (num_to_read * 6) - first;
    memcpy((uint8_t *)ring_buffer_u8 + write_pos, (uint8_t *)data_buffer, first);
    memcpy((uint8_t *)ring_buffer_u8, (uint8_t *)data_buffer + first, remaining);
  } else {
    memcpy((uint8_t *)ring_buffer_u8 + write_pos, (uint8_t *)data_buffer, num_to_read * 6);
  }
  write_pos = (write_pos + 6 * num_to_read) % RING_BUFFER_LEN_U8;
}

void stop_collecting() {
  write_register(REG_CTRL1, 0);
}

const uint16_t sign_mask = (1u << RESOLUTION) - 1;
const float resolution_divisor = (float) (1u << (RESOLUTION - 2));

void setup() {
  int led_state = 0;
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(INT, INPUT);
  digitalWrite(LED_BUILTIN, led_state = !led_state);

  Serial.begin(115200);
  Serial.println("BOOT");
  attachInterrupt(digitalPinToInterrupt(INT), read_lis3dh_fifo, RISING);

  setupSPI();
  setupLIS3DH();

  WiFi.mode(WIFI_STA);
  WiFi.begin(STASSID, STAPSK);

  server.on("/data", []() {
    if (!lis3dh_found) {
      server.send(404, "text/plain", "LIS3DH not found\n");
      return;
    }
    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "text/plain", "");
    char buf[48];
    float in_g[3];
    server.sendContent("Sample,X,Y,Z,Overflows,DataRate\n");
    start_collecting();
    while (server.client().connected()) {

      if (fifo_read_error) {
        server.sendContent(F("Failed to retrieve data from accelerometer\n"));
        break;
      }
      while (total_numread <= total_handled);
      uint16_t samples_read = total_numread - total_handled;
      if (samples_read > RING_BUFFER_LEN) {
        server.sendContent(F("Overflowed local buffer\n"));
        break;
      }
      if (total_handled == 0) {
        read_pos = (read_pos + 3) % RING_BUFFER_LEN;
        samples_read--;
      }

      while (samples_read > 0) {
        for (int i = 0; i < 3; ++i) {
          int dataVal = ring_buffer[(read_pos + i) % RING_BUFFER_LEN];
          dataVal >>= 16 - RESOLUTION;
          if (dataVal & (1u << (RESOLUTION - 1)))
            dataVal |= ~sign_mask;

          in_g[i] = (float)(int16_t) dataVal / resolution_divisor;
        }
        snprintf(buf, 48, "%d,%.3f,%.3f,%.3f,%d,%d\n", total_handled++,
          in_g[0], in_g[1], in_g[2],
          fifo_overflows, fifo_datarate);

        read_pos = (read_pos + 3) % RING_BUFFER_LEN;
        --samples_read;
        server.sendContent(buf);
      }

    }
    stop_collecting();
  });

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    digitalWrite(LED_BUILTIN, led_state = !led_state);
  }
  Serial.println("WIFI UP");

  if (!MDNS.begin("lis3dh")) {
    while (true) {
        delay(250);
        digitalWrite(LED_BUILTIN, led_state = !led_state);
    }
  }
  Serial.println("MDNS UP");

  server.begin();
  Serial.println("HTTP UP");
  digitalWrite(LED_BUILTIN, 1);
  Serial.print("Free RAM: ");
  Serial.println(system_get_free_heap_size());
}

void loop() {
  static unsigned long blink = 0;
  static int led_state = 0;
  if (!lis3dh_found) {
    if (millis() - blink > 45) {
      digitalWrite(LED_BUILTIN, led_state = !led_state);
      blink = millis();
    }
  }
  server.handleClient();
  MDNS.update();
}
