/*********************************************************************
 This is an example for our nRF52 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

/*
 * This sketch demonstrate how to run both Central and Peripheral roles
 * at the same time. It will act as a relay between an central (mobile)
 * to another peripheral using bleuart service.
 * 
 * Mobile <--> DualRole <--> peripheral Ble Uart
 */
#include <bluefruit.h>

//*******Motion related
#include <Adafruit_APDS9960.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_LSM6DS33.h>
#include <Adafruit_SHT31.h>
#include <Adafruit_Sensor.h>
#include <PDM.h>
#include <Wire.h>
#include "Adafruit_DRV2605.h"

//Haptics Controller for vibratr disc
Adafruit_DRV2605 drv;

//Sensors on NRF52840 Sense
Adafruit_APDS9960 apds9960; // proximity, light, color, gesture
Adafruit_BMP280 bmp280;     // temperautre, barometric pressure
Adafruit_LIS3MDL lis3mdl;   // magnetometer
Adafruit_LSM6DS33 lsm6ds33; // accelerometer, gyroscope
Adafruit_SHT31 sht30;       // humidity

//Features Enable/Disable
#define VIBRATOR 0
#define PROXIMITY 1
#define TEMP_BARO 1
#define MAGNETO 1
#define ACC_GYRO 1
#define HUMIDITY 1
#define MIC 0 //Do not use.  Issue with programming DFU if this is enabled. 

//vibrator effect
uint8_t effect = 1;

//Sensors
uint8_t proximity;
uint16_t r, g, b, c;
float temperature, pressure, altitude;
float magnetic_x, magnetic_y, magnetic_z;
float accel_x, accel_y, accel_z;
float gyro_x, gyro_y, gyro_z;
float humidity;
int32_t mic;

extern PDMClass PDM;
short sampleBuffer[256];  // buffer to read samples into, each sample is 16-bits
volatile int samplesRead; // number of samples read

//*******End Motion related

// OTA DFU service
BLEDfu bledfu;

// Peripheral uart service
BLEUart bleuart;

// Central uart client
BLEClientUart clientUart;

void setup()
{
  uint32_t i;
  
  Serial.begin(115200);
  //while ( !Serial ) delay(10);   // for nrf52840 with native usb

  Serial.println("Bluefruit52 Dual Role BLEUART Sensors+Vibrator Example");
  Serial.println("-------------------------------------\n");
  
  // Initialize Bluefruit with max concurrent connections as Peripheral = 1, Central = 1
  // SRAM usage required by SoftDevice will increase with number of connections
  Bluefruit.begin(1, 1);
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values
  Bluefruit.setName("Bluefruit52 Duo Sense");

  // Callbacks for Peripheral
  Bluefruit.Periph.setConnectCallback(prph_connect_callback);
  Bluefruit.Periph.setDisconnectCallback(prph_disconnect_callback);

  // Callbacks for Central
  Bluefruit.Central.setConnectCallback(cent_connect_callback);
  Bluefruit.Central.setDisconnectCallback(cent_disconnect_callback);

  // To be consistent OTA DFU should be added first if it exists
  bledfu.begin();

  // Configure and Start BLE Uart Service
  bleuart.begin();
  bleuart.setRxCallback(prph_bleuart_rx_callback);

  // Init BLE Central Uart Serivce
  clientUart.begin();
  clientUart.setRxCallback(cent_bleuart_rx_callback);


  /* Start Central Scanning
   * - Enable auto scan if disconnected
   * - Interval = 100 ms, window = 80 ms
   * - Filter only accept bleuart service
   * - Don't use active scan
   * - Start(timeout) with timeout = 0 will scan forever (until connected)
   */
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.setInterval(160, 80); // in unit of 0.625 ms
  Bluefruit.Scanner.filterUuid(bleuart.uuid);
  Bluefruit.Scanner.useActiveScan(false);
  Bluefruit.Scanner.start(0);                   // 0 = Don't stop scanning after n seconds

  // Set up and start advertising
  startAdv();

  //*******Init motion related
  init_motion();

  Serial.println("setup done");
}

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include bleuart 128-bit uuid
  Bluefruit.Advertising.addService(bleuart);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();

  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   *
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds
}

void loop()
{
  // do nothing, all the work is done in callback

  loop_motion();
  sendBLEData();
  
}

/*------------------------------------------------------------------*/
/* Peripheral
 *------------------------------------------------------------------*/
void prph_connect_callback(uint16_t conn_handle)
{
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char peer_name[32] = { 0 };
  connection->getPeerName(peer_name, sizeof(peer_name));

  Serial.print("[Prph] Connected to ");
  Serial.println(peer_name);
}

void prph_disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  Serial.println();
  Serial.println("[Prph] Disconnected");
}

void prph_bleuart_rx_callback(uint16_t conn_handle)
{
  (void) conn_handle;
  uint8_t xx=1000;
  // Forward data from Mobile to our peripheral
  char str[20+1] = { 0 };
  bleuart.read(str, 20);

  Serial.print("[Prph] RX: ");
  Serial.println(str);  

  if ( clientUart.discovered() )
  {
    clientUart.print(str);
  }else
  {
    bleuart.println("[Prph]CentralRole NC");
    delay(xx);
    bleuart.println("ACC:1.0:0.0:-1.0:XX");
    delay(xx);
    bleuart.println("GYR:0.0:1.0:0.1:XX");
    delay(xx);
    bleuart.println("MAG:0.0:0.1:1.0:XX");
    
    //bleuart.println("012345678901234567890123456789");
  }
}

/*------------------------------------------------------------------*/
/* Central
 *------------------------------------------------------------------*/
void scan_callback(ble_gap_evt_adv_report_t* report)
{
  // Since we configure the scanner with filterUuid()
  // Scan callback only invoked for device with bleuart service advertised  
  // Connect to the device with bleuart service in advertising packet  
  Bluefruit.Central.connect(report);
}

void cent_connect_callback(uint16_t conn_handle)
{
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char peer_name[32] = { 0 };
  connection->getPeerName(peer_name, sizeof(peer_name));

  Serial.print("[Cent] Connected to ");
  Serial.println(peer_name);;

  if ( clientUart.discover(conn_handle) )
  {
    // Enable TXD's notify
    clientUart.enableTXD();
  }else
  {
    // disconnect since we couldn't find bleuart service
    Bluefruit.disconnect(conn_handle);
  }  
}

void cent_disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;
  
  Serial.println("[Cent] Disconnected");
}

/**
 * Callback invoked when uart received data
 * @param cent_uart Reference object to the service where the data 
 * arrived. In this example it is clientUart
 */
void cent_bleuart_rx_callback(BLEClientUart& cent_uart)
{
  char str[20+1] = { 0 };
  cent_uart.read(str, 20);
      
  Serial.print("[Cent] RX: ");
  Serial.println(str);

  if ( bleuart.notifyEnabled() )
  {
    // Forward data from our peripheral to Mobile
    bleuart.print( str );
  }else
  {
    // response with no prph message
    clientUart.println("[Cent] Peripheral role not connected");
  }  
}

void init_motion(void)
{

#if VIBRATOR
  // initialise vibrator
  drv.begin();
  drv.selectLibrary(1);
  // I2C trigger by sending 'go' command 
  // default, internal trigger when sending GO command
  drv.setMode(DRV2605_MODE_INTTRIG); 
#endif

#if PROXIMITY
  // initialize the sensors
  apds9960.begin();
  apds9960.enableProximity(true);
  apds9960.enableColor(true);
#endif

#if TEMP_BARO
  bmp280.begin();
#endif

#if MAGNETO
  lis3mdl.begin_I2C();
#endif

#if ACC_GYRO
  lsm6ds33.begin_I2C();
#endif

#if HUMIDITY
  sht30.begin();
#endif

#if MIC
  /*
  PDM.onReceive(onPDMdata);
  PDM.begin(1, 16000);
  */
#endif

}

void loop_motion(void)
{

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;

#if PROXIMITY
  proximity = apds9960.readProximity();
  while (!apds9960.colorDataReady()) {
    delay(5);
  }
  apds9960.getColorData(&r, &g, &b, &c);
#endif

#if TEMP_BARO
  temperature = bmp280.readTemperature();
  pressure = bmp280.readPressure();
  altitude = bmp280.readAltitude(1013.25);
#endif

#if MAGNETO
  lis3mdl.read();
  magnetic_x = lis3mdl.x;
  magnetic_y = lis3mdl.y;
  magnetic_z = lis3mdl.z;
#endif

#if ACC_GYRO
  lsm6ds33.getEvent(&accel, &gyro, &temp);
  accel_x = accel.acceleration.x;
  accel_y = accel.acceleration.y;
  accel_z = accel.acceleration.z;
  gyro_x = gyro.gyro.x;
  gyro_y = gyro.gyro.y;
  gyro_z = gyro.gyro.z;
#endif

#if HUMIDITY
  humidity = sht30.readHumidity();
#endif

#if MIC  //Do not enable
  //samplesRead = 0;
  //mic = getPDMwave(4000);
#endif

#if VIBRATOR
  onVibrator(effect);
  //delay(300);
  effect++;
  if (effect > 117) effect = 1;
#endif

}

void sendBLEData(void)
{
  static uint32_t ii=0, xx=100;
  char str[21];

    sprintf(str, "\n**Sensors Output**");
    bleuart.print(str);
    delay(xx);

#if PROXIMITY  
    sprintf(str, "\nPr:%d", apds9960.readProximity());
    bleuart.print(str);
    delay(xx);
#endif

#if TEMP_BARO
    sprintf(str, "\nT:%.2f", temperature);
    bleuart.print(str);
    delay(xx);

    sprintf(str, "\nP:%.2f", pressure);
    bleuart.print(str);
    delay(xx);

    sprintf(str, "\nAl:%.2f", altitude);  
    bleuart.print(str);
    delay(xx);
#endif

#if ACC_GYRO
    sprintf(str, "\nAc:%.2f:%.2f:%.2f", accel_x, accel_y, accel_z);  
    bleuart.print(str);
    delay(xx);

    sprintf(str, "\nGy:%.2f:%.2f:%.2f", gyro_x, gyro_y, gyro_z);  
    bleuart.print(str);
    delay(xx);
#endif

#if MAGNETO
    sprintf(str, "\nMxy:%.2f:%.2f", magnetic_x, magnetic_y);  
    bleuart.print(str);
    delay(xx);
    sprintf(str, "\nMz:%.2f", magnetic_z);  
    bleuart.print(str);    
    delay(xx);
#endif

#if HUMIDITY
    sprintf(str, "\nHm:%.2f%%", humidity);  
    bleuart.print(str);
    delay(xx);
#endif

  Serial.println("\nFeatherSenseDemo");
  Serial.println("------------------");
  
  Serial.print("Proximity: ");
  Serial.println(apds9960.readProximity());
  Serial.print("Red: ");
  Serial.print(r);
  Serial.print(" Green: ");
  Serial.print(g);
  Serial.print(" Blue :");
  Serial.print(b);
  Serial.print(" Clear: ");
  Serial.println(c);
  
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" C");
  Serial.print("Barometric pressure: ");
  Serial.println(pressure);
  Serial.print("Altitude: ");
  Serial.print(altitude);
  Serial.println(" m");
  Serial.print("Magnetic: ");
  Serial.print(magnetic_x);
  Serial.print(" ");
  Serial.print(magnetic_y);
  Serial.print(" ");
  Serial.print(magnetic_z);
  Serial.println(" uTesla");
  Serial.print("Acceleration: ");
  Serial.print(accel_x);
  Serial.print(" ");
  Serial.print(accel_y);
  Serial.print(" ");
  Serial.print(accel_z);
  Serial.println(" m/s^2");
  Serial.print("Gyro: ");
  Serial.print(gyro_x);
  Serial.print(" ");
  Serial.print(gyro_y);
  Serial.print(" ");
  Serial.print(gyro_z);
  Serial.println(" dps");
  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println(" %");
  /*
  Serial.print("Mic: ");
  Serial.println(mic);
  */
  /*
  Serial.print("Vibration effect type: ");
  Serial.println(effect);
  */
}



/*****************************************************************/
int32_t getPDMwave(int32_t samples) {
  short minwave = 30000;
  short maxwave = -30000;

  while (samples > 0) {
    if (!samplesRead) {
      yield();
      continue;
    }
    for (int i = 0; i < samplesRead; i++) {
      minwave = min(sampleBuffer[i], minwave);
      maxwave = max(sampleBuffer[i], maxwave);
      samples--;
    }
    // clear the read count
    samplesRead = 0;
  }
  return maxwave - minwave;
}

void onPDMdata() {
  // query the number of bytes available
  int bytesAvailable = PDM.available();

  // read into the sample buffer
  PDM.read(sampleBuffer, bytesAvailable);

  // 16-bit, 2 bytes per sample
  samplesRead = bytesAvailable / 2;
}

void onVibrator(uint8_t v_effect)
{
  // set the effect to play
  drv.setWaveform(0, v_effect);  // play effect 
  drv.setWaveform(1, 0);       // end waveform

  // play the effect!
  drv.go();  
}