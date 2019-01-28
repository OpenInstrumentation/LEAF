// This #include statement was automatically added by the Particle IDE.
//#include <Adafruit_BME280.h>




/* 
 Yggdrasil Code
 
 Particle Electron Pin Outs for Sensors
 
 Sap Flux DIFF   - WKP    A7 (WKP)

 SD Card        - MOSI   A5 <-- Todo
 SD Card        - MISO   A4
 SD Card        - SCK    A3
 SD Card        - SS(CS) A2
 Temp           - D-In   A1
 Pressure       - A-In   B0
 Soil-1         - A-In   B5
 Soil-2         - A-In   B4
 
 Ultra Sonic    - Echo   D7 <-- Relay for Batt kick (Test)
 Ultra Sonic    - Trigg  D6
 DHT22          - D-In   D5
 Rain Bucket    - A-In   D4 <-- Interrupt  
 Water Detector - A-In   D3 <-- Interrupt
 Accel.         - SCL    D1 <--|
 Accel.         - SDA    D0    |---Share SCL and SDA I2C
 BME280         - SCL    D1    |
 BME280         - SDA    D0 <--|
 Turbidity      - D-In   C4 <-- Todo
 Turbidity      - S3     C3 
 Turbidity      - S2     C2
 Turbidity      - S1     C1
 Turbidity      - S0     C0
 */
 
// Included Libraries
// Additional Math Functions
#include <math.h>
// DHT22 for Temp, Humidity and Pressure
#include <Adafruit_DHT_Particle.h>
// SDCard Reader/Writer
#include <SdFat.h>
// Ultra Sonic HC-SR04
#include <HC-SR04.h>
// Accelerometer (I2C-0x6A) 9DOF 
#include <SparkFunLSM9DS1.h>
// Temp /Relative humidity and Pressure (I2C- 0x77 or 0x76?) BME280
#include <SparkFunBME280.h>
// Water Digital Temp DS18B20
#include <DS18B20.h>

//------------------------------------------------------------------------------
// Sap Flux Hot Definition
#define Sap_Flux_Hot A7


//------------------------------------------------------------------------------
// Sap Flux Cold Definition
#define Sap_Flux_Cold A6


//------------------------------------------------------------------------------
// SDCard Definition
#define SPI_CONFIGURATION 0
// Setup SPI configuration.
// Primary SPI with DMA
// SCK => A3, MISO => A4, MOSI => A5, SS => A2 (default)
SdFat sd;
File myFile;
const uint8_t chipSelect = SS; // Default SD chip select is SS pin
// store error strings in flash to save RAM
#define error(s) sd.errorHalt(F(s))

//------------------------------------------------------------------------------
// Digital Temp DS18B20
#define DS18B20PIN A1
DS18B20  ds18b20(DS18B20PIN, true);


//------------------------------------------------------------------------------
// Pressure Definition
#define WATER_PRESSURE A0


//------------------------------------------------------------------------------
// Soil Moisture 1 Definition
#define Soil_Moisture_1 B5


//------------------------------------------------------------------------------
// Soil Moisture 2 Definition
#define Soil_Moisture_2 B4


//------------------------------------------------------------------------------
// Depth Sensor HC-SR04 Pin Definition
//const int echoPin = D7;
//const int triggerPin = D6;
//HC_SR04 waterlevel = HC_SR04(triggerPin, echoPin);


//Battery Relay
#define batt_relay D7


//------------------------------------------------------------------------------
//DHT22 Definition
#define DHTPIN D5     // Ref above pins
#define DHTTYPE DHT22		// DHT 22 (AM2302)
// Connect pin 1 (on the left) of the sensor to +5V
// Connect pin 2 of the sensor D5
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 5 (data) to pin 1 (power) of the sensor
DHT dht(DHTPIN, DHTTYPE);


//------------------------------------------------------------------------------
// RAIN DRIP BUCKET DEFINITION
#define DRIP_SENSOR_PIN D4

//------------------------------------------------------------------------------
// INTERNAL WATER SENSOR DEFINITION
#define WATER_SENSOR_PIN D3


//------------------------------------------------------------------------------
//Accelerometer 9DOF I2C Definition
LSM9DS1 imu;

// SDO_XM and SDO_G are both pulled high, so our addresses are:
#define LSM9DS1_M	0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG	0x6B // Would be 0x6A if SDO_AG is LOW

////////////////////////////
// Sketch Output Settings //
////////////////////////////
#define PRINT_CALCULATED
//#define PRINT_RAW
#define PRINT_SPEED 250 // 250 ms between prints

// Earth's magnetic field varies by location. Add or subtract 
// a declination to get a more accurate heading. Calculate 
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION 3.44 // Declination (degrees) in Austin, TX.


//------------------------------------------------------------------------------
// T/RH/P BME280 I2C Definition
BME280 t_rh_p_bme;
//Adafruit_BME280 t_rh_p_bme;
//------------------------------------------------------------------------------
// TURBIDITY SENSOR DEFINITION
#define TURBIDITY_INPUT_PIN C4
#define TURBIDITY_S3_PIN C3
#define TURBIDITY_S2_PIN C2
#define TURBIDITY_S1_PIN C1
#define TURBIDITY_S0_PIN C0

#define SEALEVELPRESSURE_HPA (1013.25)

//-----------------------------------------------------------------------------
//  GLOBAL VARIABLE DECLARATIONS FOR PARTICLE CALL VARIABLE USAGE
double sf_h, sf_c, h_dht, t_dht, t_bme, rel_humid_bme, p_bme, ax, ay, az;
int accel_test = 1;
double slam=0.;
String myID = System.deviceID();
bool dht_status = true;

//-----------------------------------------------------------------------------
//  GLOBAL INTERRUPT VARIABLES AND HANDLER FOR RAINBUCKET

 volatile bool bucket_state = false;
 const unsigned int DEBOUNCE_TIME = 750;
 static unsigned long last_interrupt_time = 0;
 
 int rain = -1;


//long int loopCount = 0; //If a set number of loops are to happen before microcontroller sleep
unsigned long lastTime = 0;

const String key = "CZJ4L4MWV7ZE3HDS";

//----------------------------------------------------//
//            BEGIN SETUP                             //
//----------------------------------------------------//

void setup() {
  Serial.begin(9600);
  Particle.publish("ImOn", "Entering Setup");
  
  
    // PRESENT VARIABLES FOR TESTING
    /*Particle.variable("Sap Flux Ht", sf_h);
    Particle.variable("DHT Temp C", t_dht);
    Particle.variable("BME Temp", t_bme);
    Particle.variable("Lp Run Count", loopCount);
    */
    //Serial.println(myID);
    //Particle.variable("Device ID", myID);
    
    Particle.function("KickBatt", kickbatt);
    
    
    


// Initialize sd card  
  // Initialize SdFat or print a detailed error message and halt
  // Use half speed like the native library.
  Particle.publish("SDCard_Setup", "Initializing");
  if (!sd.begin(chipSelect, SPI_HALF_SPEED)) {
    sd.initErrorHalt();
     Particle.publish("SDCard_Setup", "SD Card init failed");
    }


  // open the file for write at end like the "Native SD library"
  if (!myFile.open("startup.txt", O_RDWR | O_CREAT | O_AT_END)) {
    sd.errorHalt("opening startup.log for write failed");
    Particle.publish("SDCard_Setup", "SD Card startup.txt init failed");
  }

  // if the file opened okay, write to it:
 Time.zone(-6);
 time_t time = Time.now();
 String timeStamp = Time.format(time, TIME_FORMAT_ISO8601_FULL);
  
  
  
  
  myFile.println("////////////////SD Card initialized///////////////////");
  myFile.printf(timeStamp,"\n");
  myFile.printf("fileSize: %d\n", myFile.fileSize());
  myFile.printf("Device ID ",myID.c_str(),"\n");
  myFile.printf("[--------------------------------------------------]\n");
  
  // close the file:
  myFile.close();
  //Serial.println("done.");


/*
  // re-open the file for reading:
  if (!myFile.open("test.txt", O_READ)) {
    sd.errorHalt("opening test.txt for read failed");
  }
  Serial.println("test.txt content:");

  // read from the file until there's nothing else in it:
  int data;
  while ((data = myFile.read()) >= 0) {
    Serial.write(data);
  }
  // close the file:
  myFile.close();

*/


//------------------------------------------------------------------------------
//Initialize HC-SR04 Waterlevel
//waterlevel.init();
delay (1000);

//------------------------------------------------------------------------------
//Initialize DHT22

Particle.publish("DHT_Setup", "DHT Initializing");
dht.begin();
delay(1000);
    Particle.publish("DHT_Test", String(dht.getTempCelcius()));




/*
//------------------------------------------------------------------------------
//Initialize Accelerometer 9DOF I2C
imu.settings.device.commInterface = IMU_MODE_I2C;
imu.settings.device.mAddress = LSM9DS1_M;
imu.settings.device.agAddress = LSM9DS1_AG;
  // The above lines will only take effect AFTER calling
  // imu.begin(), which verifies communication with the IMU
  // and turns it on.
  if (!imu.begin()) {
      Particle.publish("IMU_Setup", "Accelerometer Failed");
      accel_test = 0;
   }

*/

//------------------------------------------------------------------------------
//Initialize BME280
//I2C Setup
 Particle.publish("BME_Setup", "Initializing");

//t_rh_p_bme.settings.commInterface = I2C_MODE;
//t_rh_p_bme.settings.I2CAddress = 0x76; //Could be 0x76


	t_rh_p_bme.settings.commInterface = I2C_MODE;
	t_rh_p_bme.settings.I2CAddress = 0x76;

	//For SPI enable the following and dissable the I2C section
	//t_rh_p_bme.settings.commInterface = SPI_MODE;
	//t_rh_p_bme.settings.chipSelectPin = 10;


	//***Operation settings*****************************//

	//runMode can be:
	//  0, Sleep mode
	//  1 or 2, Forced mode
	//  3, Normal mode
	t_rh_p_bme.settings.runMode = 3; //Normal mode

	//tStandby can be:
	//  0, 0.5ms
	//  1, 62.5ms
	//  2, 125ms
	//  3, 250ms
	//  4, 500ms
	//  5, 1000ms
	//  6, 10ms
	//  7, 20ms
	t_rh_p_bme.settings.tStandby = 0;

	//filter can be off or number of FIR coefficients to use:
	//  0, filter off
	//  1, coefficients = 2
	//  2, coefficients = 4
	//  3, coefficients = 8
	//  4, coefficients = 16
	t_rh_p_bme.settings.filter = 4;

	//tempOverSample can be:
	//  0, skipped
	//  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
	t_rh_p_bme.settings.tempOverSample = 5;

	//pressOverSample can be:
	//  0, skipped
	//  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
    t_rh_p_bme.settings.pressOverSample = 5;

	//humidOverSample can be:
	//  0, skipped
	//  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
	t_rh_p_bme.settings.humidOverSample = 5;

	
	Serial.print("Program Started\n");
	Serial.print("Starting BME280... result of .begin(): 0x");

	//Calling .begin() causes the settings to be loaded
	delay(10);  //Make sure sensor had enough time to turn on. BME280 requires 2ms to start up.
	//Particle.publish("BME_Setup", "BME Failed");
	if (!t_rh_p_bme.begin()){
    Particle.publish("BME_Setup", "BME Failed");
}
	
	//Serial.println(t_rh_p_bme.begin(), HEX);

	Serial.print("Displaying ID, reset and ctrl regs\n");

	Serial.print("ID(0xD0): 0x");
	Serial.println(t_rh_p_bme.readRegister(BME280_CHIP_ID_REG), HEX);
	Serial.print("Reset register(0xE0): 0x");
	Serial.println(t_rh_p_bme.readRegister(BME280_RST_REG), HEX);
	Serial.print("ctrl_meas(0xF4): 0x");
	Serial.println(t_rh_p_bme.readRegister(BME280_CTRL_MEAS_REG), HEX);
	Serial.print("ctrl_hum(0xF2): 0x");
	Serial.println(t_rh_p_bme.readRegister(BME280_CTRL_HUMIDITY_REG), HEX);




//***Operation settings*****************************//
/*
	//runMode can be:
	//  0, Sleep mode
	//  1 or 2, Forced mode
	//  3, Normal mode
	t_rh_p_bme.settings.runMode = 3; //Normal mode

	//tStandby can be:
	//  0, 0.5ms
	//  1, 62.5ms
	//  2, 125ms
	//  3, 250ms
	//  4, 500ms
	//  5, 1000ms
	//  6, 10ms
	//  7, 20ms
	t_rh_p_bme.settings.tStandby = 0;

	//filter can be off or number of FIR coefficients to use:
	//  0, filter off
	//  1, coefficients = 2
	//  2, coefficients = 4
	//  3, coefficients = 8
	//  4, coefficients = 16
	t_rh_p_bme.settings.filter = 4;

	//tempOverSample can be:
	//  0, skipped
	//  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
	t_rh_p_bme.settings.tempOverSample = 5;

	//pressOverSample can be:
	//  0, skipped
	//  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
    t_rh_p_bme.settings.pressOverSample = 5;

	//humidOverSample can be:
	//  0, skipped
	//  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
	t_rh_p_bme.settings.humidOverSample = 5;


if (!t_rh_p_bme.begin()){
    Particle.publish("BME_Setup", "BME Failed");
}
*/
delay(1000);
//Particle.variable("BME280 Addr", t_rh_p_bme.readRegister(BME280_CHIP_ID_REG));
Particle.variable("BME280 Alt", String(bme_altitude()));

//---------------------------------------------------------------------------
//-------- PIN DEFINITIONS --------------------------------------------------
//---------------------------------------------------------------------------

pinMode(Sap_Flux_Hot,INPUT); //Sap Flux Hot
pinMode(Sap_Flux_Cold,INPUT); //Sap Flux Cold
//DS18B20 Temp prode account for no resister
pinMode(DS18B20PIN,INPUT_PULLDOWN);
pinMode(WATER_PRESSURE,INPUT); //Water Pressure
pinMode(Soil_Moisture_1,INPUT); //Soil Moisture 1
pinMode(Soil_Moisture_2,INPUT); //Soil Moisture 2
//pinMode(DHTPIN,INPUT); //DHT
//Initialize Rain Bucket Intterupt 
  pinMode(DRIP_SENSOR_PIN, INPUT_PULLDOWN); /* Interrupt pin */
  attachInterrupt(DRIP_SENSOR_PIN, bucket_handler, RISING);
  Particle.variable("BucketCount", rain);


pinMode(WATER_SENSOR_PIN,INPUT); //Internal Water Senser
pinMode(TURBIDITY_INPUT_PIN,INPUT); //Turbidity Input
pinMode(TURBIDITY_S3_PIN,INPUT); //Turbidity S3
pinMode(TURBIDITY_S2_PIN,OUTPUT); //Turbidity S2
pinMode(TURBIDITY_S1_PIN,OUTPUT); //Turbidity S1
pinMode(TURBIDITY_S0_PIN,OUTPUT); //Turbidity S0
pinMode(batt_relay, OUTPUT);
digitalWrite(batt_relay, LOW);


Particle.publish("ImSetup", "Entering Loop");
Particle.function("DHTReset", dht_status_reset);
 delay (1000);

} //<---------------END SETUP----------------------<<-------------------------<<

// Sensor functions
float sapflux_hot(){
    return analogRead(Sap_Flux_Hot);
}

float sapflux_cold(){
    return analogRead(Sap_Flux_Cold);
}

float powermeter(){
    FuelGauge batt;
    float level = batt.getVCell();
    return level;
}

float r_temp(){
    float _temp, celsius;
    int   i = 0;
    int MAXRETRY = 4;
    do {
        _temp = ds18b20.getTemperature();
    } while (!ds18b20.crcCheck() && MAXRETRY > i++);

    if (i < MAXRETRY) {
    celsius = _temp;
    }
  else {
    celsius = 0;
  }
    return celsius;
}

float r_pressure(){
    return analogRead(WATER_PRESSURE);
}

float r_soilmoisture_1(){
    return analogRead(Soil_Moisture_1);
}

float r_soilmoisture_2(){
    return analogRead(Soil_Moisture_2);
}
/*
float r_water_level(){
    float levelcm = waterlevel.distCM();
    return waterlevel.distCM();
}
*/
float dht_humidity(){
   float h = dht.getHumidity();
    if (dht_status == true){
        if (isnan(h)) {
            Particle.publish("DHT","Humidity Failed DHT sensor!");
            dht_status = false;
            return 0;
          }
        else {
       return h;
       }
    }   
    else {
        return 0;
    }
}    
    
float dht_temp(){
     float t = dht.getTempCelcius();
    if (dht_status == true){
        if (isnan(t)) {
            Particle.publish("DHT","Temperature Failed DHT sensor!");
            dht_status = false;
            return 0;
          }
        else {
       return t;
       }
    }   
    else {
        return 0;
    }
}

int dht_status_reset(String input){
    if (input == "true" || "on" || "reset"){
    dht_status = true;
    Particle.publish("DHT","DHT status set to true");
    return 0;
    }
    else {
         Particle.publish("DHT","Use reset as input");
         return 1;
    }
    
           
} 

float r_turbidity();
//int accel_vals[9];
int accel(float &ref_ax, float &ref_ay, float &ref_az){
    // To read from the accelerometer, you must first call the
    // readAccel() function. When this exits, it'll update the
    // ax, ay, and az variables with the most current data.
    imu.readAccel();
  
    // Now we can use the ax, ay, and az variables as we please.
    // Either print them as raw ADC values, or calculated in g's.
    #ifdef PRINT_CALCULATED
        // If you want to print calculated values, you can use the
        // calcAccel helper function to convert a raw ADC value to
        // g's. Give the function the value that you want to convert.
        ref_ax = imu.calcAccel(imu.ax);
        ref_ay = imu.calcAccel(imu.ay);
        ref_az = imu.calcAccel(imu.az);
    #elif defined PRINT_RAW 
        ref_ax = imu.ax;
        ref_ay = imu.ay;
        ref_az = imu.az;
    #endif
}
/*
char accel_working() {
    char status
    imu.readAccel();
    if (){ status = B;}
    else {status = G;}
    return status;
}
*/
float bme_temp(){
    t_rh_p_bme.readTempC();
    return t_rh_p_bme.readTempC();
    //return t_rh_p_bme.readTemperature();
}
float bme_rh(){
    return t_rh_p_bme.readFloatHumidity();
    //return t_rh_p_bme.readHumidity();
}
float bme_p(){
    return t_rh_p_bme.readFloatPressure();
    //return t_rh_p_bme.readPressure();
}
double bme_altitude(){
    return t_rh_p_bme.readFloatAltitudeFeet();
    //return t_rh_p_bme.readAltitude(SEALEVELPRESSURE_HPA);
}

// This function turns on then back off the relay that connects the usb to the external battery pack 
int kickbatt(String command){
    if (command == "On" || "on" || "ON"){
    digitalWrite(batt_relay, HIGH);   
    delay (5000);
    digitalWrite(batt_relay, LOW);
    return 1;
    }
}

//////////////////////////////////////////////////////
///     VOID LOOP START                           ///
/////////////////////////////////////////////////////
void loop() {


String data, sd_data, ts_data;
Time.zone(-6);
time_t time = Time.now();
String timeStamp = Time.format(time, TIME_FORMAT_ISO8601_FULL); 
Serial.println(timeStamp);    
    
// Increment and reset from interrupt
if (bucket_state == true){
    rain++;
   // Particle.publish("Bucket Trip (.1')", String(rain));
    String rain_data_file_name = myID + "-rainbucket.log";

    if (!myFile.open(rain_data_file_name, O_RDWR | O_CREAT | O_AT_END)) {
        sd.errorHalt("opening rain data file for write failed");
        Serial.println("SD Card file deviceID-xxx open failed");
    }
  
  myFile.println(timeStamp + " .1");
  
  myFile.close();
    
    bucket_state = false;
    //interrupts();
}	
unsigned long now = millis();
//int log_time = 1; //number between when you want to write data to the sdcard in minutes
int publish_time = 10; //number between when you want to publish data events in minutes
	//if ((now - lastTime) >= (log_time * 60000) || now <= 15000){
    //Particle.publish("Data", data);
    //Particle.publish("thingSpeakWrite_All", ts_data, 60, PRIVATE);	
/*Sensor Measurement order:
    1. Sap Flux Hot
    2. Sap Flux Cold
    3. Power (volt) Meter
    4. Temp Sensor (Roots)
    5. Pressure Sensor (Roots)
    6. Soil Moisture 1 (Roots)
    7. Soil Moisture 2 (Roots)
    8. Water Level (Roots)
    9. DHT22 (Temp/RHumidity)
    10. Accelerometer (!Roots)
    11. Turbidity (Roots)
    12. Unit Battery
    
    
    NOTE: Rain Bucket is an Interrupt 
    
*/

// Varible to set True for roots location or False for branch location
//bool roots = false;



//Uncomment below for yggdrasil-tester to be included as a root box
//myID = "30004e000151363131363432";

if (myID == "30004e000151363131363432" || myID == "25004e000251363131363432") { //Cypress Root or Oak Root
    sf_h = sapflux_hot();
    //sf_c = sapflux_cold();
    float battpower = powermeter();
    float w_temp = r_temp();
    //float w_temp = 0;
    float w_pressure = 0;//r_pressure();
    float soilmoisture_1 = map(r_soilmoisture_1(), 4095., 0., 0., 100.);
    float soilmoisture_2 = map(r_soilmoisture_2(), 4095., 0., 0., 100.);
    //float rh_bme = bme_rh();
    float t_bme = bme_temp();
    //float p_bme = bme_p();
    //float w_level = r_water_level();
    h_dht = dht_humidity();
    t_dht = dht_temp();
    if (h_dht == NULL) {
        h_dht = -1;
        t_dht = -1;
    }
    
    data = String::format("{\"Sap_Flux_Hot(V)\": %4.2f, \"Water_Temp(V)\": %4.2f, \"Water_Pressure(V)\": %4.2f, \"Soil_Moistures_1(\%)\": %4.2f, \"Soil_Moisture_2(\%)\": %4.2f, \"Hum(\%)\": %4.2f, \"Temp_DHT(°C)\": %4.2f, \"Temp_BME(°C)\": %4.2f}",sf_h, w_temp, w_pressure, soilmoisture_1, soilmoisture_2, h_dht, t_dht, t_bme);
    sd_data = String::format("{\"Sap_Flux_Hot(V)\": %4.2f, \"Water_Temp(V)\": %4.2f, \"Water_Pressure(V)\": %4.2f, \"Soil_Moistures_1(\%)\": %4.2f, \"Soil_Moisture_2(\%)\": %4.2f, \"Hum(\%)\": %4.2f, \"Temp_DHT(°C)\": %4.2f, \"Temp_BME(°C)\": %4.2f}",sf_h, w_temp, w_pressure, soilmoisture_1, soilmoisture_2, h_dht, t_dht, t_bme);
    ts_data = "{ \"1\": \"" + String(sf_h) + "\"," +
       "\"2\": \"" + String(w_temp) + "\"," +
       "\"3\": \"" + String(w_pressure) + "\"," +
       "\"4\": \"" + String(soilmoisture_1) + "\"," +
       "\"5\": \"" + String(soilmoisture_2) + "\"," +
       "\"6\": \"" + String(h_dht) + "\"," +
       "\"7\": \"" + String(t_dht) + "\"," +
       "\"8\": \"" + String(t_bme) + "\"," +
       "\"k\": \"" + key + "\" }";
    }
else {
    float sf_h = sapflux_hot();
    float battpower = powermeter();
   // float h_dht = dht_humidity();
    //float t_dht = dht_temp();
    if (h_dht == NULL) {
        h_dht = -1;
        t_dht = -1;
    }
    float rh_bme = bme_rh();
    float t_bme = bme_temp();
    float p_bme = bme_p();
    slam = 0;
    for (int i=0;i<100;i++) {  
        if (imu.accelAvailable()) {
            imu.readAccel();
            float ax0 = ax;
            float ay0 = ay;
            float az0 = az;
            ax = imu.ax;
            ay = imu.ay;
            az = imu.az;
            float dax = ax-ax0; 
            float day = ay-ay0;
            float daz = az-az0;
            float dot_product = dax*dax + day*day + daz*daz;  // squared amplitude of jerk
            
            // slam: summed leaf activity metric = sum of dot product 
            slam += dot_product; // integrate squared amplitude of jerk
        }
    }
    float sum_axg = 0;
    float sum_ayg = 0;
    float sum_azg = 0;
    for (int i=0;i<100;i++){
        float get_axg = 0;
        float get_ayg = 0;
        float get_azg = 0;
        accel(get_axg, get_ayg, get_azg);
        sum_axg += get_axg;
        sum_ayg += get_ayg;
        sum_azg += get_azg;
    }
    
    sd_data =  String::format("%s, %4.2f ,%4.2f ,%4.2f ,%4.2f ,%4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f",timeStamp.c_str(), sf_h, battpower, h_dht, t_dht, rh_bme, t_bme, p_bme, slam, sum_axg/100, sum_ayg/100, sum_azg/100);
    data = String::format("{\"Sap_Flux_Diff(V)\": %4.2f, \"Batt_Power(V)\": %4.2f, \"DHT_Hum(\%)\": %4.2f, \"DHT_Temp(°C)\": %4.2f, \"R-Humidity\": %4.2f,\"BME Temp (°C)\": %4.2f, \"Air Pressure (~kPa)\": %4.2f, \"Accel Metric\": %4.2f}",sf_h, battpower, h_dht, t_dht, rh_bme, t_bme, p_bme, slam);
    }
//--------------Un Comment to start Particle Publish ---------------------
//if (loopCount % 120 == 0){
//unsigned long now = millis();
//int publish_time = 10; //number between when you want to publish data events in minutes
	if ((now - lastTime) >= (publish_time * 60000) || now <= 15000){
    Particle.publish("Data", data);
    Particle.publish("thingSpeakWrite_All", ts_data, 60, PRIVATE);
	lastTime = now;
}



Serial.println(sd_data);

String data_file_name = myID + "-";

if (!myFile.open(data_file_name, O_RDWR | O_CREAT | O_AT_END)) {
    sd.errorHalt("opening data file for write failed");
    Serial.println("SD Card file deviceID-xxx open failed");
  }
  
  myFile.println(sd_data);
  
  myFile.close();

// open the file for write at end like the "Native SD library"
/*if (!myFile.open("data.log", O_RDWR | O_CREAT | O_AT_END)) {
    sd.errorHalt("opening data.log for write failed");
    }
myFile.print(timeStamp);
myFile.print(",");
myFile.println(data);
myFile.close();
*/


/*
//DHT22    

    loopCount++;
// Reading temperature or humidity takes about 250 milliseconds!
// Sensor readings may also be up to 2 seconds 'old' (its a 
// very slow sensor)
	float h = dht.getHumidity();
// Read temperature as Celsius
	float t = dht.getTempCelcius();
// Read temperature as Farenheit
	float f = dht.getTempFarenheit();
  
// Check if any reads failed and exit early (to try again).
	if (isnan(h) || isnan(t) || isnan(f)) {
		Serial.println("Failed to read from DHT sensor!");
		return;
	}

// Compute heat index
// Must send in temp in Fahrenheit!
	float hi = dht.getHeatIndex();
	float dp = dht.getDewPoint();
	float k = dht.getTempKelvin();

	Serial.print("Humid: "); 
	Serial.print(h);
	Serial.print("% - ");
	Serial.print("Temp: "); 
	Serial.print(t);
	Serial.print("*C ");
	Serial.print(f);
	Serial.print("*F ");
	Serial.print(k);
	Serial.print("*K - ");
	Serial.print("DewP: ");
	Serial.print(dp);
	Serial.print("*C - ");
	Serial.print("HeatI: ");
	Serial.print(hi);
	Serial.println("*C");
	Serial.println(Time.timeStr());
	if (loopCount%5 == 0){
	    // Get some data
        String data = String::format("{\"Hum(\%)\": %4.2f, \"Temp(°C)\": %4.2f, \"DP(°C)\": %4.2f, \"HI(°C)\": %4.2f}", h, t, dp, hi);
        // Trigger the integration
        Particle.publish("DHT22", data, PRIVATE);
         // Wait 60 seconds
        delay(60000);
	    Particle.publish("readings", String::format("{\"Hum(\%)\": %4.2f, \"Temp(°C)\": %4.2f, \"DP(°C)\": %4.2f, \"HI(°C)\": %4.2f}", h, t, dp, hi));
	    
	}
	//String timeStamp = Time.timeStr();
//	Particle.publish("readings", String::format("{\"Hum(\%)\": %4.2f, \"Temp(°C)\": %4.2f, \"DP(°C)\": %4.2f, \"HI(°C)\": %4.2f}", h, t, dp, hi));
//	delay(10000);
//	loopCount++;
//	if(loopCount >= 6){
//	  Particle.publish("state", "Going to sleep for 5 minutes");
//	  delay(1000);
//	  System.sleep(SLEEP_MODE_DEEP, 300);  
//	}
*/
delay(30000);






//loopCount++;
} // <------- END LOOP



// Interrupt handler for drip bucket

void bucket_handler() {
    unsigned long interrupt_time = millis();
  if (interrupt_time - last_interrupt_time > DEBOUNCE_TIME) {
   //noInterrupts();
  bucket_state = true;
  }
  last_interrupt_time = interrupt_time; 
}
