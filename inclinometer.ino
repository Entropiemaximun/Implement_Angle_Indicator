// idea  to manage  MPU6050 and  WS2812B  to indicate  position according LED color

//////////////////////////////source
//https://www.firediy.fr/article/mesurer-des-angles-avec-un-arduino-drone-ch-7
//
// NeoPixel Ring simple sketch (c) 2013 Shae Erisson
// Released under the GPLv3 license to match the rest of the
// Adafruit NeoPixel library


//////////////////////////////////////////////////////////


////////////////////////////entropiemaximun////////////////////

///17/07/2021   init prog according example 
///05/08/2021   field test (https://youtu.be/OA_bdoBcYg4)

///15/08/2021   initial deposit 
///15-17/08/2021   extra  function with  other button and  fix angle color according switch activation
//10/10/2021  1.6 add filter ti define the zero at the init  and  add choice of  led color  between  range angle 
//21/10/2021 2.0  clean the files
///////////////////
// arduino NANO

//+5V > +5V  MPU  
// A5 >  SCL  MPU
// A4 > SDA  MPU
// 
// VIN > W2812B +5V
// D6 > W2812B signal
//
// D7 > Switch ( potentiometer to shift down the level 12V > 5v
//
//
//
//



#define buttonPin  7  //   button   to set the  fix  angle  function 

#define ANGLE_REF   -38  // angle ref pour la fonction godet connu  sera  actif  avec le bouton
#define Bpixel  200 //  power  of leds
#define STAB_NOISE    0.5  // angle Sensitivity  to set the ref angleas init step
#define ANGLE_MPU    Y  // angle choice  X or Y /  not Z  for  MPU
#define sample 100   
//----------------------------------------------------------------------------------------------------------------------
#include <Wire.h>
//----------------------------------------------------------------------------------------------------------------------
#define YAW   0
#define PITCH 1
#define ROLL  2
#define PITCH2  3

#define X     0     // X axis
#define Y     1     // Y axis
#define Z     2     // Z axis

#define MPU_ADDRESS 0x68  // I2C address of the MPU-6050
#define FREQ        250// 250  Sampling frequency
#define SSF_GYRO    65.5  // Sensitivity Scale Factor of the gyro from datasheet


//-------------------------------------------------------

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

// Which pin on the Arduino is connected to the NeoPixels?
#define PIN        6 // On Trinket or Gemma, suggest changing this to 1

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS 8 // Popular NeoPixel ring size

// When setting up the NeoPixel library, we tell it how many pixels,
// and which pin to use to send signals. Note that for older NeoPixel
// strips you might need to change the third parameter -- see the
// strandtest example for more information on possible values.
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

#define DELAYVAL 200 // Time (in milliseconds) to pause between pixels

int npixel =0;
int chenillard =0;


#define VERT (0, 126, 58)
#define VERT1 (100, 209, 62)

#define BLEU (0, 38, 133)
#define BLEU1 (66, 154, 223)
#define BLEU2 (77, 199, 253)


#define JAUNE (118, 57, 49)
#define JAUNE1 (241, 171, 0)
#define JAUNE2 (250, 223, 0)



#define ROUGE (205, 30, 16)
#define ROUGE1 (252, 0, 127)
#define ROUGE2 (254, 121, 209)


#define VIOLET (76, 94, 119)
#define VIOLET1 (94, 83, 199)
#define VIOLET2 (126, 119, 210)


#define SANS (0, 0, 0)


#define ANGLE0 0
#define ANGLE1 7
#define ANGLE2 14
#define ANGLE3 21
#define ANGLE4 28
#define ANGLE5 35
#define ANGLE6 42
#define ANGLE7 49
#define ANGLE8 56
#define ANGLE9 63
#define ANGLE10 70
#define ANGLE11 77





//----------------------------------------------------------------------------------------------------------------------
// The RAW values got from gyro (in °/sec) in that order: X, Y, Z
int gyro_raw[3] = {0, 0, 0};

// Average gyro offsets of each axis in that order: X, Y, Z
long gyro_offset[3] = {0, 0, 0};

// Calculated angles from gyro's values in that order: X, Y, Z
float gyro_angle[3]  = {0, 0, 0};

// The RAW values got from accelerometer (in m/sec²) in that order: X, Y, Z
int acc_raw[3] = {0 , 0 , 0};

// Calculated angles from accelerometer's values in that order: X, Y, Z
float acc_angle[3] = {0, 0, 0};
float acc_angle2 =  0;
float calc_angle =  0;
float calc_anglev =  0;
int intensitypixel = 10;
// Total 3D acceleration vector in m/s²
long acc_total_vector;

/**
 * Real measures on 3 axis calculated from gyro AND accelerometer in that order : Yaw, Pitch, Roll
 *  - Left wing up implies a positive roll
 *  - Nose up implies a positive pitch
 *  - Nose right implies a positive yaw
 */
float measures[3] = {0, 0, 0};

// MPU's temperature
int temperature;

// Init flag set to TRUE after first loop
boolean initialized;
boolean initializedR; // permitto switch  init  to working mode
boolean initializedM; // Acces to mode 2

unsigned int  period; // Sampling period
unsigned long loop_timer;
int initcycle =0 ;
//----------------------------------------------------------------------------------------------------------------------
void setup() {
  Serial.begin(38400); //Only for debug


//Serial.print( "debug" ) ;
//
//   pinMode(buttonPin, INPUT_PULLUP); 
   pinMode(buttonPin, INPUT); 

  Wire.begin();
  TWBR = 12; // Set the I2C clock speed to 400kHz.

  setupMpu6050Registers();
  calibrateMpu6050();

  loop_timer = micros();
 // period = (1000000/ FREQ) ; // Sampling period in µs
 period = (1000) ; // Sampling period in µs



  readSensor();
  calculateAngles();

  pixels.begin();    
initializedR = false;  // consider  position  to be checked
initializedM = false;  // consider  position  to be checked
}

void loop() {



  pixels.setBrightness (Bpixel/intensitypixel*10);


if(npixel++ >= NUMPIXELS) npixel=0;
  
readSensor();
calculateAngles();

calc_anglev = (calc_anglev*(sample-1) + acc_angle[ANGLE_MPU]*1)/sample ;
  Serial.print(  calc_anglev ) ; 

if ( initializedM == true)


{  Serial.print( "!") ; 
 calc_angle = calc_anglev - ANGLE_REF*1 ;
}
else
{  Serial.print( "?") ;
 calc_angle =  calc_anglev -acc_angle2;
}
   Serial.println(  calc_angle ) ; 

 

if ( initializedR == false) {
 if  (initcycle ++  >  4 * sample ) initializedR = true;
  
if (chenillard++ > 255) chenillard = 0;
 if ( (chenillard/2)*2==chenillard){
  pixels.setPixelColor(chenillard/30, pixels.Color(chenillard, 0, 255- chenillard));
 }
  pixels.setPixelColor(npixel, pixels.Color SANS);
 
  acc_angle2 = (acc_angle2*(sample -1) + acc_angle[ANGLE_MPU]*1 )/sample;


}    else
{
  pixels.setPixelColor(npixel, pixels.Color SANS);
  if (calc_angle >0 ){
  if ((calc_angle -(npixel+1))< ANGLE10   ) pixels.setPixelColor(npixel, pixels.Color VERT1 );
  if ((calc_angle -(npixel+1))< ANGLE9   ) pixels.setPixelColor(npixel, pixels.Color SANS );
  if ((calc_angle -(npixel+1))< ANGLE8   ) pixels.setPixelColor(npixel, pixels.Color JAUNE2 );
  if ((calc_angle -(npixel+1))< ANGLE7   ) pixels.setPixelColor(npixel, pixels.Color SANS );
  if ((calc_angle -(npixel+1))< ANGLE6   ) pixels.setPixelColor(npixel, pixels.Color JAUNE1 );
  if ((calc_angle -(npixel+1))< ANGLE5   ) pixels.setPixelColor(npixel, pixels.Color SANS );
  if ((calc_angle -(npixel+1))< ANGLE4   ) pixels.setPixelColor(npixel, pixels.Color JAUNE );
  if ((calc_angle -(npixel+1))< ANGLE3   ) pixels.setPixelColor(npixel, pixels.Color SANS );
  if ((calc_angle -(npixel+1))< ANGLE2   ) pixels.setPixelColor(npixel, pixels.Color BLEU2 );
  if ((calc_angle -(npixel+1))< ANGLE1   ) pixels.setPixelColor(npixel, pixels.Color BLEU );
  if ((calc_angle -(npixel+1))< ANGLE0   ) pixels.setPixelColor(npixel, pixels.Color VERT );
  }else{
  if ((calc_angle +(NUMPIXELS -npixel))> -ANGLE10   ) pixels.setPixelColor(npixel, pixels.Color VERT1 );
  if ((calc_angle +(NUMPIXELS -npixel))> -ANGLE9   ) pixels.setPixelColor(npixel, pixels.Color SANS );
  if ((calc_angle +(NUMPIXELS -npixel))> -ANGLE8   ) pixels.setPixelColor(npixel, pixels.Color VIOLET2 );
  if ((calc_angle +(NUMPIXELS -npixel))> -ANGLE7   ) pixels.setPixelColor(npixel, pixels.Color SANS );
  if ((calc_angle +(NUMPIXELS -npixel))> -ANGLE6   ) pixels.setPixelColor(npixel, pixels.Color VIOLET1 );
  if ((calc_angle +(NUMPIXELS -npixel))> -ANGLE5   ) pixels.setPixelColor(npixel, pixels.Color SANS );
  if ((calc_angle +(NUMPIXELS -npixel))> -ANGLE4   ) pixels.setPixelColor(npixel, pixels.Color VIOLET );
  if ((calc_angle +(NUMPIXELS -npixel))> -ANGLE3   ) pixels.setPixelColor(npixel, pixels.Color SANS );
  if ((calc_angle +(NUMPIXELS -npixel))> -ANGLE2   ) pixels.setPixelColor(npixel, pixels.Color ROUGE2 );
  if ((calc_angle +(NUMPIXELS -npixel))> -ANGLE1   ) pixels.setPixelColor(npixel, pixels.Color ROUGE );
  if ((calc_angle +(NUMPIXELS -npixel))> -ANGLE0   ) pixels.setPixelColor(npixel, pixels.Color VERT );
  }

}


if (digitalRead(buttonPin) == HIGH){
  initializedM = true; 
  pixels.setPixelColor(npixel, pixels.Color(random(255), random(255), random(255)));
}

  pixels.show();   // Send the updated pixel colors to the hardware.

  while (micros() - loop_timer < period);
  loop_timer = micros();
}


//  original data  from MPU




/**
 * Configure gyro and accelerometer precision as following:
 *  - accelerometer: ±8g
 *  - gyro: ±500°/s
 *
 * @see https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
 * @return void
 */
void setupMpu6050Registers() {
  // Configure power management
  Wire.beginTransmission(MPU_ADDRESS); // Start communication with MPU
  Wire.write(0x6B);                    // Request the PWR_MGMT_1 register
  Wire.write(0x00);                    // Apply the desired configuration to the register
  Wire.endTransmission();              // End the transmission
  
  // Configure the gyro's sensitivity
  Wire.beginTransmission(MPU_ADDRESS); // Start communication with MPU
  Wire.write(0x1B);                    // Request the GYRO_CONFIG register
  Wire.write(0x08);                    // Apply the desired configuration to the register : ±500°/s
  Wire.endTransmission();              // End the transmission
  
  // Configure the acceleromter's sensitivity
  Wire.beginTransmission(MPU_ADDRESS); // Start communication with MPU
  Wire.write(0x1C);                    // Request the ACCEL_CONFIG register
  Wire.write(0x10);                    // Apply the desired configuration to the register : ±8g
  Wire.endTransmission();              // End the transmission
  
  Wire.beginTransmission(MPU_ADDRESS); // Start communication with MPU
  Wire.write(0x1A);                    // Request the CONFIG register
  Wire.write(0x03);                    // Set Digital Low Pass Filter about ~43Hz
  Wire.endTransmission();              // End the transmission
}

/**
 * Calibrate MPU6050: take 2000 samples to calculate average offsets.
 * During this step, the quadcopter needs to be static and on a horizontal surface.
 *
 * This function also sends low throttle signal to each ESC to init and prevent them beeping annoyingly.
 *
 * This function might take ~2sec for 2000 samples.
 *
 * @return void
 */
void calibrateMpu6050()
{
  
//Serial.println( "calibration" ) ;
  int max_samples = 100;

  for (int i = 0; i < max_samples; i++) {
    readSensor();

//Serial.println( i) ;
    gyro_offset[X] += gyro_raw[X];
    gyro_offset[Y] += gyro_raw[Y];
    gyro_offset[Z] += gyro_raw[Z];

    // Just wait a bit before next loop
    delay(3);
  }

  // Calculate average offsets
  gyro_offset[X] /= max_samples;
  gyro_offset[Y] /= max_samples;
  gyro_offset[Z] /= max_samples;
}

/**
 * Request raw values from MPU6050.
 *
 * @return void
 */
void readSensor() {
  Wire.beginTransmission(MPU_ADDRESS); // Start communicating with the MPU-6050
  Wire.write(0x3B);                    // Send the requested starting register
  Wire.endTransmission();              // End the transmission
  Wire.requestFrom(MPU_ADDRESS, 14);   // Request 14 bytes from the MPU-6050

  // Wait until all the bytes are received
  while (Wire.available() < 14);

  acc_raw[X]  = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the acc_raw[X] variable
  acc_raw[Y]  = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the acc_raw[Y] variable
  acc_raw[Z]  = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the acc_raw[Z] variable
  temperature = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the temperature variable
  gyro_raw[X] = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the gyro_raw[X] variable
  gyro_raw[Y] = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the gyro_raw[Y] variable
  gyro_raw[Z] = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the gyro_raw[Z] variable
}

/**
 * Calculate real angles from gyro and accelerometer's values
 */
void calculateAngles()
{
  calculateGyroAngles();
  calculateAccelerometerAngles();

  if (initialized) {
    // Correct the drift of the gyro with the accelerometer
    gyro_angle[X] = gyro_angle[X] * 0.9996 + acc_angle[X] * 0.0004;
    gyro_angle[Y] = gyro_angle[Y] * 0.9996 + acc_angle[Y] * 0.0004;
  } else {
    // At very first start, init gyro angles with accelerometer angles
    gyro_angle[X] = acc_angle[X];
    gyro_angle[Y] = acc_angle[Y];

    initialized = true;
  }

  // To dampen the pitch and roll angles a complementary filter is used
  measures[ROLL]  = measures[ROLL]  * 0.9 + gyro_angle[X] * 0.1 ;//+ acc_angle[X] * 0.0004;;
  measures[PITCH] = measures[PITCH] * 0.9 + gyro_angle[Y] * 0.1 ;//+ acc_angle[Y] * 0.0004;
  measures[YAW]   = -gyro_raw[Z] / SSF_GYRO; // Store the angular motion for this axis
}

/**
 * Calculate pitch & roll angles using only the gyro.
 */
void calculateGyroAngles()
{
  // Subtract offsets
  gyro_raw[X] -= gyro_offset[X];
  gyro_raw[Y] -= gyro_offset[Y];
  gyro_raw[Z] -= gyro_offset[Z];

  // Angle calculation using integration
  gyro_angle[X] += (gyro_raw[X] / (FREQ * SSF_GYRO));
  gyro_angle[Y] += (-gyro_raw[Y] / (FREQ * SSF_GYRO)); // Change sign to match the accelerometer's one

  // Transfer roll to pitch if IMU has yawed
  gyro_angle[Y] += gyro_angle[X] * sin(gyro_raw[Z] * (PI / (FREQ * SSF_GYRO * 180)));
  gyro_angle[X] -= gyro_angle[Y] * sin(gyro_raw[Z] * (PI / (FREQ * SSF_GYRO * 180)));
}

/**
 * Calculate pitch & roll angles using only the accelerometer.
 */
void calculateAccelerometerAngles()
{
  // Calculate total 3D acceleration vector : √(X² + Y² + Z²)
  acc_total_vector = sqrt(pow(acc_raw[X], 2) + pow(acc_raw[Y], 2) + pow(acc_raw[Z], 2));

  // To prevent asin to produce a NaN, make sure the input value is within [-1;+1]
  if (abs(acc_raw[X]) < acc_total_vector) {
    acc_angle[X] = asin((float)acc_raw[Y] / acc_total_vector) * (180 / PI); // asin gives angle in radian. Convert to degree multiplying by 180/pi
  }

  if (abs(acc_raw[Y]) < acc_total_vector) {
    acc_angle[Y] = asin((float)acc_raw[X] / acc_total_vector) * (180 / PI);
  }

}
