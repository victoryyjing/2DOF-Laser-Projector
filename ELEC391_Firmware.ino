#define ISR_ON_TIMER 12
#define ENCODER_OPTIMIZE_INTERRUPTS

// Encoder
//#include <Encoder.h>
#include <Wire.h>

byte buffer[4];
int16_t position1;
int16_t position2;

// Motor 1 (LASER) Pins
//Encoder lmotor(2, 3);
//#define enA 13
#define in1 9
#define in2 10

// Motor 2 (BASE) Pins
//Encoder bmotor(18, 19);
#define in3 8
#define in4 7
//#define enB 4

// const int counterPins[] = {53, 51, 49, 47, 45, 43, 41, 39, 37, 35};
// #define DIR_INPUT 33

// Tuning Constants
#define dt 0.001
#define FDDdeg 24 // IMPORTANT
// const int numBits = sizeof(counterPins) / sizeof(counterPins[0]);

#define DEFAULTPWM 180

#define lMapRange 25000
#define bMapRange 25000

// PID global variables
double Kmaster_b, Kp_b, Ki_b, Kd_b;
double Kmaster_l, Kp_l, Ki_l, Kd_l;
volatile double bpos_adj, lpos_adj;
volatile double bpos_diff_old = 0, lpos_diff_old = 0;

// Motor global variables
volatile int bmotorpos = 0, newposb = 0;
volatile int lmotorpos = 0, newposl = 0;
volatile int btargetpos, ltargetpos;
volatile bool motordir;
int laserindex = 0, baseindex = 0;

/* ---------- SQUARE ---------- */
// double lasertargets[] =   {-25, -25, 25, 25};
// double basetargets[] =    {-25, 25, 25, -25};

/* ---------- STAR ---------- */
double lasertargets[] =   {-20, 40, -25, 20, 10};
double basetargets[] =    {-20, -5, 10, -35, 25};

/* ---------- OCTAGON ---------- */
// double lasertargets[] =   {0, 5, 15, 25, 35, 40, 35, 25, 15, 5};
// double basetargets[] =    {0, -10, -20, -20, -10, 0, 10, 20, 20, 10};


// ISR variables
float ISRrate = 2*0.001; // IMPORTANT

// Filter variables
double FilterCoeff[FDDdeg] = { 0.1621, 0.1362, 0.1145, 0.0962, 0.0809, 0.0680, 0.0571, 0.0480,
                              0.0403, 0.0339, 0.0285, 0.0239, 0.0201, 0.0169, 0.0142, 0.0119,
                              0.0100, 0.0084, 0.0071, 0.0060, 0.0050, 0.0042, 0.0035, 0.0030};

volatile double bFDDarray[FDDdeg] = {0};
volatile double lFDDarray[FDDdeg] = {0};


void setup() {
  // Set pin modes
  pinMode(ISR_ON_TIMER, OUTPUT);
  // Motor pin modes
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  /* vvvvvvvvvvvv PID TUNING PARAMETERS vvvvvvvvvvvv */
  // Laser tuning parameters
  Kmaster_l = 190;
  Kp_l = 1;
  Ki_l = 1;
  Kd_l = 0.01;
 
  // Base tuning parameters
  Kmaster_b = 190;
  Kp_b = 1;
  Ki_b = 1;
  Kd_b = 0.01;
  /* ^^^^^^^^^^^^ PID TUNING PARAMETERS ^^^^^^^^^^^^ */

  // Decoder
  Wire.begin(8);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event
  Serial.begin(115200);           // start serial for output

  // Timer stuff I definitely understand
  cli();
  TCCR1A = 0;           TCCR1B = 0;
  TCCR1B |= B00000100;  TIMSK1 |= B00000010;
  OCR1A = ISRrate/0.000016;
  sei();

  // Default PWM state
  analogWrite(in1, DEFAULTPWM);
  analogWrite(in2, DEFAULTPWM);
  analogWrite(in3, DEFAULTPWM);
  analogWrite(in4, DEFAULTPWM);
}


void loop() {
  /* ------------------------------- */
  if(abs(lmotorpos - ltargetpos) <= 2 && abs(bmotorpos - btargetpos) <= 2){
    laserindex = (laserindex + 1) % (sizeof(lasertargets) / sizeof(lasertargets[0]));
    baseindex = (baseindex + 1) % (sizeof(basetargets) / sizeof(basetargets[0]));
  }
  ltargetpos = lasertargets[laserindex];
  btargetpos = basetargets[baseindex];
  /* ------------------------------- */

  if(lpos_adj < 0) { // If adjust request is negative, we are above our goal
    analogWrite(in1, map(abs(lpos_adj), 0, lMapRange, DEFAULTPWM, 255));
    //analogWrite(in1, 254);
    analogWrite(in2, 0);
  } else if(lpos_adj > 0) { // If request is positive, we are below our goal
    analogWrite(in1, 0);
    analogWrite(in2, map(abs(lpos_adj), 0, lMapRange, DEFAULTPWM, 255));
    //analogWrite(in3, 255);
  } else {
    analogWrite(in1, 0);
	  analogWrite(in2, 0);
  }



  if(bpos_adj < 0){ // If adjust request is negative, we are to the right of our goal
    analogWrite(in3, map(abs(bpos_adj), 0, bMapRange, DEFAULTPWM, 255));
    analogWrite(in4, 0);
  } else if(bpos_adj > 0){
    analogWrite(in3, 0);
    analogWrite(in4, map(abs(bpos_adj), 0, bMapRange, DEFAULTPWM, 255));
  } else{
    analogWrite(in3, 0);
	  analogWrite(in4, 0);
  }
}


ISR(TIMER1_COMPA_vect){
  digitalWrite(ISR_ON_TIMER, HIGH); // Interrupt Start

  TCNT1 = 0; // First, set the timer back to 0 so it resets for next interrupt 
  double bpos_diff = btargetpos - bmotorpos;
  double lpos_diff = ltargetpos - lmotorpos;
    
  bpos_adj = bPIDCtrl(bpos_diff);
  lpos_adj = lPIDCtrl(lpos_diff);

  digitalWrite(ISR_ON_TIMER, LOW); // Interrupt End
}

double bPIDCtrl(double pos_diff){
  double integral = 0;

  /* ---------- PID START ---------- */
  // Compute proportional term
  double proportional = pos_diff;

  // Compute integral term
  integral += pos_diff * dt;

  // Compute derivative term
  double dxval = (pos_diff - bpos_diff_old) / dt;
  double derivative = bFDDWSFilter(dxval);

  // Store old position difference (NOT WORKING)
  bpos_diff_old = pos_diff;

  // Return tuned and adjusted position
  double pos_adj = Kmaster_b*((Kp_b * proportional) + (Ki_b * integral) + (Kd_b * derivative));
  /* ----------- PID END ----------- */

  return pos_adj;
}

double lPIDCtrl(double pos_diff){
  double integral = 0;

  /* ---------- PID START ---------- */
  // Compute proportional term
  double proportional = pos_diff;

  // Compute integral term
  integral += pos_diff * dt;

  // Compute derivative term
  double dxval = (pos_diff - lpos_diff_old) / dt;
  double derivative = lFDDWSFilter(dxval);

  // Store old position difference (NOT WORKING)
  lpos_diff_old = pos_diff;

  // Return tuned and adjusted position
  double pos_adj = Kmaster_l*((Kp_l * proportional) + (Ki_l * integral) + (Kd_l * derivative));
  /* ----------- PID END ----------- */

  return pos_adj;
}


double bFDDWSFilter(double dxterm){
  double filtered = 0.0;

  memmove(bFDDarray + 1, bFDDarray, (FDDdeg - 1) * sizeof(double));
  bFDDarray[0] = dxterm;

  for (int j = 0; j < FDDdeg; j++) {
    filtered += bFDDarray[j] * FilterCoeff[j];
  }

  if(isinf(filtered) || isnan(filtered)) return 0;
  
  return filtered;
}

double lFDDWSFilter(double dxterm){
  double filtered = 0.0;

  memmove(lFDDarray + 1, lFDDarray, (FDDdeg - 1) * sizeof(double));
  lFDDarray[0] = dxterm;

  for (int j = 0; j < FDDdeg; j++) {
    filtered += lFDDarray[j] * FilterCoeff[j];
  }

  if(isinf(filtered) || isnan(filtered)) return 0;
  
  return filtered;
}

void receiveEvent(int howMany)
{
  while(1 < Wire.available()) // loop through all but the last
  {
    buffer[0] = Wire.read(); // High byte of motor 1 position
    buffer[1] = Wire.read(); // Low byte of motor 1 position
    buffer[2] = Wire.read(); // High byte of motor 2 position
    buffer[3] = Wire.read(); // Low byte of motor 2 position
  }

  newposl = (buffer[0] << 8) | buffer[1];    // receive byte as an integer
  newposb = (buffer[2] << 8) | buffer[3]; 
  
  if (newposb != bmotorpos || newposl != lmotorpos) {
    bmotorpos = newposb;
    lmotorpos = newposl;
  }
}
