#include <SoftwareSerial.h>
#include <Servo.h>

//RC CONTROL

#define mode 8
#define motor_switch 13
#define servo_switch 12
unsigned int motor_in;
unsigned int servo_in;

// For camera

uint8_t clockPin = 3;
uint8_t syncPin = 4;
uint8_t dataPin = A0; // must be in range 0 to 5
unsigned int lightVal[128];
unsigned int expose = 1000;   //int expose = 7390; // 8333uSec ie 1/120th second
unsigned long uptime, downtime;

// For Motor

#define pmos 11
#define nmos 7
#define max_speed 50 //20 
#define turn_speed 90 //60
uint8_t motor_speed = 40;

// For servo

#define servo_left 1300  //2000
#define servo_center 1550
#define servo_right 1800 //1000
float _servo[128];
double k = 0;

Servo myservo;  // create servo object to control a servo
#define Filter_Const 0.25 //.20 = 20//0 to 1
#define kp      1.12// 1.12 = 20//1.2 = 40 //1.5 = 50//kp=1.25 for speed = 60 80 110
#define kd      0.065//0.08 = 20 //0.11 = 40   //0.07 = 50 //0.10 for 60 80
#define ki      0.00   //0.01 for 80
float p = 0, p_last = 0;
float _period = 0, _error = 0, _dterror = 0, _integral = 0, _lasterror = 0, _pid = 0, Filter_Out = 0, Filter_Out_Prev = 0;

//For increased rate of loop

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

void setup() {

  myservo.attach(9);  // attaches the servo on pin 0 to the servo object
  pinMode(pmos, OUTPUT);
  pinMode(nmos, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(syncPin, OUTPUT);
  pinMode(dataPin, INPUT);
  pinMode(mode, INPUT);
  pinMode(motor_in, INPUT);
  pinMode(servo_in, INPUT);

  for (int i = 0; i < 128; i++)
  {
    _servo[i] = servo_left + (i * 3.90625); //500
  }
  Serial.begin(115200);
  cbi(ADCSRA, ADPS2) ; //sbi
  sbi(ADCSRA, ADPS1) ; //cbi
  cbi(ADCSRA, ADPS0) ;

}

void getCamera()
{

  uptime = micros();
  digitalWrite(clockPin, LOW);
  digitalWrite(syncPin, HIGH);
  digitalWrite(clockPin, HIGH);
  digitalWrite(syncPin, LOW);
  digitalWrite(clockPin, LOW);
  
  delayMicroseconds(expose);
  k = 0;

  for (int j = 0; j < 128; j++)
  {
    lightVal[j] = analogRead(dataPin);
    k = k + lightVal[j];
    digitalWrite(clockPin, HIGH);
    digitalWrite(clockPin, LOW);
    k = k / 128;
    k = k*1.15;//k * 1.25; //k * 1.07; //1.25 ;//+ 5;//1.3+13
  }  
}

void Servo_out()
{
  for (int i = 0; i < 128; i++)
  {
    if (lightVal[i] >= k && lightVal[i + 1] >= k && lightVal[i + 2] >= k && lightVal[i + 3] >= k)
      for (int j = 128; j > 0; j--)
      {
        if (lightVal[j] >= k && lightVal[j - 1] >= k && lightVal[j - 2] >= k && lightVal[j - 3] >= k )
        {
          if ((j - i) < 10)
          {
            _error = servo_center -  _servo[(i+j)/2];
            Filter_Out_Prev = Filter_Out; // save previous value
            Filter_Out = Filter_Const*_error + (1-Filter_Const)* Filter_Out; // take a portion of the new input and a portion of the old value to average out over several samples
            // simulate a first order exponential filter.
            _dterror = (Filter_Out - Filter_Out_Prev) / _period; // derivative output based on filtered value
            _integral = _integral + _error*_period;
            _pid= _error*kp + _integral*ki +_dterror*kd;
            _lasterror = _error;
            p = servo_center - _pid;
          }

          else
          {
            p = p_last;
          }

          if (p > servo_right )
          {
            p = servo_right;
          }
          if (p < servo_left)
          {
            p = servo_left;
          }
          if (abs(p-p_last) <= 150)
          {
            motor_speed = turn_speed;
          }
          else if (abs(p-p_last) <=100)
          {
            motor_speed = (uint8_t)((turn_speed + max_speed)/2);
          }
          else
          {
            motor_speed = max_speed;
          }
          p_last = p;
          myservo.writeMicroseconds(p);
          break;
        }
      }
      break;
    }
  }
}

void loop()
{
  if (digitalRead(mode) == 1)
  {
    getCamera( );
    Servo_out();
    analogWrite(pmos, motor_speed);
    digitalWrite(nmos, LOW);
    downtime = micros();
    _period = downtime - uptime;
  }
  else
  {
    while (digitalRead(mode) == 0  )
    {
     
      digitalWrite(pmos, HIGH);
      digitalWrite(nmos, HIGH);
      servo_in = pulseIn(servo_switch, HIGH);
      if (servo_in > 1500 && servo_in < 1950)
      {
        myservo.writeMicroseconds( map(servo_in, 1500, 1850, servo_left, servo_right));
      }
    }
  }
}

