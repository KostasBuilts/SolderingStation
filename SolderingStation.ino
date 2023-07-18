#include <LiquidCrystal_PCF8574.h>
#include <Wire.h>


#define TEMP_PROBE_PIN   A0
#define MAX_PWM         240
#define MIN_PWM         0
#define MAX_INTEGRAL    500
#define MIN_INTEGRAL    -500
#define SAMPLE_PERIOD   100
#define DISPLAY_PERIOD  1000
#define LCD_WIDTH  16
#define LCD_HEIGHT 2

LiquidCrystal_PCF8574 lcd(0x27); // set the LCD address to 0x27 for a 16 chars and 2 line display

///////////////////////////////////////////////////////
// The buffers where the serial connection is stored
//////////////////////////////////////////////////////
char serialBuffer[32];

/////////////////////////////////////
// Buffer to update the screen
////////////////////////////////////
char Line0[LCD_WIDTH];
char Line1[LCD_WIDTH];

bool SolderIron_state = false; //This bool is true if the soldering iron is on
bool openMenu = false; // this bool is true if the menu is open

double pwm = 10;
double currentTemp;
double setTemp = 160;
double pointV1, pointV2, pointT1, pointT2, slope = 116, beta = 11.84;

double Kp = 10;      //proportional gain
double Ki = 0.0015;  //integral gain
double Kd = 30;      //derivative gain

int T = SAMPLE_PERIOD;       //sample time in milliseconds (ms)

static unsigned long currentDisplay_time, lastDisplay_time; //Time since last lcd refresh variables

uint8_t ipwm;
///////////////////////////////////////////////////////
// this 2 dimensional array is the predefined menu
///////////////////////////////////////////////////////
char menu[][15]={"Kp = ",
            "Ki = ",
            "Kd = ",
            "Version 0.1.9",
            "               "};

byte bar0[8] = {
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
};

byte bar1[8] = {
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B11111,
};

byte bar2[8] = {
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B11111,
  B11111,
};

byte bar3[8] = {
  B00000,
  B00000,
  B00000,
  B00000,
  B11111,
  B11111,
  B11111,
};

byte bar4[8] = {
  B00000,
  B00000,
  B00000,
  B11111,
  B11111,
  B11111,
  B11111,
};

byte bar5[8] = {
  B00000,
  B00000,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
};

byte bar6[8] = {
  B00000,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
};

byte bar7[8] = {
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
};


void displayLcdTemp(double tempC, int x , int y)
{
    lcd.setCursor(x,y);
    lcd.print("      ");
    if(tempC<100)
        lcd.setCursor(x+1,y);
    else
        lcd.setCursor(x,y);

    lcd.print(tempC);
    
}

void MainLcdScreen(void)
{
    Serial.println(currentTemp);
    displayLcdTemp(currentTemp,1,0);
    lastDisplay_time = currentDisplay_time;
    if(ipwm>127){
        lcd.setCursor(0,1);
        lcd.write(byte(7));
        lcd.setCursor(0,0);
        lcd.write(byte((ipwm-127)>>4));
    }
    else {
        lcd.setCursor(0,0);
        lcd.write(byte(0));
        lcd.setCursor(0,1);
        lcd.write(byte(ipwm>>4));`
    }
}

void setup(void)
{
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  Serial.println("Starting Temp control");
  
  lcd.begin(16, 2); // initialize the lcd

  
  lcd.setBacklight(255);
  lcd.home();
  lcd.clear();
  lcd.print(" KK electronics ");
  lcd.setCursor(0,1);
  lcd.print(" Super  Station ");
  
  lcd.createChar(0, bar0);
  lcd.createChar(1, bar1);
  lcd.createChar(2, bar2);
  lcd.createChar(3, bar3);
  lcd.createChar(4, bar4);
  lcd.createChar(5, bar5);
  lcd.createChar(6, bar6);
  lcd.createChar(7, bar7);
  delay(2000);
  lcd.clear();
  displayLcdTemp(setTemp,1,1);
}




/*
 ****************************************************************************************************
        SERIAL COMMANDS DECODE ROUTINE

        O open iron
        X close iron
        t set T1 temp
        T set T2 remp
        A calculate slope and beta from point1 nad point2
        a show slope and beta
        S set temperature in celcius degrees
        U up the menu
        D down th menu
        M open menu
 ****************************************************************************************************
*/
void getCommand()
{
  //first bit is the command
  switch (serialBuffer[0])
  {
    case 'O':
    case 'o':
      Serial.print("Opening Soldering Iron ");
      SolderIron_state = true;
      break;

    case 'X':
    case 'x':
      Serial.print("Closing Soldering Iron ");
      SolderIron_state = false;
      break;

    case 't':
      pointV1 = (4.99 * analogRead(TEMP_PROBE_PIN)) / 1024;
      pointT1 = getNumber(1);
      Serial.print("Setting point T1=");
      Serial.print(pointT1);
      Serial.print("C and V1=");
      Serial.println(pointV1);
      break;

    case 'T':
      pointV2 = (4.99 * analogRead(TEMP_PROBE_PIN)) / 1024;
      pointT2 = getNumber(1);
      Serial.print("Setting point T2=");
      Serial.print(pointT2);
      Serial.print("C and V2=");
      Serial.println(pointV2);
      break;

    case 'A':
      slope = (pointT2 - pointT1) / (pointV2 - pointV1);
      beta = pointT1 - (slope * pointV1);
      Serial.print("Calculate Temperature slope=");
      Serial.print(slope);
      Serial.print(" and beta=");
      Serial.println(beta);
      break;

    case 'a':
      Serial.print("Temperature slope=");
      Serial.print(slope);
      Serial.print(" and beta=");
      Serial.println(beta);
      break;

    case 'S':
    case 's':
      setTemp = getNumber(1);
      displayLcdTemp(setTemp,1,1);
      Serial.print("Setting Temperature ");
      Serial.print(getNumber(1));
      Serial.println(" C");
      break;

      case 'U':
      case 'u':
        btnUp();

    default:
      Serial.println("Error in command\n");
      break;
  }
}

/*
 * *************************************
   Clear the inbuffer array from last
   command
 * *************************************
*/
void clearserialBuffer()
{
  for (int i = 0; i < 16; i++)
    serialBuffer[i] = 0;
}






/*
 * **************************************************
   This routine uses a a Serial object to get data
   from serial port.  *The object must had been
   initialized in setup routine.
   The routine uses a global array inbuffer to store
   the serial bytes received. Returns true when the
   last byte received is 0x0A.
 * **************************************************
*/
boolean getSerialData()
{
  static uint8_t i;

  while (Serial.available() > 0)
  {
    serialBuffer[i] = Serial.read();
    if (serialBuffer[i] == 10)
    {
      i = 0;
      return true;
    }
    ++i;
    if (i > sizeof(serialBuffer) - 1)
      i = sizeof(serialBuffer) - 1;
  }
  return false;
}



/*
 * *********************************************
   Convert the ASCII chars fron the inbuffer
   array to number.
   The routine starts from startPosition 2
   of inbuffer
 * *********************************************
*/
long getNumber(uint8_t startPosition)
{
  long result = 0;
  uint8_t i = startPosition;

  while (isNumber(serialBuffer[i]))
  {
    result = result * 10 + (serialBuffer[i] - 48);
    ++i;
  }

  return result;
}



/*
 * *********************************************
   Convert the ASCII chars fron the inbuffer
   array to a binaty number (HEX).
   The routine starts from startPosition 2
   of inbuffer
 * *********************************************
*/
long getBinaryNumber(uint8_t startPosition)
{
  long result = 0;
  uint8_t i = startPosition;

  while (isBinaryDigit(serialBuffer[i]))
  {
    result = result * 2 + (serialBuffer[i] - 48);
    ++i;
  }

  return result;
}





/*
 * ***************************************************
   Return true if ASCII c is number
 * ***************************************************
*/
boolean isNumber(char c)
{
  if (c >= '0' && c <= '9') return true;
  else return false;
}


/*
 * ***************************************************
   Return true if ASCII c is binary 0 or 1
 * ***************************************************
*/
boolean isBinaryDigit(char c)
{
  if (c == '0' || c == '1') return true;
  else return false;
}



/**
 * ********************************************
   SLOPE=78.571 B=25

   analogVolt=4.99*val / 1024
   Temp=78.571*analogVolt + 25
*/
double getTemp(int analogPin)
{
  double measureVolt;
  measureVolt = (4.99 * analogRead(analogPin)) / 1024;
  //Serial.println(val);
  return (slope * measureVolt) + beta;
}





//*******************************************************************************************************
void PID_Control() {
  //bool result=false;  
  static double total_error, last_error;
  static unsigned long last_time;
  unsigned long current_time = millis();

  //delta time interval
  int delta_time = current_time - last_time;

  if (delta_time >= T)
  {
    double error = setTemp - currentTemp;

    //************accumalates the error - integral term
    total_error += error;
    if (total_error >= MAX_INTEGRAL) total_error = MAX_INTEGRAL;
    else if (total_error <= MIN_INTEGRAL) total_error = MIN_INTEGRAL;

    //***************difference of error for derivative term
    double delta_error = error - last_error;

    //******************PID control compute
    pwm = Kp * error + (Ki * T) * total_error + (Kd / T) * delta_error;
    if (pwm >= MAX_PWM) pwm = MAX_PWM;
    else if (pwm <= MIN_PWM) pwm = MIN_PWM;

    last_error = error;
    last_time = current_time;
    //result=true;
  }
  //return result
}



//*************************************************************************************
void loop(void)
{
  currentTemp = getTemp(TEMP_PROBE_PIN);

  if (SolderIron_state) 
  {
    //calls the PID function every T interval and outputs a control signal
    PID_Control();
    ipwm = (uint8_t)pwm;
  }
  else 
  {
    ipwm=0;
  }
  analogWrite(9, ipwm);


  //Display temp evary one second
  currentDisplay_time = millis();
  if (currentDisplay_time - lastDisplay_time > DISPLAY_PERIOD && openMenu == false) 
  {
    MainLcdScreen();
  }
  else
  {
    showMenu();
  }

  //Scan Serial for commands
  if (getSerialData()) 
  {
    Serial.print(serialBuffer);
    getCommand();
    clearserialBuffer();
  }
}
