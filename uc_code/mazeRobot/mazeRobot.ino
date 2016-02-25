/* Main source code for the maze robot. */

#define USE_USB_SERIAL 1 // change this to 0 to enable hardware serial
#if USE_USB_SERIAL
  #define SerPort SerialUSB
#else
  #define SerPort Serial1 // Note: change this to the correct serial port.
#endif

#define USE_VERBOSE_DEBUG 1

/* Parameters that can be adjusted to tweek the behavior. */
#define MAX_SPEED_CHANGE 50
#define SPEED_CHANGE_RAMP_DELAY 50 /* ms */
#define MOTOR_SCALE_FACTOR 2000

#define DXL_BUS_SERIAL1 1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04

/* Note: The Left wheel + direction is considered forward */
#define ID_NUM_LEFT 1
#define ID_NUM_RIGHT 2

#define BOARD_RIGHT_DRIVE 16
#define BOARD_RIGHT_SENSE 17

/* TODO: these need to be define to the correct pins */
#define BOARD_LEFT_DRIVE 18
#define BOARD_LEFT_SENSE 19
// #define BOARD_LINE_DRIVE 20
#define BOARD_LINE_SENSE 21

/* buffers for the incomming serial data */
static char commandBuffer[256];
static int commandBufferIndex = 0;

Dynamixel Dxl(DXL_BUS_SERIAL1);

/* Note: The speed values are in the range [-1023, 1023] with positive
 * values being forward in the robot body frame. The specifics of handling
 * the direction bit and making sure both motors turn in the correct direction
 * are handled internaly.
 */
int leftSpeed = 0;
int leftGoalSpeed = 0;
int rightSpeed = 0;
int rightGoalSpeed = 0;
int maxSpeedChange = MAX_SPEED_CHANGE;

long nextRightUpdate = 0;
long nextLeftUpdate = 0;

long motorTimeout = -1;

/* Trip Sensors (buttons, bumpers, line sensors) */
#define USER_BUTTON   0x01
#define LEFT_BUMPER   0x02
#define RIGHT_BUMPER  0x04
#define LINE_SENSOR   0x08
volatile int tripSensorStatus = 0;
volatile long userButtonLockout = -1;
volatile long leftBumperLockout = -1;
volatile long rightBumperLockout = -1;
volatile long lineSensorLockout = -1;

#define USER_BUTTON_LOCKOUT_TIME 200 /* ms */
#define BUMPER_LOCKOUT_TIME 200 /* ms */
#define LINE_SENSOR_LOCKOUT_TIME 200 /* ms */

void setup()
{
#if USE_USB_SERIAL == 0
  SerPort.begin(57000);
#endif
  Dxl.begin(3);
  Dxl.wheelMode(ID_NUM_LEFT); //wheelMode() is to use wheel mode
  Dxl.wheelMode(ID_NUM_RIGHT); //wheelMode() is to use wheel mode
  pinMode(BOARD_RIGHT_DRIVE, OUTPUT);
  digitalWrite(BOARD_RIGHT_DRIVE, 1);
  pinMode(BOARD_RIGHT_SENSE, INPUT_PULLDOWN);
  
  /* attach the interrupts */
//  attachInterrupt(BOARD_BUTTON_PIN, userButtonHandler, RISING);
//  attachInterrupt(BOARD_RIGHT_SENSE, leftBumperHandler, RISING);
//  attachInterrupt(BOARD_LEFT_SENSE, rightBumperHandler, RISING);
//  attachInterrupt(BOARD_LINE_SENSE, lineSensorHandler, RISING);
  
  pinMode(BOARD_LED_PIN, OUTPUT);
}

void loop()
{
  /* TODO: this currently is free running, it should be configured to run at 
   * a fixed rate
   */
#if USE_USB_SERIAL
  if(SerPort.isConnected())
#endif
  {
    while(SerPort.available())
    {
      commandParser(SerPort.read());
    }
  }
  
//  handleTripSensors();
  
  setMotorSpeed();
  handleMotorTimeout(); /* TODO: actualy write this function */
}

/*
 * Sensor handling.
 */
void handleTripSensors(void)
{
  if(tripSensorStatus & USER_BUTTON)
  {
    tripSensorStatus &= ~USER_BUTTON;
    SerPort.println("<sensor user:1>");
  }
  if((userButtonLockout > 0) && (millis() > userButtonLockout))
  {
    userButtonLockout = -1;
    attachInterrupt(BOARD_BUTTON_PIN, userButtonHandler, RISING);
  }
    
  if(tripSensorStatus & LEFT_BUMPER)
  {
    tripSensorStatus &= ~LEFT_BUMPER;
    SerPort.println("<sensor left:1>");
  }
  if((leftBumperLockout > 0) && (millis() > leftBumperLockout))
  {
    leftBumperLockout = -1;
    attachInterrupt(BOARD_LEFT_SENSE, leftBumperHandler, RISING);
  }
  
  if(tripSensorStatus & RIGHT_BUMPER)
  {
    tripSensorStatus &= ~RIGHT_BUMPER;
    SerPort.println("<sensor right:1>");
  }
  if((rightBumperLockout > 0) && (millis() > rightBumperLockout))
  {
    rightBumperLockout = -1;
    attachInterrupt(BOARD_RIGHT_SENSE, rightBumperHandler, RISING);
  }
  
  if(tripSensorStatus & LINE_SENSOR)
  {
    tripSensorStatus &= ~LINE_SENSOR;
    SerPort.println("<sensor line:1>");
  }
  if((lineSensorLockout > 0) && (millis() > lineSensorLockout))
  {
    lineSensorLockout = -1;
    attachInterrupt(BOARD_LINE_SENSE, lineSensorHandler, RISING);
  }
}

void userButtonHandler(void)
{
  tripSensorStatus |= USER_BUTTON;
  long userButtonLockout = USER_BUTTON_LOCKOUT_TIME;
  detachInterrupt(BOARD_BUTTON_PIN);
}

void leftBumperHandler(void)
{
  tripSensorStatus |= LEFT_BUMPER;
  long leftBumperLockout = BUMPER_LOCKOUT_TIME;
  detachInterrupt(BOARD_LEFT_SENSE);
}

void rightBumperHandler(void)
{
  tripSensorStatus |= RIGHT_BUMPER;
  long rightBumperLockout = BUMPER_LOCKOUT_TIME;
  detachInterrupt(BOARD_RIGHT_SENSE);
}

void lineSensorHandler(void)
{
  tripSensorStatus |= LINE_SENSOR;
  long lineSensorLockout = LINE_SENSOR_LOCKOUT_TIME;
  detachInterrupt(BOARD_LINE_SENSE);
}

/*
 * Motor Speed Updating.
 */
void setMotorSpeed(void)
{
  /* Note the timeings are still arbitraty, I havent done the math yet. */ 
  if(nextLeftUpdate == 0 || nextLeftUpdate < millis())
  {
    if(abs(leftGoalSpeed - leftSpeed) < maxSpeedChange)
    {
      leftSpeed = leftGoalSpeed;
      nextLeftUpdate = 0x7FFFFFFF;
    }
    else
    {
      nextLeftUpdate = millis() + SPEED_CHANGE_RAMP_DELAY;
      if((leftGoalSpeed - leftSpeed) > 0)
      {
        leftSpeed += maxSpeedChange;
      }
      else
      {
        leftSpeed -= maxSpeedChange;
      }
    }
    /* Set the motor speed based on the current values */
#if USE_VERBOSE_DEBUG
    SerPort.print("Setting Left = ");
    SerPort.println(leftSpeed);
#endif
    if(leftSpeed < 0)
    {
      Dxl.goalSpeed(ID_NUM_LEFT, (-leftSpeed) | 0x400);
    }
    else
    {
      Dxl.goalSpeed(ID_NUM_LEFT, leftSpeed);
    }
  }
  
  if(nextRightUpdate == 0 || nextRightUpdate < millis())
  {
    if(abs(rightGoalSpeed - rightSpeed) < maxSpeedChange)
    {
      rightSpeed = rightGoalSpeed;
      nextRightUpdate = 0x7FFFFFFF;
    }
    else
    {
      nextRightUpdate = millis() + SPEED_CHANGE_RAMP_DELAY;
      if((rightGoalSpeed - rightSpeed) > 0)
      {
        rightSpeed += maxSpeedChange;
      }
      else
      {
        rightSpeed -= maxSpeedChange;
      }
    }
    
    /* Set the motor speed based on the current values */
#if USE_VERBOSE_DEBUG
    SerPort.print("Setting Right = ");
    SerPort.println(rightSpeed);
#endif
    if(rightSpeed < 0)
    {
      Dxl.goalSpeed(ID_NUM_RIGHT, -rightSpeed);
    }
    else
    {
      Dxl.goalSpeed(ID_NUM_RIGHT, rightSpeed | 0x400);
    }
  }
}

void setLeftGoalSpeed(int s)
{
  s = constrain(s, -1023, 1023);
  leftGoalSpeed = s;
  nextLeftUpdate = 0;
}

void setRightGoalSpeed(int s)
{
  s = constrain(s, -1023, 1023);
  rightGoalSpeed = s;
  nextRightUpdate = 0;
}

void handleMotorTimeout(void)
{
  if((motorTimeout >= 0) && (motorTimeout < millis()))
  {
    setLeftGoalSpeed(0);
    setRightGoalSpeed(0);
  }
}
  

/*
 *  Functions for parsing incoming commands.
 */
void commandParser(char c)
{
  static int parserReading = false;

  if(c == '<')
  {
    parserReading = true;
    commandBufferIndex = 0;
  }
  else if(parserReading)
  {
    if(c == '>')
    {
      commandBuffer[commandBufferIndex] = '\0';
      parseCommand(commandBuffer);
      parserReading = false;
    }
    else
    {
      commandBuffer[commandBufferIndex++] = c;
    }
  }
   
}

/* currently only the following commands are parsed
 * <arc r:[radius value] w:[angular velocity]>
 * r is in mm and w is in radians per ms (guessed) refrence values are r=1000, w=200
 * <rotate a:[angle to rotate through] t:[time]>
 * a is in degrees and t is in ms. (rotates through a degrees in t ms)
 * NOTE: the values are relative and don't corrispond to and real values
 */
void parseCommand(char * cmd)
{
#if USE_VERBOSE_DEBUG
  SerPort.println(cmd);
#endif
  const char * arcCmd = "arc";
  const char * rotateCmd = "rotate";
  
  if(cmd[0] == 'a') /* Parser 'arc' Command */
  {
    if(cmd[1] == 'r' && cmd[2] == 'c')
    {
      /* Execute arc command */
      float r = 1;
      float w = 0;
      
      /* Note: the behavior for a malformed command is undefined (but almost certainly bad)
       */
      int i = 2;
      while(cmd[i++] != ' '); /* find the next space */
      if(cmd[i] == 'r')
      {
        while(cmd[i++] != ':'); /* find the ':' */
        r = atol(&cmd[i])/1000.0;
      }
      
      while(cmd[i++] != ' '); /* find the next space */
      if(cmd[i] == 'w')
      {
        while(cmd[i++] != ':'); /* find the ':' */
        w = atol(&cmd[i])/1000.0;
      }
      
#if USE_VERBOSE_DEBUG
      SerPort.print("Got r:");
      SerPort.print(r);
      SerPort.print(" w:");
      SerPort.println(w);
#endif
      
      setLeftGoalSpeed(round((w*(r-0.0465))*MOTOR_SCALE_FACTOR));
      setRightGoalSpeed(round((w*(r+0.0465))*MOTOR_SCALE_FACTOR));
      
#if USE_VERBOSE_DEBUG  
      SerPort.println("exe arc!");
#endif
    }
  }
  else if(cmd[0] == 'r') /* Parse 'rotate' Command */
  {
    int i = 1;
    while(rotateCmd[i] != '\0')
    {
      if(rotateCmd[i] != cmd[i])
      {
        break;
      }
      ++i;
    }
    if(rotateCmd[i] == '\0')
    {
      int a = 0;
      long t = 0;
      /* Execute Rotate Command */
      while(cmd[++i] == ' '); /* find the first non-space character */
      if(cmd[i] == 'a')
      {
        while(cmd[i++] != ':'); /* find the ':' */
        a = atol(&cmd[i]);
      }
      while(cmd[i++] != ' '); /* find the next space */
      while(cmd[++i] == ' '); /* find the first non-space character */
      if(cmd[i] == 't')
      {
        while(cmd[i++] != ':'); /* find the ':' */
        t = atol(&cmd[i]);
      }
      float arclength_mm = PI*93.0*a/360.0;
      float v_mm_per_ms = arclength_mm/t;
      
      setLeftGoalSpeed(-round(v_mm_per_ms*MOTOR_SCALE_FACTOR));
      setRightGoalSpeed(round(v_mm_per_ms*MOTOR_SCALE_FACTOR));
      motorTimeout = millis() + t;
      
      
#if USE_VERBOSE_DEBUG
      SerPort.println("Exe rotate!");
#endif
    }
  }
  else if(cmd[0] == 'm') /* Parser direct motor set command */
  {
    int i = 0;
    while(cmd[i++] != ' ');
    int l = atol(&cmd[i]);
    while(cmd[i++] != ' ');
    int r = atol(&cmd[i]);

    
    setLeftGoalSpeed(l);
    setRightGoalSpeed(r);
    
#if USE_VERBOSE_DEBUG
    SerPort.print("Got l:");
    SerPort.print(l);
    SerPort.print(" r:");
    SerPort.println(r);
#endif
  }
}

      
    

