/* Main source code for the maze robot. */

/* Parameters that can be adjusted to tweek the behavior. */
#define MAX_SPEED_CHANGE 50
#define SPEED_CHANGE_RAMP_DELAY 50 /* ms */

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
  Dxl.begin(3);
  Dxl.wheelMode(ID_NUM_LEFT); //wheelMode() is to use wheel mode
  Dxl.wheelMode(ID_NUM_RIGHT); //wheelMode() is to use wheel mode
  pinMode(BOARD_RIGHT_DRIVE, OUTPUT);
  digitalWrite(BOARD_RIGHT_DRIVE, 1);
  pinMode(BOARD_RIGHT_SENSE, INPUT_PULLDOWN);
  
  /* attach the interrupts */
  attachInterrupt(BOARD_BUTTON_PIN, userButtonHandler, RISING);
  attachInterrupt(BOARD_RIGHT_SENSE, leftBumperHandler, RISING);
  attachInterrupt(BOARD_LEFT_SENSE, rightBumperHandler, RISING);
  attachInterrupt(BOARD_LINE_SENSE, lineSensorHandler, RISING);
  
  pinMode(BOARD_LED_PIN, OUTPUT);
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

void loop()
{
  /* TODO: this currently is free running, it should be configured to run at 
   * a fixed rate
   */
  if(SerialUSB.isConnected())
  {
    while(SerialUSB.available())
    {
      commandParser(SerialUSB.read());
    }
  }
  
  handleTripSensors();
  
  setMotorSpeed();
}

void handleTripSensors(void)
{
  if(tripSensorStatus & USER_BUTTON)
  {
    tripSensorStatus &= ~USER_BUTTON;
    SerialUSB.println("<sensor user:1>");
  }
  if((userButtonLockout > 0) && (millis() > userButtonLockout))
  {
    userButtonLockout = -1;
    attachInterrupt(BOARD_BUTTON_PIN, userButtonHandler, RISING);
  }
    
  if(tripSensorStatus & LEFT_BUMPER)
  {
    tripSensorStatus &= ~LEFT_BUMPER;
    SerialUSB.println("<sensor left:1>");
  }
  if((leftBumperLockout > 0) && (millis() > leftBumperLockout))
  {
    leftBumperLockout = -1;
    attachInterrupt(BOARD_LEFT_SENSE, leftBumperHandler, RISING);
  }
  
  if(tripSensorStatus & RIGHT_BUMPER)
  {
    tripSensorStatus &= ~RIGHT_BUMPER;
    SerialUSB.println("<sensor right:1>");
  }
  if((rightBumperLockout > 0) && (millis() > rightBumperLockout))
  {
    rightBumperLockout = -1;
    attachInterrupt(BOARD_RIGHT_SENSE, rightBumperHandler, RISING);
  }
  
  if(tripSensorStatus & LINE_SENSOR)
  {
    tripSensorStatus &= ~LINE_SENSOR;
    SerialUSB.println("<sensor line:1>");
  }
  if((lineSensorLockout > 0) && (millis() > lineSensorLockout))
  {
    lineSensorLockout = -1;
    attachInterrupt(BOARD_LINE_SENSE, lineSensorHandler, RISING);
  }
}

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
    SerialUSB.print("Setting Left = ");
    SerialUSB.println(leftSpeed);
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
    SerialUSB.print("Setting Right = ");
    SerialUSB.println(rightSpeed);
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
 * <arc r:[radius value] v:[tangential velocity]>
 * <rotate a:[angle to rotate through] w:[angular velocity]>
 * NOTE: the values are relative and don't corrispond to and real values
 */
void parseCommand(char * cmd)
{
  SerialUSB.println(cmd);
  const char * arcCmd = "arc";
  const char * rotateCmd = "rotate";
  
  if(cmd[0] == 'a')
  {
    if(cmd[1] == 'r' && cmd[3] == 'c')
    {
      /* Execute arc command */
      SerialUSB.println("exe arc!");
    }
  }
  else if(cmd[0] == 'r')
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
      /* Execute Rotate Command */
      SerialUSB.println("Exe rotate!");
    }
  }
  else if(cmd[0] == 'm')
  {
    int i = 0;
    while(cmd[i++] != ' ');
    int l = atol(&cmd[i]);
    while(cmd[i++] != ' ');
    int r = atol(&cmd[i]);
    l = constrain(l, -1023, 1023);
    r = constrain(r, -1023, 1023);
    
    leftGoalSpeed = l;
    nextLeftUpdate = 0;
    rightGoalSpeed = r;
    nextRightUpdate = 0;
    
    SerialUSB.print("Got l:");
    SerialUSB.print(l);
    SerialUSB.print(" r:");
    SerialUSB.println(r);
  }
}

      
    

