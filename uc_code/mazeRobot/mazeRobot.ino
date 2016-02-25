/* Main source code for the maze robot. */
#define DXL_BUS_SERIAL1 1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04

/* Note: The Left wheel + direction is considered forward */
#define ID_NUM_LEFT 1
#define ID_NUM_RIGHT 2
/* Note: The Left wheel + direction is considered forward */
#define BOARD_RIGHT_DRIVE 16
#define BOARD_RIGHT_SENSE 17

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
int maxSpeedChange = 50;

void setup()
{
  Dxl.begin(3);
  Dxl.wheelMode(ID_NUM_LEFT); //wheelMode() is to use wheel mode
  Dxl.wheelMode(ID_NUM_RIGHT); //wheelMode() is to use wheel mode
  pinMode(BOARD_RIGHT_DRIVE, OUTPUT);
  digitalWrite(BOARD_RIGHT_DRIVE, 1);
  pinMode(BOARD_RIGHT_SENSE, INPUT_PULLDOWN);
//  attachInterrupt(BOARD_RIGHT_SENSE, turnLeft, RISING);
  pinMode(BOARD_LED_PIN, OUTPUT);
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
  
  /* Set the motor speed based on the current values */
  Dxl.goalSpeed(ID_NUM_RIGHT, 200);
  //Dxl.goalSpeed(ID_NUM_RIGHT, 210 | 0x400);
  

}

void setMotorSpeed(void)
{
  /* Note the timeings are still arbitraty, I havent done the math yet. */
  static long nextLeftUpdate = 0; 
  if(nextLeftUpdate == 0 || nextLeftUpdate < millis())
  {
    if(abs(leftGoalSpeed - leftSpeed) < maxSpeedChange)
    {
      leftSpeed = leftGoalSpeed;
      nextLeftUpdate = 0x7FFFFFFF;
    }
    else
    {
      nextLeftUpdate = millis() + 50;
      if((leftGoalSpeed - leftSpeed) > 0)
      {
        leftSpeed += maxSpeedChange;
      }
      else
      {
        leftSpeed -= maxSpeedChange;
      }
    }
    /* TODO: Actualy apply the update */
  }
  
  static long nextRightUpdate = 0; 
  if(nextRightUpdate == 0 || nextRightUpdate < millis())
  {
    if(abs(rightGoalSpeed - rightSpeed) < maxSpeedChange)
    {
      rightSpeed = rightGoalSpeed;
      nextRightUpdate = 0x7FFFFFFF;
    }
    else
    {
      nextRightUpdate = millis() + 50;
      if((rightGoalSpeed - rightSpeed) > 0)
      {
        rightSpeed += maxSpeedChange;
      }
      else
      {
        rightSpeed -= maxSpeedChange;
      }
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
      SerialUSB.println('exe arc!');
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
      SerialUSB.println('exe rotate!');
    }
  }
  else if(cmd[0] == 'm')
  {
    int i = 0;
    while(cmd[i++] != ' ');
    int l = atol(&cmd[i]);
    while(cmd[i++] != ' ');
    int r = atol(&cmd[i]);
    
    SerialUSB.print('Got l:');
    SerialUSB.print(r);
    SerialUSB.print(' r:');
    SerialUSB.println(l);
  }
    
}

      
    

