/* Main source code for the maze robot. */


/* buffers for the incomming serial data */
static char commandBuffer[256];
static int commandBufferIndex = 0;
static char overflowBuffer[256];
static int overflowBufferIndex = 0;

/* Flag to indicate that a command is currently being received */
int cmdReceiveInProgress = false;

void setup()
{

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

void parseCommand(char * cmd)
{
  SerialUSB.println(cmd);
}

      
    

