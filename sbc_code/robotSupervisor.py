from RobotMessage import RobotMessage
import serial
from time import sleep
from time import now
from threading import Timer

STATES = {
'init': 1,
'moving_arc': 3,
'moving_rotate': 4,
'done': 5
}

standard_arc = {'r':300, 'w':700}
standard_rotate = {'a':23, 't':50}

EVENTS = ['user_button_hit', 'left_bumper_hit', 'line_hit']

currentState = 'init'
startTime = now()

def state_transition(msg):
    global currentState
    global ser
    if currentState == 'init':
        if msg.msg_type == 'sensor':
            if 'user' in msg.target_values:
                print 'got init'
                currentState = 'moving_arc'
                sleep(2)
                startTime = now()
                print 'got command'
                ser.write(str(RobotMessage('arc', standard_arc)))
                
    elif currentState == 'moving_arc':
        if msg.msg_type == 'sensor':
            if 'right' in msg.target_values:
                print 'starting rotate'
                ser.write(str(RobotMessage('rotate', standard_rotate)))
                sleep(.750)
                print 'starting arc'
                ser.write(str(RobotMessage('arc', standard_arc)))
            if 'user' in msg.target_values:
                ser.write('<m 0 0>')
                sleep(5)
                ser.write(str(RobotMessage('arc', standard_arc)))
            if 'line' in msg.target_values:
                print 'saw sensor'
                if (now() - startTime) > 10:
                    currentState = 'done'
                    sleep(0.5)
                    ser.write('<m 0 0>')
                    state = 'done'
    elif currentState == 'done':
        if msg.msg_type == 'sensor':
            if 'user' in msg.target_values:
                print restarting
                currentState = 'moving_arc'
                sleep(2)
                startTime = now()
                print 'got command'
                ser.write(str(RobotMessage('arc', standard_arc)))
                
    
def handleLine(line):
    msg = RobotMessage.from_string(line)
    if msg is not None:
        state_transition(msg)
    

    
ser = serial.Serial(
    port='/dev/ttyAMA0',
    baudrate = 57600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
    )
    
    
while(True):
    line = ser.readline()
    print line.__repr__()
    handleLine(line)    
    