# implementation of a simple non-hierarchal state machine with entry/exit actions

import serial
from RobotMessage import RobotMessage
from threading import Timer

standard_arc = {'r':-300, 'w':700}
standard_rotate = {'a':45, 't':100}

# implements a finite state machine, state table is a dictionary look up table 
# of state-name:function-handle pairs. the functions implemnt the events that 
# the state can handle. These functions must be run-to-completion
class RobotStateMachine(object):
    def __init__(self, state_table, initial_state):
        self.state_table = state_table
        self._current_state = initial_state
        #execute the entry action of the initial state
        self.fire_event('entry')
        
    def fire_event(self, trigger, args=None):
        (self.state_table[self._current_state])(trigger, args)
    
    def transition(self, state):
        # make sure state is a valid state
        if state not in self.state_table:
            raise ValueError('Invalid state')
        
        # exicute the exit action of the current state
        print 'Exiting {}'.format(self._current_state)
        (self.state_table[self._current_state])('exit', None)
        self._current_state = state
        print 'Entering {}'.format(self._current_state)
        (self.state_table[self._current_state])('entry', None)


class MazeRobot(RobotStateMachine):
    def __init__(self, ser):
        self.ser = ser
        states = {'init':self.state_init,
                  'exit_gate':self.state_exit_gate,
                  'moving_arc':self.state_moving_arc,
                  'rotate':self.state_rotate,
                  'goal_found':self.state_goal_found}
                  
        RobotStateMachine.__init__(self, states, 'init')
    

    def timer_callback(self, target):
        target('timer', None)
        
    def state_example(self, trigger, args):
        if trigger == 'entry':
            pass
        elif trigger == 'exit':
            pass
        elif trigger == 'user_button':
            pass
        elif trigger == 'right_sensor':
            pass
        elif trigger == 'left_sensor':
            pass
        elif trigger == 'goal_sensor':
            pass
        elif trigger == 'timer':
            pass
        else:
            pass
        
    def state_init(self, trigger, args):
        if trigger == 'entry':
            print 'state_init: entry'
            self.ser.write('<m 0 0>')
            pass
        elif trigger == 'exit':
            pass
        elif trigger == 'user_button':
            print 'state_init: user_button'
            Timer(2.0, self.timer_callback, [self.state_init]).start()
        elif trigger == 'right_sensor':
            pass
        elif trigger == 'left_sensor':
            pass
        elif trigger == 'goal_sensor':
            pass
        elif trigger == 'timer':
            print 'state_init: timer'
            self.transition('exit_gate')
            pass
        else:
            pass

    def state_exit_gate(self, trigger, args):
        if trigger == 'entry':
            print 'state_exit_goal: entry'
            self.ser.write('<m 290 300>')
            pass
        elif trigger == 'exit':
            pass
        elif trigger == 'user_button':
            print 'state_exit_goal: user_button'
            self.transition('init')
            pass
        elif trigger == 'right_sensor':
            pass
        elif trigger == 'left_sensor':
            pass
        elif trigger == 'goal_sensor':
            print 'state_exit_goal: goal_sensor'
            self.transition('moving_arc')
            pass
        elif trigger == 'timer':
            pass
        else:
            pass
        
    def state_moving_arc(self, trigger, args):
        if trigger == 'entry':
            print 'state_moving_arc: entry'
            self.ser.write(str(RobotMessage('arc', standard_arc)))
            pass
        elif trigger == 'exit':
            pass
        elif trigger == 'user_button':
            print 'state_moving_arc: user_button'
            self.transition('init')
        elif trigger == 'right_sensor':
            print 'state_moving_arc: right_sensor'
            self.transition('rotate')
        elif trigger == 'left_sensor':
            pass
        elif trigger == 'goal_sensor':
            print 'state_moving_arc: goal_sensor'
            self.transition('goal_found')
        elif trigger == 'timer':
            pass
        else:
            pass

    def state_rotate(self, trigger, args):
        if trigger == 'entry':
            self.ser.write(str(RobotMessage('rotate', standard_rotate)))
            Timer(1.0, self.timer_callback, [self.state_init]).start()
            pass
        elif trigger == 'exit':
            pass
        elif trigger == 'user_button':
            print 'state_rotate: user_button'
            self.transition('init')
            pass
        elif trigger == 'right_sensor':
            pass
        elif trigger == 'left_sensor':
            pass
        elif trigger == 'goal_sensor':
            pass
        elif trigger == 'timer':
            self.transition('moving_arc')
            pass
        else:
            pass


    def state_goal_found(self, trigger, args):
        if trigger == 'entry':
            print 'state_goal_found: entry'
            self.ser.write('<m 0 0>')
            pass
        elif trigger == 'exit':
            pass
        elif trigger == 'user_button':
            print 'state_goal_found: user_button'
            self.transition('init')
            pass
        elif trigger == 'right_sensor':
            pass
        elif trigger == 'left_sensor':
            pass
        elif trigger == 'goal_sensor':
            pass
        elif trigger == 'timer':
            pass
        else:
            pass


    
def main():
    ser = serial.Serial(
        port='/dev/ttyAMA0',
        baudrate = 57600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=1
        )
    
    robot = MazeRobot(ser)
    
    while(True):
        line = ser.readline()
        print line.__repr__()
        
        msg = RobotMessage.from_string(line)
        if msg is not None:
            if msg.msg_type == 'sensor':
                if 'right' in msg.target_values:
                    print 'Saw right sensor'
                    robot.fire_event('right_sensor')
                if 'left' in msg.target_values:
                    print 'Saw left sensor'
                    robot.fire_event('left_sensor')
                if 'user' in msg.target_values:
                    print 'Saw user button'
                    robot.fire_event('user_button')
                if 'line' in msg.target_values:
                    print 'Saw goal sensor'
                    robot.fire_event('goal_sensor')        

if __name__ == "__main__":
    main()