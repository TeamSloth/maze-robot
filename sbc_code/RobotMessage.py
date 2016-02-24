

# class that represents a message
class RobotMessage(object):
    def __init__(self, msg_type=None, target_values=None, opts=None):
        self.msg_type = msg_type
        # dont use [] or {} as default values as modification to the object
        # modifies the default value itself
        # see: http://effbot.org/zone/default-values.htm
        if target_values is None:
            self.target_values = {}
        else:
            self.target_values = target_values
        if opts is None:
            self.opts = []
        elif hasattr(opts, 'lower'):
            self.opts = []
            self.opts.append(opts)
        else:
            self.opts = opts
    
    def add_target(self, target, value):
        self.target_values[target] = value
    
    def add_opt(self, opt):
        self.opts.append(opt)
    
    def parse_msg(self, msg):
        idx = msg.find('<')
        idx_end = msg.find('>')
        if idx < 0 or idx_end < 0: # Invalid message
            return None
        
        m = msg[idx+1:idx_end]
        tok = m.split()
        if len(tok) != 0:
            cmd = tok[0]
            opt_idx = cmd.find('(')
            opt_idx_end = cmd.find(')')
            if opt_idx*opt_idx_end < 0: # if only one prenthisis is found -> bad string
                return None
            
            if opt_idx < 0:
                self.msg_type = cmd
            else:
                opt_str = cmd[opt_idx+1:opt_idx_end]
                self.msg_type = cmd[:opt_idx]
                for o in opt_str.split(','):
                    self.add_opt(o)
            
            for t in tok[1:]:
                x = t.split(':')
                self.add_target(x[0], x[1])
        return msg[idx_end+1:]
    
    @classmethod
    def from_string(cls, str):
        msg_obj = cls()
        if msg_obj.parse_msg(str) is not None:
            return msg_obj
        else:
            return None
    
    def __str__(self):
        target_value_pairs = ' '.join(['{}:{}'.format(t,v) for t,v in self.target_values.iteritems()])
        if self.opts:
            opt_list = ','.join(self.opts)
            return '<{}({}) {}>'.format(self.msg_type, opt_list, target_value_pairs)
        elif self.target_values:
            return '<{} {}>'.format(self.msg_type, target_value_pairs)
        elif self.msg_type:
            return '<{}>'.format(self.msg_type)
        else:
            return '<>'

if __name__ == "__main__":
    # test scripts
    m1 = RobotMessage()
    m2 = RobotMessage('reset')
    m3 = RobotMessage('speed', {'m1':20, 'm2':-25})
    m4 = RobotMessage('speed', {'m1':20, 'm2':-25}, 'rad/sec')
    m5 = RobotMessage('speed', {'m1':20, 'm2':-25}, ['rad/sec','blue'])
    
    msgs = [m1,m2,m3,m4,m5]
    new_objs = []
    for m in msgs:
        print m
        new_objs.append(RobotMessage.from_string(str(m)))
    
    for m in new_objs:
        print m
    
    