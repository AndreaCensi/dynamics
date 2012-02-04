

class DynamicsException(Exception):
    def __init__(self, dynamics, msg):
        Exception.__init__(self, dynamics, msg)
        self.dynamics = dynamics


class InvalidState(DynamicsException):
    def __init__(self, dynamics, state):
        self.state = state
        msg = 'Invalid state %r for %s.' % (state, dynamics)
        DynamicsException.__init__(dynamics, msg)


class InvalidCommands(DynamicsException):
    def __init__(self, dynamics, commands):
        self.commands = commands
        msg = 'Invalid commands %r for %s.' % (commands, dynamics)
        DynamicsException.__init__(dynamics, msg)
