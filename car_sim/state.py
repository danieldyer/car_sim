# These objects hold the simulation state.
class WheelState:
    __slots__ = ['angular_velocity_rad_s']
    def __init__(self):
        self.angular_velocity_rad_s = 0.0
        
class EngineState:
    __slots__ = ['gear', 'rpm']
    def __init__(self):
        # Gear 0 is idle; gear -1 is reverse.
        self.gear = 0
        self.rpm = 1000.0

class VehicleState:
    __slots__ = ['wheels', 'engine', 'position_m', 'heading_rad', 'velocity_m_s', 'angular_velocity_rad_s']
    def __init__(self):
        self.wheels = [WheelState(), WheelState(), WheelState(), WheelState()]
        self.engine = EngineState()
        
        self.position_m = [0.0, 0.0]
        self.heading_rad = 0.0
        self.velocity_m_s = [0.0, 0.0]
        self.angular_velocity_rad_s = 0.0

class ControlState:
    __slots__ = ['steering_angle', 'throttle', 'brake']
    def __init__(self):
        self.steering_angle = 0.0
        self.throttle = 0.0
        self.brake = 0.0
        
class ObjectiveState:
    __slots__ = ['bearing_rad', 'distance_m', 'index']
    def __init__(self):
        self.bearing_rad = 0.0
        self.distance_m = 0.0
        self.index = 0

class SimulationState:
    __slots__ = ['vehicle', 'controls', 'objective', 'time']
    def __init__(self):
        self.vehicle = VehicleState()
        self.controls = ControlState()
        self.objective = ObjectiveState()
        
        self.time = 0.0
