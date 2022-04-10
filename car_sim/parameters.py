import math
import numpy

# These objects hold simulation parameters.
class WheelParameters:
    __slots__ = ['position_m', 'radius_m', 'inertia_moment_kgm2', \
                 'max_brake_torque_nm', 'rolling_friction_coeff', \
                 'slip_map_bp_m_s', 'slip_map_mu']
    def __init__(self, position):
        wheelbase_m = 2.272
        front_track_m = 1.475
        rear_track_m = 1.550
        if position == 'fl':
            tyre_width_mm = 235.0
            tyre_profile_mm = 40.0
            wheel_diameter_in = 18.0
            self.position_m = [wheelbase_m / 2.0, -front_track_m / 2.0]
        elif position == 'fr':
            tyre_width_mm = 235.0
            tyre_profile_mm = 40.0
            wheel_diameter_in = 18.0
            self.position_m = [wheelbase_m / 2.0, front_track_m / 2.0]
        elif position == 'rl':
            tyre_width_mm = 285.0
            tyre_profile_mm = 30.0
            wheel_diameter_in = 18.0
            self.position_m = [-wheelbase_m / 2.0, -rear_track_m / 2.0]
        elif position == 'rr':
            tyre_width_mm = 285.0
            tyre_profile_mm = 30.0
            wheel_diameter_in = 18.0
            self.position_m = [-wheelbase_m / 2.0, rear_track_m / 2.0]
        else:
            raise Exception('Unrecognised wheel position')
        
        # Inertia moment calculations. Use 195/65/R15 weighing 9 kg as reference.
        tyre_mass_kg = 9.0 * (tyre_width_mm / 195.0) * \
            ((wheel_diameter_in * 25.4 * 0.5 + tyre_profile_mm) / (15.0 * 25.4 * 0.5 + 65.0))
        wheel_mass_kg = 18;
        
        wheel_izz_kgm2 = 0.5 * wheel_mass_kg * (wheel_diameter_in * 25.4 * 0.5 / 1000.0)**2.0
        tyre_izz_kgm2 = tyre_mass_kg * ((wheel_diameter_in * 25.4 * 0.5 + tyre_profile_mm) / 1000.0)**2.0
        
        self.inertia_moment_kgm2 = wheel_izz_kgm2 + tyre_izz_kgm2
        self.radius_m = (wheel_diameter_in * 25.4 * 0.5 + tyre_profile_mm) / 1000.0
        self.max_brake_torque_nm = 15000.0
        self.rolling_friction_coeff = 0.01
        
        # Tyre friction slip map.
        self.slip_map_bp_m_s = [0.0, 0.1, 0.2, 0.5, 0.8, 1.0]
        self.slip_map_mu = [0.0, 0.95, 1.1, 0.90, 0.85, 0.82]

class EngineParameters:
    __slots__ = ['torque_map_nm', 'torque_map_bp_rpm', 'gear_ratios', 'final_drive_ratio', 'drivetrain_efficiency']
    def __init__(self):
        self.torque_map_nm = [340, 400, 500, 560, 590, 600, 590, 570, 540, 490, 460]
        self.torque_map_bp_rpm = [2250, 2500, 3000, 3500, 4000, 4500, 5000, 5500, 6000, 6500, 6750]
        self.gear_ratios = [41.0 / 13.0, 40.0 / 20.0, 36.0 / 25.0, 34.0 / 30.0, 32.0 / 33.0, 29.0 / 35.0]
        self.final_drive_ratio = 31.0 / 9.0
        self.drivetrain_efficiency = 0.85
        
class VehicleParameters:
    __slots__ = ['wheels', 'engine', 'mass_kg', 'length_m', 'width_m', 'inertia_moment_kgm2',
                 'CdA_m2', 'max_steering_angle']
    def __init__(self):
        self.wheels = [WheelParameters('fl'), WheelParameters('fr'), WheelParameters('rl'), WheelParameters('rr')]
        self.engine = EngineParameters()
        self.mass_kg = 1295.0
        self.length_m = 4.245
        self.width_m = 1.855
        self.inertia_moment_kgm2 = (1.0 / 12.0) * self.mass_kg * (self.length_m**2.0 + self.width_m**2.0)
        self.CdA_m2 = 0.55
        
        # Calculate steering angle limits.
        turning_circle_m = 11.8
        self.max_steering_angle = math.atan2(self.length_m, (turning_circle_m / 2.0))

# Smoke puff properties.
class SmokeParameters:
    __slots__ = ['drag', 'duration', 'minsize', 'maxsize']
    def __init__(self):
        self.drag = 0.9
        self.duration = 3.0
        self.minsize = 16.0
        self.maxsize = 64.0

class ObjectiveParameters:
    __slots__ = ['position_m']
    def __init__(self):
        self.position_m = numpy.array([[0.0, 0.0]])

class SimulationParameters:
    __slots__ = ['dt', 'vehicle', 'smoke', 'objective']
    def __init__(self):
        self.dt = 1.0 / 60.0
        self.vehicle = VehicleParameters()
        self.smoke = SmokeParameters()
        self.objective = ObjectiveParameters()
