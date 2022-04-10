import math
import copy
import numpy

def calculate_diff_torque(engine_params, engine_state, throttle):
    engine_torque = numpy.interp(engine_state.rpm,
                                 engine_params.torque_map_bp_rpm,
                                 engine_params.torque_map_nm,
                                 left=engine_params.torque_map_nm[0],
                                 right=engine_params.torque_map_nm[-1]) * throttle
    
    # Rev-limiter.
    if engine_state.rpm > engine_params.torque_map_bp_rpm[-1]:
        engine_torque = 0.0
    
    # Apply gearing.
    if engine_state.gear == -1:
        transmission_torque = -engine_torque * engine_params.gear_ratios[0]
    elif engine_state.gear == 0:
        transmission_torque = 0.0
    else:
        transmission_torque = engine_torque * \
            engine_params.gear_ratios[engine_state.gear - 1]
    
    # Apply final drive ratio and drivetrain efficiency.
    return transmission_torque * engine_params.final_drive_ratio * engine_params.drivetrain_efficiency
    
def calculate_road_velocity_at_contact_patch(vehicle_state, wheel_params, wheel_state, track_angle):
    c_heading = math.cos(vehicle_state.heading_rad)
    s_heading = math.sin(vehicle_state.heading_rad)
    dcm_body_to_world = numpy.array([(c_heading, -s_heading), (s_heading, c_heading)])
    
    c_track = math.cos(track_angle)
    s_track = math.sin(track_angle)
    dcm_wheel_to_body = numpy.array([(c_track, -s_track), (s_track, c_track)])

    # Road velocity in wheel frame.
    road_velocity_wheel = dcm_wheel_to_body.transpose() @ \
        (-(dcm_body_to_world.transpose() @ vehicle_state.velocity_m_s) - \
        numpy.array([-wheel_params.position_m[1], wheel_params.position_m[0]]) * vehicle_state.angular_velocity_rad_s)
    
    # Contact patch velocity in wheel frame, relative to the vehicle.
    contact_patch_velocity_wheel = numpy.array([wheel_state.angular_velocity_rad_s * wheel_params.radius_m, 0])
    
    # Contact patch velocity in wheel frame, relative to the ground.
    contact_patch_velocity = contact_patch_velocity_wheel - road_velocity_wheel
    
    contact_patch_speed = numpy.linalg.norm(contact_patch_velocity)
    road_speed = numpy.linalg.norm(road_velocity_wheel)
        
    mu = calculate_tyre_mu(wheel_params, contact_patch_speed)
    
    return contact_patch_velocity, dcm_wheel_to_body, mu
    
def calculate_tyre_mu(wheel_params, slip_speed):
    return numpy.interp(slip_speed, wheel_params.slip_map_bp_m_s, wheel_params.slip_map_mu,
                        left=wheel_params.slip_map_mu[0], right=wheel_params.slip_map_mu[-1])
    
# Calculate total force and torque acting on the wheel.
def calculate_wheel_forces(sim_params, vehicle_state, wheel_params, wheel_state, track_angle,
                           normal_force, drive_torque, brake_fraction):
    contact_patch_velocity, dcm_wheel_to_body, mu = calculate_road_velocity_at_contact_patch(
        vehicle_state, wheel_params, wheel_state, track_angle)

    contact_patch_speed = numpy.linalg.norm(contact_patch_velocity)
    
    # Force vector is in the direction that it acts on the wheel, in wheel frame.
    if contact_patch_speed > 1e-6:
        force_vector = -contact_patch_velocity / contact_patch_speed
    else:
        force_vector = numpy.array([0.0, 0.0])
    
    friction_torque = force_vector[0] * mu * normal_force * wheel_params.radius_m

    # Calculate braking torque (including rolling resistance).
    braking_torque = math.copysign((wheel_params.max_brake_torque_nm * brake_fraction + \
                                    wheel_params.rolling_friction_coeff * normal_force),
                                   -wheel_state.angular_velocity_rad_s)
    
    # Apply numerical protections.
    damping_factor = 1.0
    
    # Numerical protection on friction torque.
    max_t = numpy.abs((wheel_params.inertia_moment_kgm2 * \
        (-contact_patch_velocity[0] / wheel_params.radius_m) * damping_factor) / sim_params.dt)
    friction_torque_unclipped = friction_torque
    friction_torque = numpy.clip(friction_torque, -max_t, max_t)
    
    if numpy.abs(friction_torque_unclipped) > 1e-6:
        longitudinal_force = force_vector[0] * (friction_torque / friction_torque_unclipped)
    else:
        longitudinal_force = 0.0
    
    contact_patch_force = numpy.array([longitudinal_force, force_vector[1]]) * mu * normal_force
    
    # Numerical protection on braking torque.
    if numpy.sign(braking_torque) != numpy.sign(wheel_state.angular_velocity_rad_s):
        max_t = numpy.abs((wheel_params.inertia_moment_kgm2 * \
                 wheel_state.angular_velocity_rad_s * damping_factor) / sim_params.dt)
        braking_torque = numpy.clip(braking_torque, -max_t, max_t)
    
    wheel_torque = drive_torque + braking_torque + friction_torque
    
    # Wheel force in body frame.
    wheel_force_body = dcm_wheel_to_body @ contact_patch_force
    
    return wheel_force_body, wheel_torque

def get_front_wheel_track_angles(control_state, vehicle_params):
    if numpy.abs(control_state.steering_angle) < 1e-6:
        fl_track_angle = 0.0
        fr_track_angle = 0.0
    else:
        turn_radius = vehicle_params.length_m / math.tan(control_state.steering_angle)
        fl_track_angle = math.atan2(vehicle_params.length_m,
                                    turn_radius - vehicle_params.wheels[0].position_m[1])
        fr_track_angle = math.atan2(vehicle_params.length_m,
                                    turn_radius - vehicle_params.wheels[1].position_m[1])
    
    return fl_track_angle, fr_track_angle

# Calculate forces and torques on the body and wheels.
def get_forces(sim_params, sim_state):
    # For the front wheels, calculate track angles.
    fl_track_angle, fr_track_angle = get_front_wheel_track_angles(sim_state.controls, sim_params.vehicle)
    
    # TODO: weight-shifting and suspension dynamics.
    normal_force = 9.81 * sim_params.vehicle.mass_kg
    
    # Calculate torque at the differential.
    diff_torque = calculate_diff_torque(sim_params.vehicle.engine,
                                        sim_state.vehicle.engine,
                                        sim_state.controls.throttle)
    
    # Calculate force and torque for each wheel.
    fl_force, fl_wheel_torque = calculate_wheel_forces(sim_params, sim_state.vehicle,
                                                       sim_params.vehicle.wheels[0], sim_state.vehicle.wheels[0],
                                                       fl_track_angle, normal_force * 0.2, 0.0,
                                                       sim_state.controls.brake)
    fr_force, fr_wheel_torque = calculate_wheel_forces(sim_params, sim_state.vehicle,
                                                       sim_params.vehicle.wheels[1], sim_state.vehicle.wheels[1],
                                                       fr_track_angle, normal_force * 0.2, 0.0,
                                                       sim_state.controls.brake)
    rl_force, rl_wheel_torque = calculate_wheel_forces(sim_params, sim_state.vehicle,
                                                       sim_params.vehicle.wheels[2], sim_state.vehicle.wheels[2],
                                                       0.0, normal_force * 0.3, -diff_torque * 0.5,
                                                       sim_state.controls.brake)
    rr_force, rr_wheel_torque = calculate_wheel_forces(sim_params, sim_state.vehicle,
                                                       sim_params.vehicle.wheels[3], sim_state.vehicle.wheels[3],
                                                       0.0, normal_force * 0.3, -diff_torque * 0.5,
                                                       sim_state.controls.brake)
    
    # Calculate body torque.
    body_torque = numpy.cross(sim_params.vehicle.wheels[0].position_m, fl_force) + \
                  numpy.cross(sim_params.vehicle.wheels[1].position_m, fr_force) + \
                  numpy.cross(sim_params.vehicle.wheels[2].position_m, rl_force) + \
                  numpy.cross(sim_params.vehicle.wheels[3].position_m, rr_force)
    
    # Convert forces to world frame.
    c_heading = math.cos(sim_state.vehicle.heading_rad)
    s_heading = math.sin(sim_state.vehicle.heading_rad)
    dcm_body_to_world = numpy.array([(c_heading, -s_heading), (s_heading, c_heading)])
    
    # Aerodynamic force (through centre of mass).
    vehicle_speed = numpy.linalg.norm(sim_state.vehicle.velocity_m_s)
    
    if vehicle_speed > 1e-6:
        drag_force = sim_params.vehicle.CdA_m2 * 1.225 * 0.5 * vehicle_speed**2.0 * \
            -numpy.array(sim_state.vehicle.velocity_m_s / vehicle_speed)
    else:
        drag_force = numpy.array([0.0, 0.0])
    
    return numpy.array(drag_force) + (dcm_body_to_world @ (fl_force + fr_force + rl_force + rr_force)), \
           body_torque, [fl_wheel_torque, fr_wheel_torque, rl_wheel_torque, rr_wheel_torque]

# Used for check for loss of traction.
def get_wheel_slip_speeds(sim_params, sim_state):
    fl_track_angle, fr_track_angle = get_front_wheel_track_angles(sim_state.controls, sim_params.vehicle)
    
    fl_cp_vel, _, _ = calculate_road_velocity_at_contact_patch(sim_state.vehicle,
                                                               sim_params.vehicle.wheels[0],
                                                               sim_state.vehicle.wheels[0],
                                                               fl_track_angle)
    fr_cp_vel, _, _ = calculate_road_velocity_at_contact_patch(sim_state.vehicle,
                                                               sim_params.vehicle.wheels[1],
                                                               sim_state.vehicle.wheels[1],
                                                               fr_track_angle)
    rl_cp_vel, _, _ = calculate_road_velocity_at_contact_patch(sim_state.vehicle,
                                                               sim_params.vehicle.wheels[2],
                                                               sim_state.vehicle.wheels[2],
                                                               0.0)
    rr_cp_vel, _, _ = calculate_road_velocity_at_contact_patch(sim_state.vehicle,
                                                               sim_params.vehicle.wheels[3],
                                                               sim_state.vehicle.wheels[3],
                                                               0.0)
    
    return numpy.linalg.norm(fl_cp_vel), \
           numpy.linalg.norm(fr_cp_vel), \
           numpy.linalg.norm(rl_cp_vel), \
           numpy.linalg.norm(rr_cp_vel)

# Simulation step function.
def sim_step(sim_params, sim_state, control_function):
    # Update objective state.
    num_objectives = sim_params.objective.position_m.shape[0]
    objective_index = numpy.clip(sim_state.objective.index, 0, num_objectives - 1)
    
    objective_vector = numpy.array(sim_params.objective.position_m[objective_index, :]) - sim_state.vehicle.position_m
    sim_state.objective.distance_m = numpy.linalg.norm(objective_vector)
    bearing = math.atan2(objective_vector[1], objective_vector[0]) - sim_state.vehicle.heading_rad
        
    sim_state.objective.bearing_rad = (bearing + math.pi) % (2.0 * math.pi) - math.pi # Wrap to [-pi, pi)
    
    # Call control function (pass a copy of the params and state).
    steering_angle, throttle, brake = control_function(copy.deepcopy(sim_params), copy.deepcopy(sim_state))
    
    # Limit steering angle, throttle and brake.
    sim_state.controls.steering_angle = numpy.clip(steering_angle,
                                                   -sim_params.vehicle.max_steering_angle,
                                                   sim_params.vehicle.max_steering_angle)
    sim_state.controls.throttle = numpy.clip(throttle, 0.0, 1.0)
    sim_state.controls.brake = numpy.clip(brake, 0.0, 1.0)
    
    # State vector for integrator.
    state_vector = numpy.concatenate((
        numpy.array(sim_state.vehicle.position_m),
        numpy.array([sim_state.vehicle.heading_rad]),
        numpy.array(sim_state.vehicle.velocity_m_s),
        numpy.array([sim_state.vehicle.angular_velocity_rad_s]),
        numpy.array([sim_state.vehicle.wheels[0].angular_velocity_rad_s]),
        numpy.array([sim_state.vehicle.wheels[1].angular_velocity_rad_s]),
        numpy.array([sim_state.vehicle.wheels[2].angular_velocity_rad_s]),
        numpy.array([sim_state.vehicle.wheels[3].angular_velocity_rad_s])
    ))
    
    def sim_ode(t, y):
        new_state = copy.deepcopy(sim_state)
        
        new_state.time = t
        new_state.vehicle.position_m[0] = y[0]
        new_state.vehicle.position_m[1] = y[1]
        new_state.vehicle.heading_rad = y[2]
        new_state.vehicle.velocity_m_s[0] = y[3]
        new_state.vehicle.velocity_m_s[1] = y[4]
        new_state.vehicle.angular_velocity_rad_s = y[5]
        new_state.vehicle.wheels[0].angular_velocity_rad_s = y[6]
        new_state.vehicle.wheels[1].angular_velocity_rad_s = y[7]
        new_state.vehicle.wheels[2].angular_velocity_rad_s = y[8]
        new_state.vehicle.wheels[3].angular_velocity_rad_s = y[9]
        
        body_force, body_torque, wheel_torques = get_forces(sim_params, new_state)
        
        vehicle_accel = body_force / sim_params.vehicle.mass_kg
        angular_accel = body_torque / sim_params.vehicle.inertia_moment_kgm2
        
        return numpy.array([
            y[3], # X-velocity
            y[4], # Y-velocity
            y[5], # Angular velocity
            vehicle_accel[0],
            vehicle_accel[1],
            angular_accel,
            wheel_torques[0] / sim_params.vehicle.wheels[0].inertia_moment_kgm2,
            wheel_torques[1] / sim_params.vehicle.wheels[1].inertia_moment_kgm2,
            wheel_torques[2] / sim_params.vehicle.wheels[2].inertia_moment_kgm2,
            wheel_torques[3] / sim_params.vehicle.wheels[3].inertia_moment_kgm2,
        ])
    
    state_vector = state_vector + sim_ode(sim_state.time, state_vector) * sim_params.dt
    
    sim_state.time += sim_params.dt
    sim_state.vehicle.position_m[0] = state_vector[0]
    sim_state.vehicle.position_m[1] = state_vector[1]
    sim_state.vehicle.heading_rad = (state_vector[2] + math.pi) % (2.0 * math.pi) - math.pi # Wrap to [-pi, pi)
    sim_state.vehicle.velocity_m_s[0] = state_vector[3]
    sim_state.vehicle.velocity_m_s[1] = state_vector[4]
    sim_state.vehicle.angular_velocity_rad_s = state_vector[5]
    sim_state.vehicle.wheels[0].angular_velocity_rad_s = state_vector[6]
    sim_state.vehicle.wheels[1].angular_velocity_rad_s = state_vector[7]
    sim_state.vehicle.wheels[2].angular_velocity_rad_s = state_vector[8]
    sim_state.vehicle.wheels[3].angular_velocity_rad_s = state_vector[9]
    
    # Update engine RPM based on driven wheel speed.
    wheel_speed = max(sim_state.vehicle.wheels[2].angular_velocity_rad_s,
                      sim_state.vehicle.wheels[3].angular_velocity_rad_s)
    
    if sim_state.vehicle.engine.gear == -1:
        sim_state.vehicle.engine.rpm = wheel_speed * \
            sim_params.vehicle.engine.final_drive_ratio * sim_params.vehicle.engine.gear_ratios[0]
    elif sim_state.vehicle.engine.gear == 0:
        pass
    else:
        sim_state.vehicle.engine.rpm = wheel_speed * \
            sim_params.vehicle.engine.final_drive_ratio * \
            -sim_params.vehicle.engine.gear_ratios[sim_state.vehicle.engine.gear - 1]
