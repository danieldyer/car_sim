import os
import datetime
import copy

from ipycanvas import Canvas, hold_canvas
from ipywidgets import Image
from ipyevents import Event
from time import sleep
from threading import Thread

from car_sim.parameters import *
from car_sim.state import *
from car_sim.dynamics import *

class CarSim(Thread):
    def __init__(self, canvas, control_function, init_pos, objective, scale):
        super(CarSim, self).__init__()

        self.canvas = canvas
        canvas.restore()

        car_sprite = Image.from_file(os.path.join(
            os.path.dirname(__file__),'sprites/993_gt2vg.png'))
        self.car_canvas = Canvas()
        self.car_canvas.draw_image(car_sprite)

        self.sim_state = SimulationState()
        self.sim_params = SimulationParameters()

        self.sim_state.vehicle.position_m = copy.copy(init_pos)
        self.sim_state.vehicle.engine.gear = 1

        self.sim_params.objective.position_m = numpy.array(copy.copy(objective))
        self.scale_factor = copy.copy(scale)

        self.control_function = control_function

        # Set up scale factors.
        self.m_to_px = 128.0 / self.sim_params.vehicle.length_m

        self.camera_position = numpy.array([0.0, 0.0])
        self.camera_deadzone = numpy.array([canvas.height / 3.0, canvas.width / 3.0])
        self.camera_pan_gain = 0.1;

        # Slip threshold for drawing skid marks.
        self.skid_threshold = 5.0
        self.skid_canvas = Canvas(width=canvas.width * 4.0, height=canvas.height * 4.0)
        self.skid_canvas.line_width = 10.0
        self.skid_canvas.stroke_style = 'black'
        self.skid_canvas.translate(self.skid_canvas.width / 2.0, self.skid_canvas.height / 2.0)
        self.skid_canvas.rotate(math.pi / 2.0)
        self.skid_canvas.scale(self.scale_factor)
        
        # Get objective information.
        self.num_objectives = self.sim_params.objective.position_m.shape[0]

        self.running = False

    def stop(self):
        self.running = False

    def run(self):
        self.running = True
        while self.running:
            start_tm = datetime.datetime.now()

            try:
                sim_step(self.sim_params, self.sim_state, self.control_function)
            except Exception as e:
                import traceback
                err_msg = traceback.format_exc()

                self.canvas.clear()
                self.canvas.font = '16px sans-serif'
                self.canvas.fill_style = 'red'

                y_pos = 16
                for l in err_msg.split('\n'):
                    self.canvas.fill_text(
                        l, 10, y_pos)
                    y_pos = y_pos + 20

                self.running = False
                break

            fl_slip, fr_slip, rl_slip, rr_slip = get_wheel_slip_speeds(self.sim_params, self.sim_state)

            with hold_canvas(self.canvas):
                self.canvas.clear()
                self.canvas.fill_style = '#c0c0c0'
                self.canvas.fill_rect(0, 0, self.canvas.width, self.canvas.height)

                # Draw objective info.
                self.canvas.font = '16px monospace'
                self.canvas.fill_style = 'black'
                self.canvas.fill_text(
                    f'Distance: {self.sim_state.objective.distance_m:.1f} m', 10, 16)
                self.canvas.fill_text(
                    f'Bearing:  {self.sim_state.objective.bearing_rad*180.0/math.pi:.1f} deg', 10, 34)

                # Draw skid marks.
                with hold_canvas(self.skid_canvas):
                    self.skid_canvas.save()
                    
                    self.skid_canvas.translate(
                        -self.sim_state.vehicle.position_m[0] * self.m_to_px,
                        -self.sim_state.vehicle.position_m[1] * self.m_to_px
                    )

                    for i, slip in enumerate([fl_slip, fr_slip, rl_slip, rr_slip]):
                        if slip > self.skid_threshold:
                            self.skid_canvas.save()

                            self.skid_canvas.rotate(self.sim_state.vehicle.heading_rad)

                            self.skid_canvas.translate(
                                -self.sim_params.vehicle.wheels[i].position_m[0] * self.m_to_px,
                                -self.sim_params.vehicle.wheels[i].position_m[1] * self.m_to_px
                            )

                            self.skid_canvas.global_alpha = min(1.0, 0.2 + \
                                (math.atan((slip - self.skid_threshold) / 10.0) / (math.pi / 2.0)))
                            self.skid_canvas.fill_rect(-5.0, -5.0, 10.0, 10.0)

                            self.skid_canvas.restore()

                self.canvas.save()
                self.canvas.translate(-self.camera_position[1] * self.m_to_px * self.scale_factor,
                                      self.camera_position[0] * self.m_to_px * self.scale_factor)
                self.canvas.draw_image(self.skid_canvas, -(self.skid_canvas.width - self.canvas.width) * 0.5,
                                                         -(self.skid_canvas.height - self.canvas.height) * 0.5)
                self.canvas.restore()
                self.skid_canvas.restore()

                # Draw the rest of the frame.

                self.canvas.save()
                self.canvas.translate(self.canvas.width / 2.0, self.canvas.height / 2.0)
                self.canvas.rotate(math.pi / 2.0)
                self.canvas.scale(self.scale_factor)

                # Draw objectives.
                for o in range(self.num_objectives):
                    if self.sim_state.objective.index == o:
                        self.canvas.fill_style = 'red'
                    else:
                        self.canvas.fill_style = 'green'
                    self.canvas.global_alpha = 0.5
                    self.canvas.fill_arc(
                        (-self.sim_params.objective.position_m[o, 0] + self.camera_position[0]) * self.m_to_px,
                        (-self.sim_params.objective.position_m[o, 1] + self.camera_position[1]) * self.m_to_px,
                        2.5 * self.m_to_px, 0.0, 2.0*math.pi
                    )
                    
                    self.canvas.global_alpha = 1.0
                    self.canvas.fill_style = 'black'
                    self.canvas.font = '128px sans-serif'
                    self.canvas.save()
                    self.canvas.translate((-self.sim_params.objective.position_m[o, 0] + self.camera_position[0]) * self.m_to_px,
                                          (-self.sim_params.objective.position_m[o, 1] + self.camera_position[1]) * self.m_to_px)
                    self.canvas.rotate(-math.pi / 2.0)
                    self.canvas.fill_text(
                        f'{o}', -36.0, 40.0
                    )
                    self.canvas.restore()

                # Car translation and rotation.
                self.canvas.translate(
                    (-self.sim_state.vehicle.position_m[0] + self.camera_position[0]) * self.m_to_px,
                    (-self.sim_state.vehicle.position_m[1] + self.camera_position[1]) * self.m_to_px
                )
                self.canvas.rotate(self.sim_state.vehicle.heading_rad)

                self.canvas.draw_image(self.car_canvas, -64, -29.5)
                self.canvas.restore()

                # Check termination condition.
                if self.sim_state.objective.distance_m < 2.5:
                    if self.sim_state.objective.index == self.num_objectives - 1:
                        if numpy.linalg.norm(self.sim_state.vehicle.velocity_m_s) < 0.1:
                            # Last waypoint.
                            self.running = False
                            self.canvas.font = '32px sans-serif'
                            self.canvas.fill_text(
                                f'Done! Time: {self.sim_state.time:.1f} s',
                                self.canvas.width / 2.0, self.canvas.height / 2.0)
                    else:
                        self.sim_state.objective.index += 1

            loop_time = (datetime.datetime.now() - start_tm).total_seconds()
            sleep(max(0, self.sim_params.dt - loop_time))

            # Update camera position.
            vehicle_offset_px = (self.sim_state.vehicle.position_m - self.camera_position) * self.m_to_px * self.scale_factor
            
            self.camera_position += numpy.array([
                math.copysign(max(0.0, abs(vehicle_offset_px[0]) - self.canvas.height / 2.0 + self.camera_deadzone[0]), vehicle_offset_px[0]),
                math.copysign(max(0.0, abs(vehicle_offset_px[1]) - self.canvas.width / 2.0 + self.camera_deadzone[1]), vehicle_offset_px[1])
            ]) * self.camera_pan_gain / (self.m_to_px * self.scale_factor)
