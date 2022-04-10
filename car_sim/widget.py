from ipycanvas import Canvas
from ipywidgets import Button, HBox, VBox
from functools import partial

from car_sim.simulation import *

canvas = None
c = None

def get_car_sim_widget(ControlFunction, width=800, height=400, init_pos=[0.0, -20.0], objective=[[0.0, 20.0]], scale=0.5):
    global canvas
    global c
    
    def start_sim(b):
        global canvas
        global c

        try:
            c.stop()
        except:
            pass
        else:
            del c
            sleep(0.1)

        c = CarSim(canvas, ControlFunction, init_pos, objective, scale)

        c.start()

    def stop_sim(b):
        global c

        try:
            c.stop()
        except:
            pass
        else:
            del c

    canvas = Canvas(width=width, height=height)

    start_button = Button(
        description='Start',
        disabled=False,
        button_style='', # 'success', 'info', 'warning', 'danger' or ''
        tooltip='Start the simulation'
    )

    start_button.on_click(start_sim)

    stop_button = Button(
        description='Stop',
        disabled=False,
        button_style='', # 'success', 'info', 'warning', 'danger' or ''
        tooltip='Stop the simulation',
    )

    stop_button.on_click(stop_sim)

    return VBox([
        HBox([
            start_button,
            stop_button
        ]),
        canvas
    ])
