from pyPS4Controller.controller import Controller

# Test for PS4 controller for car controls
class MyController(Controller):
    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)

    def on_L3_down(self, value):
       print("L3 down: " + str(value))

    def on_L3_left(self, value):
       print("L3 left: " + str(value))

    def on_L3_up(self, value):
       print("L3 up: " + str(value))

    def on_L3_right(self, value):
       print("L3 right: " + str(value))

    def on_L3_x_at_rest(self):
       print("L3 Rest")

controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)
controller.listen(timeout=60)