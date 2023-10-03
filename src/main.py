from machine import Pin
from utime import sleep
from sys import exit, stdin
import math


led = Pin(25, Pin.OUT)
led.value(1)


class Wheel:
    def __init__(self, fwd_pin, bwd_pin, enc_a, enc_b) -> None:
        self.fwd = Pin(fwd_pin, Pin.OUT)
        self.bwd = Pin(bwd_pin, Pin.OUT)

        self.enc_a = Pin(enc_a, Pin.IN, Pin.PULL_UP)
        self.enc_b = Pin(enc_b, Pin.IN, Pin.PULL_UP)

        self.enc_a.irq(trigger=Pin.IRQ_RISING, handler=self.read_encoder)

        self.pos_inc = 0

    def read_encoder(self, pin):
        enc_b_val = self.enc_b.value()
        self.pos_inc += 1 if enc_b_val > 0 else -1

    def forward(self):
        print("hi")
        self.fwd.high()
        self.bwd.low()

    def backward(self):
        self.fwd.low()
        self.bwd.high()

    def stop(self):
        self.fwd.low()
        self.bwd.low()


bow_port = Wheel(0, 1, 2, 3)
stern_port = Wheel(4, 5, 6, 7)

bow_starboard = Wheel(8, 9, 10, 11)
stern_starboard = Wheel(12, 13, 14, 15)

wheels = {"bp": bow_port, "sp": stern_port, "bs": bow_starboard, "ss": stern_starboard}


kp = 1
kd = 0.025
ki = 0.0


def pid_control(pos, target, delta_t, prev_error, integral_error):
    error = pos - target
    d_error = (error - prev_error) / delta_t
    integral_error += error * delta_t

    # control signal
    u = kp * error + kd * d_error + ki * integral_error

    pwr = max(math.fabs(u), 255)
    dir = 1 if u > 0 else -1

    # set motor
    # store error


cmd = None
while True:
    cmd = stdin.readline().strip()
    print(cmd)
    if cmd and cmd[:2] in wheels:
        wheel = wheels[cmd[:2]]
        if cmd[2:3] == "f":
            wheel.forward()
        elif cmd[2:3] == "b":
            wheel.backward()
        elif cmd[2:3] == "s":
            wheel.stop()
