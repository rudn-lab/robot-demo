from machine import Pin, PWM
from sys import exit, stdin
import time, math
import uselect


led = Pin(25, Pin.OUT)
led.value(1)

PWM_CAP = 65535


class Wheel:
    kp = 1
    kd = 0.5
    ki = 0.0001

    def __init__(self, fwd_pin, bwd_pin, enc_a, enc_b, is_port) -> None:
        self.fwd = PWM(Pin(fwd_pin, Pin.OUT))
        self.bwd = PWM(Pin(bwd_pin, Pin.OUT))
        self.fwd.freq(488)
        self.bwd.freq(488)

        self.enc_a = Pin(enc_a, Pin.IN, Pin.PULL_UP)
        self.enc_b = Pin(enc_b, Pin.IN, Pin.PULL_UP)

        self.is_port = is_port

        self.enc_a.irq(trigger=Pin.IRQ_RISING, handler=self.handle_encoder_interrupt)
        self.enc_b.irq(trigger=Pin.IRQ_RISING, handler=self.handle_encoder_interrupt)

        self.pos = 0
        self.target = 0
        self.prev_error = 0
        self.integral_error = 0

        self.prev_time = time.ticks_us()
        self.curr_time = time.ticks_us()

    def handle_encoder_interrupt(self, pin):
        inc = 0
        if pin == self.enc_a:
            inc = 1 if self.enc_b.value() > 0 else -1
        else:
            inc = -1 if self.enc_a.value() > 0 else 1

        self.pos += inc if self.is_port else -inc

    def run_pid_control(self):
        state = machine.disable_irq()
        try:
            """source https://curiores.com/positioncontrol"""

            self.curr_time = time.ticks_us()
            # delta time in seconds
            delta_time = time.ticks_diff(self.curr_time, self.prev_time) / 1.0e6

            error = self.pos - self.target
            d_error = (error - self.prev_error) / delta_time
            self.integral_error += error * delta_time

            # control signal
            u = self.kp * error + self.kd * d_error + self.ki * self.integral_error

            pwr = min(abs(u), PWM_CAP)
            dir = 1 if u > 0 else -1
            self.spin(dir, int(pwr))
            self.prev_error = error
        finally:
            machine.enable_irq(state)

    def spin(self, dir, pwm_val):
        dir *= -1
        pwm_val *= 1
        if dir > 0:
            self.fwd.duty_u16(pwm_val)
            self.bwd.duty_u16(0)
        elif dir < 0:
            self.fwd.duty_u16(0)
            self.bwd.duty_u16(pwm_val)
        else:
            self.fwd.duty_u16(0)
            self.bwd.duty_u16(0)

    def stop(self):
        self.target = self.pos
        self.prev_error = 0
        self.integral_error = 0


starboard_front = Wheel(0, 2, 1, 3, False)
port_front = Wheel(4, 6, 5, 7, True)

starboard_back = Wheel(8, 10, 9, 11, False)
port_back = Wheel(12, 14, 13, 15, True)

wheels = {
    "pf": port_front,
    "pb": port_back,
    "sf": starboard_front,
    "sb": starboard_back,
}

print("Type start() to begin...")


cmd = ""
while True:
    for wheel in wheels.values():
        wheel.run_pid_control()
    poll = uselect.poll()
    poll.register(stdin, uselect.POLLIN)
    res = poll.poll(1)
    line_terminated = False
    while res:
        inp = stdin.read(1)
        if inp == "\n":
            line_terminated = True
        cmd += inp
        res = poll.poll(1)
    if not line_terminated:
        continue
    line_terminated = False
    for c in cmd.split():
        if c == "!!":
            print("EXIT")
            break
        elif c == "!":
            for wheel in wheels.values():
                wheel.stop()
            print("STOP")
        elif c == "?":
            for name, w in wheels.items():
                print(name, ":", w.pos, "->", w.target, sep="")
        elif c and c[:2] in wheels:
            wheel = wheels[c[:2]]
            try:
                wheel.target = int(c[2:])
                print(c[:2], "->", c[2:], sep="")
            except ValueError:
                print(c[2:], "?", sep="")
        else:
            print("?=", repr(c))
    cmd = ""
