from machine import Pin, PWM
from sys import stdin
import time, uselect


led = Pin(25, Pin.OUT)
led.value(1)

PWM_CAP = 32768


class Wheel:
    kp = 35
    kd = 60
    ki = 0.0005

    def __init__(self, fwd_pin, bwd_pin, enc_a, enc_b, is_starboard, name) -> None:
        if is_starboard:
            fwd_pin, bwd_pin = (bwd_pin, fwd_pin)
            enc_a, enc_b = (enc_b, enc_a)

        self.fwd = PWM(Pin(fwd_pin, Pin.OUT))
        self.bwd = PWM(Pin(bwd_pin, Pin.OUT))

        self.fwd.freq(100)
        self.bwd.freq(100)

        self.enc_a = Pin(enc_a, Pin.IN, Pin.PULL_UP)
        self.enc_b = Pin(enc_b, Pin.IN, Pin.PULL_UP)

        self.name = name

        self.enc_a.irq(trigger=Pin.IRQ_RISING, handler=self.handle_encoder_a)
        self.enc_b.irq(trigger=Pin.IRQ_RISING, handler=self.handle_encoder_b)

        self.pos = 0
        self.target = 0
        self.prev_error = 0
        self.integral_error = 0
        self.speed = 0

        self.prev_time = time.ticks_us()
        self.curr_time = time.ticks_us()

    def handle_encoder_a(self, pin):
        self.pos += 1 if self.enc_b.value() > 0 else -1

    def handle_encoder_b(self, pin):
        self.pos += -1 if self.enc_a.value() > 0 else 1

    def run_pid_control(self):
        state = machine.disable_irq()
        try:
            """source https://curiores.com/positioncontrol"""
            self.curr_time = time.ticks_us()
            # delta time in seconds
            delta_time = time.ticks_diff(self.curr_time, self.prev_time) / 1.0e6

            error = self.pos - self.target
            d_error = (error - self.prev_error) / delta_time

            if self.speed < 0.9 * PWM_CAP:
                self.integral_error += error * delta_time

            # control signal
            u = self.kp * error + self.kd * d_error + self.ki * self.integral_error

            self.speed = int(min(abs(u), PWM_CAP))
            dir = -1 if u > 0 else 1
            self.spin(dir)
            self.prev_error = error
        finally:
            machine.enable_irq(state)

    def spin(self, dir):
        pwm_val = self.speed
        if pwm_val < 500:
            return
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

    def __str__(self) -> str:
        return self.name


starboard_front = Wheel(0, 2, 1, 3, True, "sf")
port_front = Wheel(4, 6, 5, 7, False, "pf")

starboard_back = Wheel(8, 10, 9, 11, True, "sb")
port_back = Wheel(12, 14, 13, 15, False, "pb")

wheels = {
    "pf": port_front,
    "pb": port_back,
    "sf": starboard_front,
    "sb": starboard_back,
}

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
                print(wheel.name, ":", wheel.pos, "->", wheel.target, sep="")
            except ValueError:
                print(c[2:], "?", sep="")
        else:
            print("?=", repr(c))
    cmd = ""
