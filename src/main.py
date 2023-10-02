from machine import Pin
from utime import sleep
from sys import exit, stdin


led = Pin(25, Pin.OUT)
led.value(1)


def front_encoder_handler(pin):
    print(f"front encoder interrupt for {pin}; val={pin.value()}")


def back_encoder_handler(pin):
    print(f"back encoder interrupt for {pin}; val={pin.value()}")


class Wheel:
    def __init__(self, fwd_pin, bwd_pin, enc_a, enc_b) -> None:
        self.fwd = Pin(fwd_pin, Pin.OUT)
        self.bwd = Pin(bwd_pin, Pin.OUT)

        self.enc_a = Pin(enc_a, Pin.IN, Pin.PULL_UP)
        self.enc_b = Pin(enc_b, Pin.IN, Pin.PULL_UP)

        self.enc_a.irq(trigger=Pin.IRQ_RISING, handler=front_encoder_handler)
        self.enc_b.irq(trigger=Pin.IRQ_RISING, handler=back_encoder_handler)

    def forward(self):
        print("hi")
        self.fwd.value(1)
        self.bwd.value(0)

    def backward(self):
        self.fwd.value(0)
        self.bwd.value(1)

    def stop(self):
        self.fwd.value(0)
        self.bwd.value(0)


bow_port = Wheel(0, 1, 2, 3)
stern_port = Wheel(4, 5, 6, 7)

bow_starboard = Wheel(8, 9, 10, 11)
stern_starboard = Wheel(12, 13, 14, 15)

wheels = {"bp": bow_port, "sp": stern_port, "bs": bow_starboard, "ss": stern_starboard}

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
