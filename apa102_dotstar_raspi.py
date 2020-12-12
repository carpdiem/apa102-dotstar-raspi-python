from itertools import permutations
import spidev

valid_arrangements = ['linear']
valid_update_strategies = ['on-command']

class DotstarDevice:
    def __init__(self, num_LEDs, arrangement, color_order, max_Hz = 3000000, bus = 0, device = 0, update = 'on-command'):
        self.num_LEDs = num_LEDs
        # Store LEDs state in (brightness, r, g, b) order
        self.LEDs_state = [(0, 0, 0, 0)] * self.num_LEDs
        if arrangement not in valid_arrangements:
            raise ValueError("invalid arrangement type")
        else:
            self.arrangement = arrangement
        if color_order.lower() not in [''.join(p) for p in permutations('bgr')]:
            raise ValueError("invalid color_order value")
        else:
            self.color_order = color_order.lower()
        self.max_Hz = float(max_Hz)
        self.device = int(device)
        self.bus = int(bus)
        if update not in valid_update_strategies:
            raise ValueError("invalid update strategy")
        else:
            self.update_strategy = update
        self.spi = spidev.SpiDev()
        self.spi.open(self.bus, self.device)
        self.spi.max_speed_hz = int(self.max_Hz)

    def set_LEDs(self, start_idx, end_idx, brightness, r, g, b):
        if start_idx < 0 or start_idx > self.num_LEDs - 1:
            raise ValueError("invalid start index: " + str(start_idx))
        if end_idx < start_idx or end_idx > self.num_LEDs:
            raise ValueError("invalid end index: " + str(end_idx))
        if brightness not in range(32):
            raise ValueError("invalid brightness value: " + brightness)
        if r not in range(256) or g not in range(256) or b not in range(256):
            raise ValueError("invalid rgb tuple: (" + str(r) + ", " + str(g) + ", " + str(b) + ")")
        for i in range(start_idx, end_idx):
            self.LEDs_state[i] = (brightness, r, g, b)

    def set_LED(self, idx, brightness, r, g, b):
        self.set_LEDs(idx, idx+1, brightness, r, g, b)

    def commit_state(self):
        to_send = self.start_frame() + self.state_to_bytes() + self.end_frame()
        self.spi.xfer(to_send)

    def start_frame(self):
        return [0x00] * 4

    def end_frame(self):
        return [0x00] * (int(self.num_LEDs / 128) + 1) * 4

    def state_to_bytes(self, state = []):
        if state == []:
            state = self.LEDs_state
        to_send = [self.tuple_to_bytes(LED) for LED in state]
        return to_send

    def tuple_to_bytes(self, t):
        if t[0] not in range(32) or t[1] not in range(256) or t[2] not in range(256) or t[3] not in range(256):
            raise ValueError("invalid LED state tuple: " + str(t))
        color_indices = [int(c) for c in self.color_order.replace('r', '1').replace('g', '2').replace('b', '3')]
        to_send = [0xe0 + t[0]]
        to_send += [t[color_indices[0]], t[color_indices[1]], t[color_indices[2]]]
        return to_send

    def get_full_state(self):
        return self.LEDs_state

    def get_LED_state(self, idx):
        return self.LEDs_state[idx]

    def commit_off(self):
        to_send = self.start_frame() + self.state_to_bytes([(0, 0, 0, 0)] * self.num_LEDs) + self.end_frame()
        self.spi.xfer(to_send)
