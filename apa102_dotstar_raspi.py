from random import randrange
from numpy import log, array, ceil
from copy import deepcopy
from itertools import permutations
import spidev
import Color_Match as cm

valid_arrangements = ['linear']
valid_update_strategies = ['on-command']

class DotstarDevice:
    def __init__(self, num_LEDs, arrangement, color_order, thermal_protection_limit = False, max_Hz = 3000000, bus = 0, device = 0, update = 'on-command', dummy = False):
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
        # note, thermal protection limit is in units of (brightness ∈ [0, 31]) * (color_channel ∈ [0, 255]),
        #   and is enforced on each color channel separately
        if thermal_protection_limit != False:
            self.thermal_limit = thermal_protection_limit
        else:
            self.thermal_limit = 31 * 255
        self.max_Hz = float(max_Hz)
        self.device = int(device)
        self.bus = int(bus)
        if update not in valid_update_strategies:
            raise ValueError("invalid update strategy")
        else:
            self.update_strategy = update
        if not dummy:
            self.spi = spidev.SpiDev()
            self.spi.open(self.bus, self.device)
            self.spi.max_speed_hz = int(self.max_Hz)

    def unsafe_change_LED_state(self, idx, brightness, r, g, b):
        self.LEDs_state[idx] = (brightness, r, g, b)

    def safe_change_LED_state(self, idx, brightness, r, g, b):
        if brightness * r > self.thermal_limit:
            r = int(self.thermal_limit / brightness)
        if brightness * g > self.thermal_limit:
            g = int(self.thermal_limit / brightness)
        if brightness * b > self.thermal_limit:
            b = int(self.thermal_limit / brightness)
        self.LEDs_state[idx] = (brightness, r, g, b)

    def set_LEDs(self, start_idx, end_idx, brightness, r, g, b, safe = True):
        if start_idx < 0 or start_idx > self.num_LEDs - 1:
            raise ValueError("invalid start index: " + str(start_idx))
        if end_idx < start_idx or end_idx > self.num_LEDs:
            raise ValueError("invalid end index: " + str(end_idx))
        if brightness not in range(32):
            raise ValueError("invalid brightness value: " + brightness)
        if r not in range(256) or g not in range(256) or b not in range(256):
            raise ValueError("invalid rgb tuple: (" + str(r) + ", " + str(g) + ", " + str(b) + ")")
        for i in range(start_idx, end_idx):
            self.set_LED(i, brightness, r, g, b, safe)

    def set_LED(self, idx, brightness, r, g, b, safe = True):
        if safe:
            self.safe_change_LED_state(idx, brightness, r, g, b)
        else:
            self.unsafe_change_LED_state(idx, brightness, r, g, b)

    def set_nth_LEDs(self, start_idx, end_idx, n, brightness, r, g, b, safe = True):
        for i in range(start_idx, end_idx, n):
            self.set_LED(i, brightness, r, g, b, safe)

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
        return sum(to_send, [])

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

    def reset_LEDs_state(self):
        self.set_LEDs(0, self.num_LEDs, 0, 0, 0, 0)

    def set_LEDs_best_match_float_rgb(self, start_idx, end_idx, r, g, b, max_pattern_width = 6, debug = False, precompute = False):
        def n_batch_config(n, r, g, b, max_level, max_pattern_width = max_pattern_width):
            n = int(n)
            if n < 1 or n > max_pattern_width:
                raise ValueError("requested pattern is too large: n = " + str(n))
            if any([i < 0. or i > 1. for i in [r, g, b]]):
                raise ValueError("invalid rgb float tuple: " + str(r) + ", " + str(g) + ", " + str(b))
            irgbs = [r * max_level, g * max_level, b * max_level]
            
#            def gen_sorted_idx_lookups(irgbs):
#                irgbs_sorted = deepcopy(irgbs)
#                irgbs_sorted.sort()
#                irgbs_sorted = [[i, False] for i in irgbs_sorted]
#                irgbs_flags = [[i, False] for i in irgbs]
#                idx_lookups = []
#                for i in irgbs_sorted:
#                    idx = irgbs_flags.index(i)
#                    idx_lookups += [idx]
#                    irgbs_flags[idx][1] = True
#                return idx_lookups

            def LED_to_irgbs(led, n):
                return [1. * led[0] * led[1] / n, 1. * led[0] * led[2] / n, 1. * led[0] * led[3] / n]

            def recursive_n_config(n, irgbs, n_original):
                def find_best_incremental_LED(irgbs):
                # use a "growing" strategy. start the test_irgb at (1, 0, 0, 0) and test out all four options of +1 to see which
                #   (including the current point) produces the least log_error. Then make that step and repeat the function
                #   until you end up with the least error produced by the current point. Then return the current point.
                    ### ↓↓↓ check if this shouldn't be current_point = (1, 0, 0, 0) instead!!!
                    current_point = (0, 0, 0, 0)
                    next_point = (1, 0, 0, 0)
                    while next_point != current_point:
                        current_point = deepcopy(next_point)
                        candidate_points = [(current_point[0], current_point[1], current_point[2], current_point[3])]
                        if current_point[0] < 31:
                            candidate_points += [(current_point[0] + 1, current_point[1], current_point[2], current_point[3])]
                        if current_point[1] < 255:
                            candidate_points += [(current_point[0], current_point[1] + 1, current_point[2], current_point[3])]
                        if current_point[2] < 255:
                            candidate_points += [(current_point[0], current_point[1], current_point[2] + 1, current_point[3])]
                        if current_point[3] < 255:
                            candidate_points += [(current_point[0], current_point[1], current_point[2], current_point[3] + 1)]
#                        candidate_points += [(current_point[0], current_point[1], current_point[2], current_point[3])]
                        candidate_errors = [compute_log_error(irgbs, LED_to_irgbs(led, n_original)) for led in candidate_points]
                        next_point = candidate_points[candidate_errors.index(min(candidate_errors))]
                        if debug:
                            print("point iteration!!!!!")
                            print(candidate_points)
                            print(candidate_errors)
                            print(next_point)
                    return current_point

                if n == 1:
                    return [find_best_incremental_LED(irgbs)]
                elif n > 1:
                    best_LED = find_best_incremental_LED(irgbs)
                    best_LED_irgbs = [best_LED[0] * best_LED[1] / n_original, best_LED[0] * best_LED[2] / n_original, best_LED[0] * best_LED[3] / n_original]
                    irgb_residuals = [irgbs[0] - best_LED_irgbs[0], irgbs[1] - best_LED_irgbs[1], irgbs[2] - best_LED_irgbs[2]]
                    return [best_LED] + recursive_n_config(n - 1, irgb_residuals, n_original)

            res = recursive_n_config(n, irgbs, n)
                
            def adjust_ordering(cfg):
            # change the ordering of the LEDs, so they are maximally mixed with bright and dim LEDs next to each other
                cfg.sort(key=(lambda x: x[0] * x[1] + x[0] * x[2] + x[0] * x[3]))
                def alternating_index(i):
                    # produces indices like 0, -1, 1, -2, 2, -3, ...
                    return int((-1)**i * ceil(i / 2))
                return [cfg[alternating_index(i)] for i, _ in enumerate(cfg)]

            return adjust_ordering(res)

        def config_to_floats(cfg, max_level):
            r, g, b = 0., 0., 0.
            r = sum([1.0 * led[0] * led[1] / max_level for led in cfg]) / len(cfg)
            g = sum([1.0 * led[0] * led[2] / max_level for led in cfg]) / len(cfg)
            b = sum([1.0 * led[0] * led[3] / max_level for led in cfg]) / len(cfg)
            return (r, g, b)

        def config_to_irgbs(cfg):
            r, g, b = 0, 0, 0
            for led in cfg:
                r += led[0] * led[1]
                g += led[0] * led[2]
                b += led[0] * led[3]
            r /= len(cfg)
            g /= len(cfg)
            b /= len(cfg)
            if debug:
                print(cfg)
                print((r, g, b))
            return (r, g, b)

        def mean_squared_error(d0, d1):
            residuals = [((d0[i] - d1[i]) / d0[i])**2.0 for i in range(len(d0))]
            return sum(residuals) / len(residuals)

        def compute_log_error(desired_irgbs, test_irgbs):
            res = 1.0
            for pair in zip(desired_irgbs, test_irgbs):
                if pair[1] <= pair[0]:
                    res *= (log(pair[0]+1) - log(pair[1]+1) + 1)
                else:
                    res *= 2.0 * (log(pair[0] + 1) + 1)
            return res

        config_options = [n_batch_config(n, r, g, b, self.thermal_limit) for n in range(1, max_pattern_width + 1)]
#        errors = [mean_squared_error((r, g, b), config_to_floats(cfg, self.thermal_limit)) for cfg in config_options]
        errors = [compute_log_error([r * self.thermal_limit, g * self.thermal_limit, b * self.thermal_limit], config_to_irgbs(cfg)) for cfg in config_options]
        if debug:
            print(config_options)
            print(errors)
        best_config = config_options[errors.index(min(errors))]

        # now set the config
        pattern_length = len(best_config)

        # cycle the best_config a random number of times to evenly distribute % chance of any LED being the MAX LED.
        # the point of this is to make sure that we're not always blasting THE SAME LEDs with the high brightnesses,
        #   and thereby, high heats, in order to prolong the LED lifetimes :)
        cycle_offset = randrange(0, pattern_length)
        best_config = best_config[cycle_offset:] + best_config[:cycle_offset]

        if precompute:
            tmp = deepcopy(self.LEDs_state)
            self.set_pattern(start_idx, end_idx, best_config, safe = False)
            precompute_res = deepcopy(self.LEDs_state)
            self.unsafe_raw_set_LEDs_state(tmp)
            return precompute_res
        else:
            self.set_pattern(start_idx, end_idx, best_config, safe = False)
            

    def set_pattern(self, start_idx, end_idx, pattern, safe = True):
        for idx, led in enumerate(pattern):
            self.set_nth_LEDs(start_idx + idx, end_idx, len(pattern), led[0], led[1], led[2], led[3], safe = safe)

    def unsafe_raw_set_LEDs_state(self, state):
        self.LEDs_state = deepcopy(state)


class MultiDotstarController:
    def __init__(self, subdevice_configs, color_order, thermal_protection_limit = False, max_Hz = 3000000, bus = 0, device = 0, update = 'on-command', dummy = False):
        def check_integrity(cfgs):
            for cfg in cfgs:
                tests = {'has start_idx': 'start_idx' in cfg.keys(),
                         'has end_idx': 'end_idx' in cfg.keys(),
                         'start_idx < end_idx': cfg['start_idx'] < cfg['end_idx'],
                         'start_idx is int': type(cfg['start_idx']) == int,
                         'end_idx is int': type(cfg['end_idx']) == int,
                         'has l1': 'l1' in cfg.keys(),
                         'has l2': 'l2' in cfg.keys(),
                         'has l3': 'l3' in cfg.keys(),
                         '0 ≤ l1 ≤ 1e-6': 0. <= cfg['l1'] <= 1.e-6,
                         '0 ≤ l2 ≤ 1e-6': 0. <= cfg['l2'] <= 1.e-6,
                         '0 ≤ l3 ≤ 1e-6': 0. <= cfg['l3'] <= 1.e-6}
                if not all(tests.values()):
                    raise RuntimeError('Error - invalid subdevice configuration. Test results: ' + str(tests))
            for idx in range(len(cfgs))[:-1]:
                if cfgs[idx]['end_idx'] != cfgs[idx + 1]['start_idx']:
                    raise ValueError('Bad index continuity between ' + str(cfgs[idx]) + ' and ' + str(cfgs[idx + 1]))

        check_integrity(subdevice_configs)
        self.total_num_LEDs = subdevice_configs[-1]['end_idx']
        self.subdevice_configs = subdevice_configs
        self.control_interface = DotstarDevice(self.total_num_LEDs, 'linear', color_order, thermal_protection_limit = thermal_protection_limit, max_Hz = max_Hz, bus = bus, device = device, update = update, dummy = dummy)

    def match_sense_vector_on_subdevice(self, cfg, sv, brightness):
        rgbs = brightness * cm.rgb_composition(cfg['l1'], cfg['l2'], cfg['l3'], sv)
        return self.control_interface.set_LEDs_best_match_float_rgb(cfg['start_idx'], cfg['end_idx'], rgbs[0,0], rgbs[1, 0], rgbs[2, 0])

    def match_sense_vector(self, sv, brightness):
        for cfg in self.subdevice_configs:
            self.match_sense_vector_on_subdevice(cfg, sv, brightness)

    def match_planck_spectrum(self, color_temp, brightness, precompute = False):
        if brightness < 0. or brightness > 1.:
            raise ValueError('Invalid brightness value: ' + str(brightness))

        if precompute:
            tmp = deepcopy(self.control_interface.LEDs_state)
        sv = cm.sense_vector(cm.planck_spectrum(color_temp))
        self.match_sense_vector(sv, brightness)
        if precompute:
            res = deepcopy(self.control_interface.LEDs_state)
            self.control_interface.unsafe_raw_set_LEDs_state(tmp)
            return res

    def commit_all_off(self):
        self.control_interface.commit_off()

    def commit_all_on(self):
        self.control_interface.commit_state()

    def visual_to_absolute_brightness(self, b):
        x = (b + 0.16) / 1.16
        if x > 6./29.:
            return x**3.0
        else:
            return (x - 4./29.) / (1./3. * (29./6.)**2.0
