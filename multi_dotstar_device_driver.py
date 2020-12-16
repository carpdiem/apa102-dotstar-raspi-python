import Color_Match as cm
import apa102_dotstar_raspi as apa

class MultiDotstarController:
    def __init__(self, subdevice_configs, color_order, thermal_protection_limit = False, max_Hz = 3000000, bus = 0, device = 0, update = 'on-command'):
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
        self.control_interface = apa.DotstarDevice(self.total_num_LEDs, 'linear', color_order, thermal_protection_limit = thermal_protection_limit, max_Hz = max_Hz, bus = bus, device = device, update = update)

    def match_sense_vector_on_subdevice(self, cfg, sv, brightness):
        rgbs = brightness * cm.rgb_composition(cfg['l1'], cfg['l2'], cfg['l3'], sv)
        return self.control_interface.set_LEDs_best_match_float_rgb(cfg['start_idx'], cfg['end_idx'], rgbs[0,0], rgbs[1, 0], rgbs[2, 0])

    def match_sense_vector(self, sv, brightness):
        for cfg in self.subdevice_configs:
            self.match_sense_vector_on_subdevice(cfg, sv, brightness)

    def match_planck_spectrum(self, color_temp, brightness):
        if brightness < 0. or brightness > 1.:
            raise ValueError('Invalid brightness value: ' + str(brightness))

        sv = cm.sense_vector(cm.planck_spectrum(color_temp))
        self.match_sense_vector(sv, brightness)
