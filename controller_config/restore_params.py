"""
Restore strut axis parameters (axes 0-5) that the 2.13.1 firmware upgrade reset to defaults,
taking the values from a saved MCD file. Servo gains, filters, and feedforward are not touched,
so the EasyTune results are kept.

Shows the changes only:
    python controller_config/restore_params.py "C:/Users/s12idc/Documents/Automation1/MCD_APS_12ID_10082026.mcd"
Writes them to the controller configuration (takes effect after a controller reset):
    python controller_config/restore_params.py <mcd> --apply
"""
import argparse
import re
import zipfile

import automation1 as a1

HOST = 'at-hex-12id.xray.aps.anl.gov'
RESTORE = [
    'HomeType', 'HomeSetup', 'HomeOffset', 'HomeSpeed', 'HomeRampRate',
    'SoftwareLimitHigh', 'SoftwareLimitLow',
    'PrimaryEncoder0SineGain', 'PrimaryEncoder0SineOffset', 'PrimaryEncoder0CosineGain',
    'PrimaryEncoder0CosineOffset', 'PrimaryEncoder0Phase',
    'PositionErrorThreshold', 'VelocityCommandThreshold', 'AverageCurrentThreshold', 'AverageCurrentTime',
    'LimitDebounceDistance', 'DefaultAxisRampRate', 'DefaultAxisRampType',
]


def saved_values(mcd):
    xml = zipfile.ZipFile(mcd).read('config/Parameters').decode('utf-8', errors='replace')
    blocks = re.findall(r'<Axis[^>]*?(?:Index|index|i)="(\d+)"[^>]*>(.*?)</Axis>', xml, re.S)
    return {int(i): dict(re.findall(r'<P id="\d+" n="(\w+)">([^<]*)</P>', body)) for i, body in blocks}


def parameters_by_name(axis_params):
    out = {}
    for gname in dir(axis_params):
        if gname.startswith('_'):
            continue
        group = getattr(axis_params, gname)
        for pname in dir(group):
            if pname.startswith('_') or pname == 'group_name':
                continue
            p = getattr(group, pname)
            if hasattr(p, 'value') and hasattr(p, 'name'):
                out[p.name] = p
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('mcd')
    parser.add_argument('--apply', action='store_true', help='write the values to the controller configuration')
    args = parser.parse_args()

    saved = saved_values(args.mcd)
    c = a1.Controller.connect(host=HOST)
    conf = c.configuration.parameters.get_configuration()
    changes = 0
    for i in range(6):
        params = parameters_by_name(conf.axes[i])
        for name in RESTORE:
            if name not in saved.get(i, {}) or name not in params:
                continue
            new = float(saved[i][name])
            old = params[name].value
            if abs(new - old) > 1e-9 * max(1.0, abs(new)):
                print(f'axis {i} {name}: {old} -> {new}')
                params[name].value = new
                changes += 1
    if args.apply and changes:
        c.configuration.parameters.set_configuration(conf)
        print(f'wrote {changes} values; reset the controller to apply them')
    else:
        print(f'{changes} values differ' + ('' if args.apply else ' (dry run, nothing written)'))
    c.disconnect()


if __name__ == '__main__':
    main()
