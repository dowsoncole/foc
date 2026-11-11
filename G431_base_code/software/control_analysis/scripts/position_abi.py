"""Resolve position ABI from the exact ELF; never guess debug read lengths."""
import re
import subprocess


def position_symbols(image, gdb):
    objects = dict(mpt='g_mpt_t485', position='g_output_position',
                   trajectory='g_output_position_test', trace='g_output_position_trace',
                   frd='g_output_position_sweep_frd')
    # New customer mode overlays trace/results in one union; old frozen ELFs
    # still have the standalone trace symbol. Resolve either exact ABI.
    for trace in ('g_output_position_capture.trace', 'g_output_position_trace'):
        objects['trace'] = trace
        if trace.endswith('.trace'):
            objects['customer'] = 'g_output_position_customer'
        else:
            objects.pop('customer', None)
        expressions = [(key, '&'+obj) for key, obj in objects.items()]
        expressions += [(key+'_size', 'sizeof('+obj+')') for key, obj in objects.items()]
        cmd = [str(gdb), '-batch', str(image)]
        for _, expr in expressions:
            cmd += ['-ex', 'p/x '+expr]
        r = subprocess.run(cmd, capture_output=True, text=True)
        vals = re.findall(r'^\$\d+ = (0x[0-9a-fA-F]+)\s*$', r.stdout, re.M)
        if not r.returncode and len(vals) == len(expressions):
            return dict(zip((key for key, _ in expressions), (int(x, 16) for x in vals)))
    raise RuntimeError(r.stdout+r.stderr)
