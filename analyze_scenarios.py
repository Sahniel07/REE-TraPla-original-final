#!/usr/bin/env python3
"""
Tabulates the metrics this thesis reports, for one scenario directory.

Reports, per the reference study: the accumulated cost l averaged over the
vehicles, the ratio l/l_fp64, the constraint violation c_v, the minimum
separation reached by any pair, and the path length. A run whose trajectory
contains a non-finite value is reported as such rather than scored, since a
NaN comparison silently passes the collision test.

  python3 analyze_scenarios.py saved/Crossing
"""
import csv, sys, math, glob, os, collections

R_SAFE = 2.5
ORDER = ['16', 'posit16es0', 'posit16es1', 'posit16es2', 'posit16es3',
         'posit16es4',
         '32', 'posit32es0', 'posit32es1', 'posit32es2', 'posit32es3', '64']
NAMES = {'16': 'fp16', '32': 'fp32', '64': 'fp64'}
PRETTY = {'posit16es0': 'posit<16,0>', 'posit16es1': 'posit<16,1>',
          'posit16es2': 'posit<16,2>', 'posit16es3': 'posit<16,3>',
          'posit16es4': 'posit<16,4>',
          'posit32es0': 'posit<32,0>', 'posit32es1': 'posit<32,1>',
          'posit32es2': 'posit<32,2>', 'posit32es3': 'posit<32,3>'}


def load(path):
    rows = list(csv.reader(open(path)))
    head = rows[0]
    ix, iy, it = head.index('x'), head.index('y'), head.index('time')
    iv, il = head.index('vehicle_id'), head.index('l')
    traj = collections.defaultdict(list)
    final_l = {}
    for row in rows[1:]:
        if len(row) <= max(ix, iy, it, iv, il):
            continue                      # malformed initial-state line
        try:
            traj[row[iv]].append((float(row[it]), float(row[ix]), float(row[iy])))
            final_l[row[iv]] = float(row[il])
        except ValueError:
            pass
    return traj, final_l


def analyse(path, zero_target=frozenset()):
    traj, final_l = load(path)
    if not traj:
        return None
    # A run whose positions are not finite produced no trajectory. Where the
    # cost accumulator nonetheless stayed finite, the accumulated cost is still
    # reported, because it is the value the solver returned and it is what the
    # per-run figures display. Such a run has no separation and no path, so it
    # is classified with the runs that did not move.
    finite_pos = all(math.isfinite(v) for p in traj.values() for _, x, y in p for v in (x, y))
    finite_cost = all(math.isfinite(v) for v in final_l.values())
    if not finite_pos and not finite_cost:
        return {'nonfinite': True}

    if not finite_pos:
        scored = {k: v for k, v in final_l.items() if k not in zero_target} or final_l
        return {'nonfinite': False,
                'cost': sum(scored.values()) / len(scored),
                'min_sep': math.inf,
                'path': 0.0}

    min_sep = math.inf
    ids = sorted(traj)
    for a in range(len(ids)):
        for b in range(a + 1, len(ids)):
            A = {round(t, 3): (x, y) for t, x, y in traj[ids[a]]}
            B = {round(t, 3): (x, y) for t, x, y in traj[ids[b]]}
            for t in set(A) & set(B):
                d = math.hypot(A[t][0] - B[t][0], A[t][1] - B[t][1])
                min_sep = min(min_sep, d)

    path_len = 0.0
    for p in traj.values():
        p = sorted(p)
        path_len += sum(math.hypot(p[i + 1][1] - p[i][1], p[i + 1][2] - p[i][2])
                        for i in range(len(p) - 1))

    # The accumulated cost is averaged over the vehicles that have a target
    # speed to reach. A vehicle whose target is zero contributes no
    # speed-tracking cost and would otherwise pull the mean down, so it is
    # excluded. Only Merge contains such a vehicle, its fourth participant.
    scored = {k: v for k, v in final_l.items() if k not in zero_target}
    if not scored:
        scored = final_l

    return {'nonfinite': False,
            'cost': sum(scored.values()) / len(scored),
            'min_sep': min_sep,
            'path': path_len}


def main(directory):
    scen = os.path.basename(directory.rstrip('/')).lower().replace(' ', '')
    for quant in ('withoutQuanti', 'withQuanti'):
        files = {}
        for tag in ORDER:
            hits = glob.glob(os.path.join(directory, f'trajectories_*_{tag}_{quant}_poly.csv'))
            if hits:
                files[tag] = hits[0]
        if not files:
            continue
        zt = frozenset({'3'}) if scen == 'merge' else frozenset()
        ref = analyse(files['64'], zt) if '64' in files else None
        ref_cost = ref['cost'] if ref and not ref['nonfinite'] else None

        print(f'=== {scen} / {quant} ===')
        print('%-14s %10s %10s %10s %10s  %s' %
              ('format', 'cost l', 'l/l_fp64', 'min_sep', 'path', 'c_v'))
        for tag in ORDER:
            if tag not in files:
                continue
            name = NAMES.get(tag, PRETTY.get(tag, tag))
            r = analyse(files[tag], zt)
            if r is None:
                print('%-14s %10s %10s %10s %10s  %s' % (name, '-', '-', '-', '-', 'no data'))
                continue
            if r['nonfinite']:
                print('%-14s %10s %10s %10s %10s  %s' % (name, '-', '-', '-', '-', 'non-finite'))
                continue
            ratio = r['cost'] / ref_cost if ref_cost else float('nan')
            cv = 'yes' if r['min_sep'] < R_SAFE else 'no'
            if r['path'] == 0.0:
                cv = 'stalled'
            print('%-14s %10.2f %10.4f %10.3f %10.2f  %s' %
                  (name, r['cost'], ratio, r['min_sep'], r['path'], cv))
        print()


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else 'saved/Intersection')
