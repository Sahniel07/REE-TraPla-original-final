#!/usr/bin/env python3
"""Emits the LaTeX table bodies of Chapter 5 directly from the stored runs.

The tables are generated rather than transcribed, so a regenerated dataset
cannot fall out of step with the reported numbers.
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from analyze_scenarios import analyse, ORDER, NAMES, PRETTY, R_SAFE
import glob

ZT = {'merge': frozenset({'3'})}

def rows(directory, scen, fmts):
    zt = ZT.get(scen, frozenset())
    out = {}
    for tag in fmts:
        rec = {}
        for quant in ('withoutQuanti', 'withQuanti'):
            hits = glob.glob(os.path.join(directory, 'trajectories_*_%s_%s_poly.csv' % (tag, quant)))
            rec[quant] = analyse(hits[0], zt) if hits else None
        out[tag] = rec
    return out

def ref_cost(directory, scen, quant):
    zt = ZT.get(scen, frozenset())
    hits = glob.glob(os.path.join(directory, 'trajectories_*_64_%s_poly.csv' % quant))
    if not hits: return None
    r = analyse(hits[0], zt)
    return None if (r is None or r['nonfinite']) else r['cost']

def cell(r, ref):
    if r is None:            return '---', '---', '---', 'n/a'
    if r['nonfinite']:       return '---', '---', '---', 'n.f.'
    if r['path'] == 0.0:     return '%.2f' % r['cost'], '%.3f' % (r['cost']/ref if ref else float('nan')), '---', 'stalled'
    cv = 'yes' if r['min_sep'] < R_SAFE else 'no'
    return ('%.2f' % r['cost'], '%.3f' % (r['cost']/ref if ref else float('nan')),
            '%.3f' % r['min_sep'], cv)

def emit(directory, scen, fmts, label):
    data = rows(directory, scen, fmts)
    ro = ref_cost(directory, scen, 'withoutQuanti')
    rq = ref_cost(directory, scen, 'withQuanti')
    print('%% %s' % label)
    for tag in fmts:
        name = NAMES.get(tag) or PRETTY.get(tag, tag)
        if name.startswith('posit<'):
            n, es = name[6:-1].split(',')
            name = 'posit$\\langle%s,%s\\rangle$' % (n, es)
        a = cell(data[tag]['withoutQuanti'], ro)
        b = cell(data[tag]['withQuanti'], rq)
        print('    %-25s & %s & %s & %s & %s & %s & %s & %s & %s \\\\' %
              (name, a[0], a[1], a[2], a[3], b[0], b[1], b[2], b[3]))
    print()

if __name__ == '__main__':
    base = os.path.dirname(os.path.abspath(__file__))
    all_f = ['64','32','posit32es0','posit32es1','posit32es2','posit32es3','16','posit16es0','posit16es1','posit16es2','posit16es3','posit16es4']
    es2   = ['64', '32', 'posit32es2', '16', 'posit16es2', 'posit16es3']
    emit(os.path.join(base, 'saved/Intersection'), 'intersection', all_f, 'tab:matrix -- Intersection, all exponent sizes')
    emit(os.path.join(base, 'saved/Crossing'),     'crossing',     es2,   'tab:scenarios -- Crossing')
    emit(os.path.join(base, 'saved/Merge'),        'merge',        es2,   'tab:scenarios -- Merge')
