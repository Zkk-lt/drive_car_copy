#!/usr/bin/env python3
"""Compile frozen/current solvers and compare exact same 240 candidate rollouts."""
import argparse,json,pathlib,subprocess,math
p=argparse.ArgumentParser();p.add_argument('--before-include',required=True)
p.add_argument('--output',default='/tmp/p35_rollout_equivalence');a=p.parse_args()
root=pathlib.Path(__file__).resolve().parents[2];output=pathlib.Path(a.output);output.mkdir(parents=True,exist_ok=True)
source=root/'test/control/compare_p35_rollouts.cpp'
results=[]
for name,inc,defs in [('before',a.before_include,[]),('after',str(root/'include'),['-DP35_OPTIMIZED'])]:
 executable=output/name
 command=['g++','-std=c++14','-O2','-fno-access-control','-I'+inc]+defs+[str(source),'-o',str(executable)]
 subprocess.check_call(command)
 data=subprocess.check_output([str(executable)],universal_newlines=True)
 (output/(name+'.txt')).write_text(data)
 results.append([[float(x) for x in row.split()] for row in data.splitlines()])
assert len(results[0])==len(results[1])==240
maximum=0.;finite_pairs=0
for a,b in zip(*results):
 assert len(a)==len(b)
 for x,y in zip(a,b):
  if not math.isfinite(x) or not math.isfinite(y):assert x==y
  else:
   maximum=max(maximum,abs(x-y));finite_pairs+=1
   assert abs(x-y)<=1e-12,'candidate rollout changed: %r vs %r'%(x,y)
report={'candidates':240,'finite_values_compared':finite_pairs,'feasible_candidates':sum(int(r[1]) for r in results[1]),'max_abs_difference':maximum,'tolerance':1e-12,'status':'PASS'}
(output/'result.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
