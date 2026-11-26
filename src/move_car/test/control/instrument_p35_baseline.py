#!/usr/bin/env python3
"""Build an instrumented COPY of the frozen P23 solver; never edit the snapshot.

Only clock measurements/result fields change. Use a generous diagnostic budget
and report uninstrumented 8 ms runs separately, since phase clocks add overhead.
Usage: instrument_p35_baseline.py SNAPSHOT_INCLUDE OUTPUT_INCLUDE LIVE_INCLUDE
"""
import pathlib
import shutil
import sys

source, output, live = map(pathlib.Path, sys.argv[1:])
output.mkdir(parents=True, exist_ok=True)
shutil.copytree(str(source / 'control'), str(output / 'control'), dirs_exist_ok=True)
shutil.copy2(str(live / 'control/research_types.h'), str(output / 'control/research_types.h'))
p = output / 'control/reduced_nmpc.h'
s = p.read_text()

def replace(old, new, count=None):
    global s
    if old not in s:
        raise RuntimeError('frozen baseline anchor absent: ' + old[:80])
    s = s.replace(old, new) if count is None else s.replace(old, new, count)

replace('out.reason=why;out.solve_ms=milliseconds(start);return out;', '''out.reason=why;out.solve_ms=milliseconds(start);
      if(c.profile){const auto& p=out.profile;
        out.profile.other_ms=std::max(0.,out.solve_ms-p.initialization_ms-p.reference_ms-
          p.prediction_ms-p.constraint_cost_ms-p.direction_ms-p.candidate_selection_ms);}
      return out;''')
replace('    Rollout best;', '    if(c.profile)out.profile.initialization_ms=milliseconds(start);\n    Rollout best;', 1)
replace('rollout(x,r,c,deadline,expired)', 'rollout(x,r,c,deadline,expired,out.profile)')
replace('    Decision seed=makeSeed(r,c,0);Rollout candidate;', '''    auto seedFor=[&](int kind){
      auto tick=c.profile?Clock::now():Clock::time_point{};
      auto x=makeSeed(r,c,kind);
      if(c.profile)out.profile.direction_ms+=milliseconds(tick);return x;
    };
    Decision seed=seedFor(0);Rollout candidate;''')
for kind in (1, 2, 3):
    replace('makeSeed(r,c,%d)' % kind, 'seedFor(%d)' % kind)
replace('      Rollout roll;evaluate(x,roll);', '      Rollout roll;evaluate(x,roll);\n      auto select_tick=c.profile?Clock::now():Clock::time_point{};', 1)
replace('        best=roll;selected=x;out.warm_started=warm;\n      }', '        best=roll;selected=x;out.warm_started=warm;\n      }\n      if(c.profile)out.profile.candidate_selection_ms+=milliseconds(select_tick);', 1)
replace('          Decision x=base;x[j]=clamp(base[j]+sign*radius,-1.,1.);', '''          auto direction_tick=c.profile?Clock::now():Clock::time_point{};
          Decision x=base;x[j]=clamp(base[j]+sign*radius,-1.,1.);
          if(c.profile)out.profile.direction_ms+=milliseconds(direction_tick);''')
replace('          Rollout roll;evaluate(x,roll);', '          Rollout roll;evaluate(x,roll);\n          auto select_tick=c.profile?Clock::now():Clock::time_point{};')
replace('            best=roll;selected=x;\n          }', '            best=roll;selected=x;\n          }\n          if(c.profile)out.profile.candidate_selection_ms+=milliseconds(select_tick);')
replace('Clock::time_point deadline,bool& expired){', 'Clock::time_point deadline,bool& expired,SolveProfile& profile){')
replace('    roll.violation=stateViolation(s,s.acceleration,0.,cap,c);', '''    auto timer=c.profile?Clock::now():Clock::time_point{};
    roll.violation=stateViolation(s,s.acceleration,0.,cap,c);
    if(c.profile)profile.constraint_cost_ms+=milliseconds(timer);''')
replace('      double alo,ahi,dlo,dhi;', '      timer=c.profile?Clock::now():Clock::time_point{};\n      double alo,ahi,dlo,dhi;')
replace('        roll.violation=std::max(alo-ahi,dlo-dhi);return roll;', '''        roll.violation=std::max(alo-ahi,dlo-dhi);
        if(c.profile)profile.constraint_cost_ms+=milliseconds(timer);return roll;''')
replace('      roll.commands[i]=u;roll.command_accelerations[i]=a;', '''      roll.commands[i]=u;roll.command_accelerations[i]=a;
      if(c.profile)profile.constraint_cost_ms+=milliseconds(timer);''')
replace('        const double absolute=t+elapsed;', '        timer=c.profile?Clock::now():Clock::time_point{};\n        const double absolute=t+elapsed;')
replace('        const double acc=(uv.speed-s.speed)', '''        if(c.profile)profile.initialization_ms+=milliseconds(timer);
        timer=c.profile?Clock::now():Clock::time_point{};
        const double acc=(uv.speed-s.speed)''')
replace('        if(roll.violation>1e-7)return roll;\n        const double veq=', '''        if(c.profile)profile.constraint_cost_ms+=milliseconds(timer);
        if(roll.violation>1e-7)return roll;
        timer=c.profile?Clock::now():Clock::time_point{};
        const double veq=''', 1)
replace('        if(!validState(s)){roll.violation=std::numeric_limits<double>::infinity();return roll;}', '''        if(c.profile)profile.prediction_ms+=milliseconds(timer);
        timer=c.profile?Clock::now():Clock::time_point{};
        if(!validState(s)){roll.violation=std::numeric_limits<double>::infinity();
          if(c.profile)profile.constraint_cost_ms+=milliseconds(timer);return roll;}''')
replace('        if(roll.violation>1e-7)return roll;\n        elapsed+=sub;', '''        if(c.profile)profile.constraint_cost_ms+=milliseconds(timer);
        if(roll.violation>1e-7)return roll;
        elapsed+=sub;''')
replace('      const auto p=referenceAt(r.reference,t+h);', '''      timer=c.profile?Clock::now():Clock::time_point{};
      const auto p=referenceAt(r.reference,t+h);
      if(c.profile)profile.reference_ms+=milliseconds(timer);
      timer=c.profile?Clock::now():Clock::time_point{};''')
replace('      if(i==c.steps-1)objective+=.4*stage;', '''      if(i==c.steps-1)objective+=.4*stage;
      if(c.profile)profile.constraint_cost_ms+=milliseconds(timer);''')
p.write_text(s)
print(p)
