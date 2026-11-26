#!/usr/bin/env python2
"""Operator-held enable heartbeat. Never publishes a chassis command.

Stopping this process removes the P7 lease; the independent controller then
publishes zero speed. This is not a replacement for the hardware emergency stop.
"""
from __future__ import print_function
import argparse
import hashlib
import json
import os
import sys
import time
import threading
try:
 from xmlrpc.client import ServerProxy,Transport
except ImportError:
 from xmlrpclib import ServerProxy,Transport


def sha(path):
 h=hashlib.sha256()
 with open(path,'rb') as f:
  for block in iter(lambda:f.read(1048576),b''):h.update(block)
 return h.hexdigest()


class BoundedTransport(Transport):
 def make_connection(self,host):
  c=Transport.make_connection(self,host);c.timeout=.08;return c


def capture_health_status(health, run_id, now_ns):
 """Explain the same fail-closed recording contract; never repair lost events."""
 age=(now_ns-health.get('steady_ns',0))*1e-9
 w=health.get('writer',{});counts=health.get('topic_event_counts',{})
 required=('/cmd_vel','/car_message','/imu','/control_research/p5/published',
     '/control_research/p5_test/p6/request','/control_research/p5_test/p6/candidate')
 missing=[topic for topic in required if not counts.get(topic,0)>0]
 reasons=[]
 if health.get('run_id')!=run_id:reasons.append('capture_run_id_mismatch')
 if health.get('state')!='recording':reasons.append('capture_not_recording')
 if not 0.<=age<=2.:reasons.append('capture_health_stale_or_future')
 if w.get('writer_error'):reasons.append('capture_writer_error')
 if w.get('queue_dropped'):reasons.append('capture_queue_dropped')
 if missing:reasons.append('capture_required_topics_missing')
 return dict(valid=not reasons,reason=reasons[0] if reasons else 'ok',reasons=reasons,
     age_s=age,state=health.get('state'),run_id=health.get('run_id'),missing_topics=missing,
     queue_depth=w.get('depth'),queue_capacity=w.get('capacity'),
     queue_dropped=w.get('queue_dropped'),dropped_by_topic=w.get('dropped_by_topic',{}),
     writer_error=w.get('writer_error'))


def capture_health_valid(health, run_id, now_ns):
 return capture_health_status(health,run_id,now_ns)['valid']


def controller_status(master, caller_id, expected_digest):
 """Read-only graph/binary check, with a distinct cause for each rejection."""
 code,message,graph=master.getSystemState(caller_id)
 if code!=1:return dict(valid=False,reason='publisher_graph_unavailable',error=message)
 publishers=dict(graph[0]).get('/cmd_vel',[])
 status=dict(valid=False,publishers=publishers)
 if publishers!=['/publish_speed_turn']:
  status['reason']='cmd_vel_publishers_changed';return status
 code,message,uri=master.lookupNode(caller_id,'/publish_speed_turn')
 if code!=1:
  status.update(reason='controller_node_unavailable',error=message);return status
 code,message,pid=ServerProxy(uri,transport=BoundedTransport()).getPid(caller_id)
 if code!=1:
  status.update(reason='controller_pid_unavailable',error=message);return status
 actual=sha('/proc/%d/exe'%pid)
 status.update(valid=actual==expected_digest,reason='ok' if actual==expected_digest else 'controller_binary_changed',
     pid=pid,binary_sha256=actual)
 return status


def authorization_status(controller_check, capture_check):
 # Do not continue heartbeats across an RPC error or recorder failure.
 try:controller=controller_check()
 except Exception as exc:
  return dict(valid=False,reason='controller_check_error',error=str(exc))
 if not controller['valid']:
  return dict(valid=False,reason=controller['reason'],controller=controller)
 try:capture=capture_check()
 except Exception as exc:
  return dict(valid=False,reason='capture_health_read_error',error=str(exc),controller=controller)
 return dict(valid=capture['valid'],reason=capture['reason'],controller=controller,capture=capture)


def update_computation_summary(summary, message, run_id):
 """Observed worker cycles since lease start; never a control-authority flag."""
 base=message.base
 if base.run_id!=run_id or base.cycle<=summary['last_cycle']:return False
 summary['last_cycle']=base.cycle
 summary['received']+=1
 summary['rollout_cycles']+=int(base.evaluations>0)
 summary['valid']+=int(base.valid)
 summary['feasible']+=int(base.feasible)
 summary['timed_out']+=int(base.timed_out)
 summary['mode']=message.computation_mode
 if base.evaluations>0:summary['decision_dimension']=base.decision_dimension
 summary['last_reason']=base.reason.split(';',1)[0]
 return True


def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--trial',required=True)
 p.add_argument('--output',help='new lease audit JSONL; defaults trial/operator_lease_<wall>.jsonl')
 a=p.parse_args();folder=os.path.realpath(a.trial)
 with open(os.path.join(folder,'trial.json')) as f:trial=json.load(f)
 if (not trial.get('run_id','').startswith('p7real_') or trial.get('terrain')!='smooth'
     or not trial.get('engineering_mapping_confirmed') or not 0.<trial.get('speed_cap',0.)<=.40):
  raise ValueError('invalid supervised trial')
 workspace=os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
 # scripts is package/scripts; workspace is two ancestors above the package.
 workspace=os.path.dirname(workspace)
 expected=trial.get('binary_paths',{}).get('control_pub',os.path.join(workspace,'devel/lib/move_car/control_pub'))
 if sha(expected)!=trial['binaries']['control_pub']:raise ValueError('binary changed; prepare a new trial')
 for rel,digest in trial['trial_files'].items():
  path=os.path.realpath(os.path.join(folder,rel))
  if not path.startswith(folder+os.sep) or sha(path)!=digest:raise ValueError('trial changed: '+rel)
 raw=os.path.join(folder,'raw',trial['run_id'],'manifest.json')
 if not os.path.isfile(raw):raise ValueError('start raw recording before enabling vehicle movement')
 import rospy
 from std_msgs.msg import String
 from move_car.msg import P5PublishedCommand,P6ControlCandidate
 from control_research.recording import monotonic_ns
 rospy.init_node('p7_operator_lease',anonymous=True)
 master=ServerProxy(os.environ.get('ROS_MASTER_URI','http://localhost:11311'),transport=BoundedTransport())
 def capture_status():
  with open(os.path.join(folder,'raw',trial['run_id'],'capture_health.json')) as f:health=json.load(f)
  return capture_health_status(health,trial['run_id'],monotonic_ns())
 def check():
  return authorization_status(lambda:controller_status(master,rospy.get_name(),trial['binaries']['control_pub']),capture_status)
 initial=check()
 if not initial['valid']:raise ValueError('operator enable refused: '+json.dumps(initial,sort_keys=True))
 topic='/control_research/p7/operator_lease'
 publisher=rospy.Publisher(topic,String,queue_size=1,latch=False,tcp_nodelay=True)
 # Observe the final publication boundary, not worker feasibility. This makes
 # a shadow-only or persistently rejected run visible to the operator in situ.
 ownership=dict(committed=0,nmpc=0,legacy=0,sequence=0,owner='waiting',reason='waiting_for_publication')
 computation=dict(received=0,rollout_cycles=0,valid=0,feasible=0,timed_out=0,last_cycle=0,
     mode='waiting',decision_dimension=0,last_reason='waiting_for_candidate',scope='observed_since_lease_start')
 ownership_lock=threading.Lock()
 def published(m):
  if m.run_id!=trial['run_id'] or not m.publish_returned or not m.history_committed:return
  with ownership_lock:
   if m.sequence<=ownership['sequence']:return
   ownership['sequence']=m.sequence;ownership['committed']+=1
   ownership['owner']=m.owner;ownership['reason']=m.reason
   if m.owner=='nmpc':ownership['nmpc']+=1
   elif m.owner=='legacy_slqr':ownership['legacy']+=1
 subscriber=rospy.Subscriber('/control_research/p5/published',P5PublishedCommand,published,queue_size=128,tcp_nodelay=True)
 log=a.output or os.path.join(folder,'operator_lease_%d.jsonl'%int(time.time()*1e9))
 fd=os.open(log,os.O_CREAT|os.O_EXCL|os.O_WRONLY,0o644)
 def computed(m):
  with ownership_lock:update_computation_summary(computation,m,trial['run_id'])
 candidate_subscriber=rospy.Subscriber('/control_research/p5_test/p6/candidate',P6ControlCandidate,computed,queue_size=64)
 if trial.get('shadow_only'):
  print('MODE: S8 publishes vehicle commands; NMPC computes shadow candidates. NMPC_selected stays zero by design.')
 sequence=monotonic_ns();sent=0;last_ownership_print=0
 print('OPERATOR ENABLE ACTIVE: cap %.3f m/s; Ctrl-C removes lease and requests P7 stop.'%trial['speed_cap'])
 try:
  with os.fdopen(fd,'w') as stream:
   while not rospy.is_shutdown():
    start=monotonic_ns();row=dict(steady_ns=start,run_id=trial['run_id'],sent=False)
    status=check()
    if not status['valid']:
     row.update(reason=status['reason'],watchdog=status)
     stream.write(json.dumps(row)+'\n');stream.flush()
     print('LEASE STOPPED: '+json.dumps(status,sort_keys=True));break
    sequence+=1;publisher.publish(String(data=trial['run_id']+'|'+str(sequence)));sent+=1
    capture=status['capture']
    with ownership_lock:
     current_ownership=dict(ownership);current_computation=dict(computation)
    row.update(sent=True,sequence=sequence,capture_age_s=capture['age_s'],
        capture_queue_depth=capture['queue_depth'],capture_queue_dropped=capture['queue_dropped'],
        published_control=current_ownership,nmpc_computation=current_computation)
    stream.write(json.dumps(row)+'\n')
    if start-last_ownership_print>=1000000000:
     print('CONTROL actual_owner=%s; NMPC_selected=%d / committed=%d; S8=%d; reason=%s'%(
       current_ownership['owner'],current_ownership['nmpc'],current_ownership['committed'],
       current_ownership['legacy'],current_ownership['reason']))
     print('NMPC observed_mode=%s; candidates=%d; rollout_cycles=%d; valid=%d; feasible=%d; timed_out=%d; last_solve_variables=%d; reason=%s'%(
       current_computation['mode'],current_computation['received'],current_computation['rollout_cycles'],
       current_computation['valid'],current_computation['feasible'],current_computation['timed_out'],
       current_computation['decision_dimension'],current_computation['last_reason']))
     sys.stdout.flush();last_ownership_print=start
    if sent%20==0:stream.flush()
    delay=.05-(monotonic_ns()-start)*1e-9
    if delay>0.:time.sleep(delay)
 except (KeyboardInterrupt,rospy.ROSInterruptException):pass
 finally:
  candidate_subscriber.unregister()
  subscriber.unregister()
  publisher.unregister()
  print('LEASE REMOVED. P7 must reach zero within lease expiry plus control/publication delay; verify physical stop.')


if __name__=='__main__':main()
