#!/usr/bin/env python2
"""Passive P5 event recorder / Python3 offline audit. Never publishes commands."""
from __future__ import print_function
import argparse,base64,json,os,time,threading,sys
import ctypes
class Timespec(ctypes.Structure):
 _fields_=[("sec",ctypes.c_long),("nsec",ctypes.c_long)]
_clock=ctypes.CDLL("librt.so.1",use_errno=True).clock_gettime
def monotonic():
 t=Timespec()
 if _clock(1,ctypes.byref(t))!=0:raise OSError("CLOCK_MONOTONIC unavailable")
 return t.sec+t.nsec*1e-9
try:import queue
except ImportError:import Queue as queue

def audit(path):
 records=[];received=[];errors=[]
 with open(path) as f:
  for line in f:
   row=json.loads(line);received.append(row['receipt_sequence'])
   if row.get('type')=='move_car/P5PublishedCommand' and row.get('decoded'):
    r=row['decoded']
    if r.get('physical_execution_confirmed'):errors.append('false_execution_claim')
    if r.get('publish_returned') and r.get('history_committed'):records.append(r)
 seq=[r['sequence'] for r in records];missing=[];duplicate=[]
 for a,b in zip(seq,seq[1:]):
  if b<=a:duplicate.append(b)
  elif b>a+1:missing.append([a+1,b-1])
 return dict(events=len(received),published_records=len(records),missing_published_ranges=missing,duplicate_or_out_of_order=duplicate,receipt_gaps=[[a+1,b-1] for a,b in zip(received,received[1:]) if b>a+1],errors=errors,observed_command_history_contiguous=bool(seq) and not(missing or duplicate or errors),strict_model_validation_allowed=False,strict_validation_reason='P5 events alone cannot certify raw sensor timing, calibration or independent-session vehicle model validation',coverage='observed recording interval only; no pre-start or post-stop completeness claim; ROS return is not physical execution')

class Writer(object):
 def __init__(self,path,capacity=2048,decoder=None):
  self.lock=threading.Lock();self.queue=queue.Queue(capacity);self.received=0;self.dropped=0;self.written=0;self.error=None;self.path=path;self.decoder=decoder;self.thread=threading.Thread(target=self.run);self.thread.daemon=True;self.thread.start()
 def put(self,row):
  with self.lock:
   self.received+=1;row['receipt_sequence']=self.received
   try:self.queue.put_nowait(row)
   except queue.Full:self.dropped+=1
 def run(self):
  try:
   with open(self.path,'x' if sys.version_info[0]>=3 else 'w') as f:
    while True:
     row=self.queue.get()
     if row is None:break
     data=row.pop('raw_bytes',b'');row['serialized_base64']=base64.b64encode(data).decode('ascii')
     if self.decoder:
      try:row['decoded']=self.decoder(row['type'],data)
      except Exception as e:row['decode_error']=repr(e)
     f.write(json.dumps(row,sort_keys=True)+'\n');self.written+=1
    f.flush();os.fsync(f.fileno())
  except Exception as e:self.error=repr(e)
 def close(self):
  # Offline shutdown only; control callback never joins this thread.
  while self.thread.is_alive():
   try:self.queue.put(None,timeout=.1);break
   except queue.Full:pass
  self.thread.join(10.)
  return dict(received=self.received,written=self.written,queue_dropped=self.dropped,writer_error=self.error,clean_shutdown=not self.thread.is_alive(),unwritten=self.received-self.written-self.dropped)

def main():
 p=argparse.ArgumentParser(description=__doc__);sub=p.add_subparsers(dest='op');r=sub.add_parser('record');r.add_argument('--output',required=True);r.add_argument('--run-id',required=True);r.add_argument('--session-id',required=True);r.add_argument('--capacity',type=int,default=2048);r.add_argument('--terrain',choices=('smooth','grass_bumpy','asphalt','unknown'),default='unknown');r.add_argument('--data-origin',choices=('real','synthetic','unknown'),default='unknown');r.add_argument('--p6',action='store_true',help='also retain P6 atomic requests/candidates and independent raw sensor events');a=sub.add_parser('audit');a.add_argument('events');a.add_argument('--output',required=True);args=p.parse_args()
 if args.op=='audit':
  if os.path.exists(args.output):raise ValueError('output exists')
  with open(args.output,'w') as f:json.dump(audit(args.events),f,indent=2)
  return
 if args.op!='record':p.error('choose record or audit')
 if not args.run_id.startswith('p5_') or not args.session_id or args.capacity<1:raise ValueError('explicit P5 run/session and bounded queue required')
 if os.path.exists(args.output):raise ValueError('new output directory required')
 os.makedirs(args.output)
 import rospy,roslib.message
 rospy.init_node('p5_passive_event_recorder',anonymous=True)
 def encode(v):
  if hasattr(v,'secs') and hasattr(v,'nsecs'):return dict(secs=v.secs,nsecs=v.nsecs)
  if hasattr(v,'__slots__'):return dict((k,encode(getattr(v,k))) for k in v.__slots__)
  if isinstance(v,(tuple,list)):return [encode(x) for x in v]
  return v
 def decode(typ,data):
  cls=roslib.message.get_message_class(typ)
  if cls is None:raise ValueError('unknown message class')
  return encode(cls().deserialize(data))
 writer=Writer(os.path.join(args.output,'events.jsonl'),args.capacity,decode)
 metadata=dict(schema_version=1,run_id=args.run_id,session_id=args.session_id,terrain=args.terrain,data_origin=args.data_origin,started_wall=time.time(),passive=True,hardware_sample_time_claim=False,control_authority=False,pid=os.getpid(),topics=[],p6_enabled=args.p6)
 subscribers=[]
 topics=['/control_research/p5/'+topic for topic in ('published','diagnostics','candidate','request')]
 if args.p6:topics+=['/control_research/p5_test/p6/request','/control_research/p5_test/p6/candidate','/car_message','/imu','/carvel','/planner/reference_path','/planner/reference_status','/control_research/p5_test/command']
 for full in topics:
  metadata['topics'].append(full)
  def callback(msg,topic=full):
   header=msg._connection_header or {};writer.put(dict(topic=topic,receipt_wall=time.time(),receipt_ros=rospy.Time.now().to_sec(),receipt_monotonic=monotonic(),type=header.get('type'),publisher=header.get('callerid'),md5sum=header.get('md5sum'),raw_bytes=msg._buff))
  subscribers.append(rospy.Subscriber(full,rospy.AnyMsg,callback,queue_size=2048,buff_size=4*1024*1024))
 try:rospy.spin()
 finally:
  for s in subscribers:s.unregister()
  metadata.update(writer.close());metadata['ended_wall']=time.time();metadata['audit']=audit(os.path.join(args.output,'events.jsonl')) if metadata['clean_shutdown'] and not metadata['writer_error'] else None
  with open(os.path.join(args.output,'manifest.json'),'w') as f:json.dump(metadata,f,indent=2)
if __name__=='__main__':main()
