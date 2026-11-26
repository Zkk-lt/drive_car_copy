"""Loss-aware passive recording primitives. No ROS imports, Python 2/3."""
from __future__ import division
import base64
import ctypes
import io
import json
import math
import threading
import time
try:
    import queue
except ImportError:
    import Queue as queue

class Timespec(ctypes.Structure):
    _fields_ = [('seconds', ctypes.c_long), ('nanoseconds', ctypes.c_long)]
_clock = ctypes.CDLL('librt.so.1', use_errno=True).clock_gettime
_clock.argtypes = [ctypes.c_int, ctypes.POINTER(Timespec)]

def monotonic_ns():
    t = Timespec()
    if _clock(1, ctypes.byref(t)):
        raise OSError('clock_gettime(CLOCK_MONOTONIC) failed')
    return t.seconds * 1000000000 + t.nanoseconds

def plain(value):
    if hasattr(value, '__slots__'):
        return {k:plain(getattr(value,k)) for k in value.__slots__}
    if isinstance(value, (tuple, list)):
        return [plain(v) for v in value]
    if isinstance(value, float) and (math.isnan(value) or math.isinf(value)):
        return {'nonfinite':str(value)}  # original IEEE bits are preserved in ros_serialized_b64
    return value

def geometry_payload(msg):
    """Exact JSON projection of geometry, avoiding per-scalar slot reflection.

    This changes neither the raw serialization nor which poses are retained.
    Nonfinite IEEE values keep the same explicit sentinel as ``plain``.
    """
    def number(v):
        if isinstance(v, float) and (math.isnan(v) or math.isinf(v)):
            return {'nonfinite':str(v)}
        return v
    def header(h):
        return {'seq':h.seq,'stamp':{'secs':h.stamp.secs,'nsecs':h.stamp.nsecs},'frame_id':h.frame_id}
    def pose(p):
        return {'position':{'x':number(p.position.x),'y':number(p.position.y),'z':number(p.position.z)},
                'orientation':{'x':number(p.orientation.x),'y':number(p.orientation.y),
                               'z':number(p.orientation.z),'w':number(p.orientation.w)}}
    kind=getattr(msg,'_type','')
    if kind=='nav_msgs/Path':
        return {'header':header(msg.header),
                'poses':[{'header':header(p.header),'pose':pose(p.pose)} for p in msg.poses]}
    if kind=='geometry_msgs/PoseArray':
        return {'header':header(msg.header),'poses':[pose(p) for p in msg.poses]}
    return plain(msg)

def quality(msg, receipt_ns):
    header = getattr(msg, 'header', None)
    stamp = header.stamp.to_nsec() if header is not None else None
    reasons = []
    if not stamp:
        reasons.append('missing_source_stamp')
    elif stamp > receipt_ns + 50000000:
        reasons.append('future_source_stamp')
    elif receipt_ns - stamp > 300000000:
        reasons.append('source_older_than_0p3s')
    frame = getattr(header, 'frame_id', '')
    if header is not None and not frame:
        reasons.append('missing_frame')
    if getattr(msg, '_type', '') == 'nav_msgs/Odometry':
        q = msg.pose.pose.orientation
        norm = q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w
        if not 0.9 <= norm <= 1.1:
            reasons.append('invalid_pose_quaternion')
        if not msg.child_frame_id:
            reasons.append('missing_child_frame')
        reasons.append('ground_speed_independence_unverified')
    if getattr(msg, '_type', '') in ('move_car/car_parameter','geometry_msgs/Twist'):
        reasons.append('receipt_time_only')
    # Acquisition validity is not an estimator confidence or a calibration certificate.
    return stamp, frame, reasons

class AsyncJsonlWriter(object):
    def __init__(self, path, capacity=1024):
        self.stream = open(path, 'w')
        self.queue = queue.Queue(maxsize=capacity)
        self.lock = threading.Lock()
        self.accepting = True
        self.stop = threading.Event()
        self.accepted = self.written = self.dropped = 0
        self.capacity = capacity
        self.max_observed_depth = 0
        self.attempted_by_topic = {}
        self.dropped_by_topic = {}
        self.written_by_topic = {}
        self.error = None
        self.thread = threading.Thread(target=self._write)
        self.thread.daemon = True
        self.thread.start()

    def submit(self, item, topic=None):
        with self.lock:
            label=topic if topic is not None else '<unspecified>'
            self.attempted_by_topic[label]=self.attempted_by_topic.get(label,0)+1
            if not self.accepting or self.error:
                self.dropped += 1
                self.dropped_by_topic[label]=self.dropped_by_topic.get(label,0)+1
                return False
            try:
                self.queue.put_nowait(item)
                self.accepted += 1
                self.max_observed_depth=max(self.max_observed_depth,self.queue.qsize())
                return True
            except queue.Full:
                self.dropped += 1
                self.dropped_by_topic[label]=self.dropped_by_topic.get(label,0)+1
                return False

    def queue_health(self):
        with self.lock:
            return {'depth':self.queue.qsize(),'capacity':self.capacity,
                    'max_observed_depth':self.max_observed_depth,'queue_dropped':self.dropped,
                    'dropped_by_topic':dict(self.dropped_by_topic),'writer_error':self.error}

    def _write(self):
        try:
            while not self.stop.is_set() or not self.queue.empty():
                try:
                    item = self.queue.get(timeout=0.1)
                except queue.Empty:
                    continue
                if callable(item):
                    item = item()
                # Raw events have no canonical-key-order contract.  On Python 2,
                # sort_keys=True disables the stdlib C encoder and serializes
                # large Path/P6 messages in Python, starving this finite queue.
                # Preserve every value/event and the actual-file SHA contract;
                # only dictionary key order changes for newly written files.
                self.stream.write(json.dumps(item, sort_keys=False, allow_nan=False)+'\n')
                self.written += 1
                label=item.get('topic','<unspecified>') if isinstance(item,dict) else '<unspecified>'
                self.written_by_topic[label]=self.written_by_topic.get(label,0)+1
                if self.written % 30 == 0:
                    self.stream.flush()
            self.stream.flush()
        except Exception as exc:
            self.error = repr(exc)
        finally:
            try:
                self.stream.close()
            except Exception as exc:
                self.error = self.error or repr(exc)

    def close(self):
        with self.lock:
            self.accepting = False
        self.stop.set()
        self.thread.join()
        return {'accepted':self.accepted, 'written':self.written, 'queue_dropped':self.dropped,
                'writer_error':self.error, 'unwritten':self.accepted-self.written,
                'capacity':self.capacity,'max_observed_depth':self.max_observed_depth,
                'attempted_by_topic':dict(self.attempted_by_topic),
                'dropped_by_topic':dict(self.dropped_by_topic),'written_by_topic':dict(self.written_by_topic),
                'durability':'buffered writes; fsync not guaranteed; writer_error invalidates completeness',
                'transport_drops':'unknown; ROS subscriber queue loss is not counted here'}

def event_record(msg, topic, run_id, receipt_ns, wall_ns, steady_ns):
    source, frame, reasons = quality(msg, receipt_ns)
    raw = io.BytesIO();msg.serialize(raw)
    # Cloud binary is already present in the exact serialization; do not expand bytes to JSON.
    payload = geometry_payload(msg) if getattr(msg,'_type','') != 'sensor_msgs/PointCloud2' else {'binary_only':True}
    return {'schema_version':1,'run_id':run_id,'topic':topic,'ros_type':msg._type,
            'md5sum':msg._md5sum,'source_stamp_ns':source,'receipt_ros_ns':receipt_ns,
            'receipt_wall_ns':wall_ns,'receipt_monotonic_ns':steady_ns,'frame_id':frame,
            'validity':{'raw_preserved':True,'usable_for_state_estimation':False,
                        'confidence':0.,'reasons':reasons+['calibration_and_time_sync_not_verified']},
            'publisher':getattr(msg,'_connection_header',{}).get('callerid','unknown'),
            'payload':payload,'ros_serialized_b64':base64.b64encode(raw.getvalue()).decode('ascii')}
