
from collections import namedtuple
from enum import IntEnum
from dataclasses import dataclass
import dataclasses


class Msg(IntEnum):
    PING        = 10
    HEARTBEAT   = 11
    BATTERY     = 12
    CALIBRATION = 13
    IMU         = 14
    SATNAV      = 15
    POSE        = 16
    COMMAND     = 17

    @staticmethod
    def str(val):
        if (val == Msg.PING): return "PING"
        elif (val == Msg.HEARTBEAT): return "HEARTBEAT"
        elif (val == Msg.BATTERY): return "BATTERY"
        elif (val == Msg.CALIBRATION): return "CALIBRATION"
        elif (val == Msg.IMU): return "IMU"
        elif (val == Msg.SATNAV): return "SATNAV"
        elif (val == Msg.POSE): return "POSE"
        return "UNKNOWN"


class Base:
    def flatten(self, data):
        if isinstance(data, tuple):
            for x in data:
                yield from self.flatten(x)
        else:
            yield data
    def serialize(self):
        return tuple(self.flatten(dataclasses.astuple(self)))

@dataclass(frozen=True)
class vec_t(Base):
    x: float
    y: float
    z: float

    def __yivo__(self):
        # (fmt, size, name, id)
        # name_t.__class__ already has name, do I need it again?
        return ("3f", 12, vec_t)

@dataclass(frozen=True)
class quat_t(Base):
    w: float
    x: float
    y: float
    z: float

    def __yivo__(self):
        # (fmt, size, name, id)
        # name_t.__class__ already has name, do I need it again?
        return ("4f", 16, quat_t)


@dataclass(frozen=True)
class date_t(Base):
    year: int # 2
    mon: int # 1
    day: int # 1

    def __yivo__(self):
        # (fmt, size, name, id)
        # name_t.__class__ already has name, do I need it again?
        return ("4B", 4, date_t)

@dataclass(frozen=True)
class clock_time_t(Base):
    hr: int # 1
    min: int # 1
    sec: float # 4

    def __yivo__(self):
        # (fmt, size, name, id)
        # name_t.__class__ already has name, do I need it again?
        return ("BBf", 6, clock_time_t)


@dataclass(frozen=True)
class pose_t(Base):
    position: vec_t # 12
    velocity: vec_t # 12
    orientation: quat_t # 16

    def __init__(self,px,py,pz,vx,vy,vz,w,x,y,z):
        object.__setattr__(self, "position", vec_t(px,py,pz))
        object.__setattr__(self, "velocity", vec_t(vx,vy,vz))
        object.__setattr__(self, "orientation", quat_t(w,x,y,z))

    def __yivo__(self):
        # (fmt, size, name, id)
        # name_t.__class__ already has name, do I need it again?
        return ("10f", 40, pose_t, Msg.POSE)

@dataclass(frozen=True)
class gps_t(Base):
    lat: float
    lon: float
    altitude: float
    # hdop: float
    # satellites: int
    # fix: int
    # year: int
    # month: int
    # day: int
    # hour: int
    # min: int
    # sec: int

    def __yivo__(self):
        # (fmt, size, name, id)
        # name_t.__class__ already has name, do I need it again?
        return ("3f", 12, gps_t, Msg.SATNAV)

@dataclass(frozen=True)
class imu_t(Base):
    a: vec_t
    g: vec_t
    m: vec_t
    pressure: float
    temperature: float
    timestamp: int

    def __init__(self,ax,ay,az,gx,gy,gz,mx,my,mz,p,t,ts):
        object.__setattr__(self, "a", vec_t(ax,ay,az))
        object.__setattr__(self, "g", vec_t(gx,gy,gz))
        object.__setattr__(self, "m", vec_t(mx,my,mz))
        object.__setattr__(self, "pressure", p)
        object.__setattr__(self, "temperature", t)
        object.__setattr__(self, "timestamp", ts)

    def __yivo__(self):
        # (fmt, size, name, id)
        # name_t.__class__ already has name, do I need it again?
        return ("11fI", 48, imu_t, Msg.IMU)


@dataclass(frozen=True)
class command_t(Base):
    # STANDBY = 1
    # ARM = 2
    # DATA_STREAM_ON = 4
    # DATA_STREAM_OFF = 8
    command: int

    def __yivo__(self):
        return ("B", 1, command_t, MSG.COMMAND)


