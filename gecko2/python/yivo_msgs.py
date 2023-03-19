from collections import namedtuple
from struct import Struct
from enum import IntEnum

# MSG_DISTANCE    = 20
# MSG_MOTORS_4    = 30
# MSG_IMU_FULL    = 42

class MSG(IntEnum):
    DISTANCE    = 20
    MOTORS_4    = 30
    IMU_FULL    = 42

# IMU = namedtuple("IMU","id ts ax ay az gx gy gz mx my mz qw qx qy qz p t")
IMU = namedtuple("IMU","id ts ax ay az gx gy gz mx my mz qw qx qy qz altitude lidar")
Range = namedtuple("Range", "id ts min max distance type")
Motor4 = namedtuple("Motor4", "m0 m1 m2 m3 armed")

msgdb = {
    MSG.DISTANCE: (Struct("<BQ3HB"), Range, 16),
    MSG.MOTORS_4: (Struct("<4HB"), Motor4, 9),
    MSG.IMU_FULL: (Struct("<BQ15f"), IMU, 69),
}


def unpack(payload, id):
    """
    [ 0, 1, 2, 3,4,    5:-2, -1]
    [h0,h1,LN,HN,T, payload, CS]
    Header: h0, h1
    N: payload length
       N = (HN << 8) + LN, max data bytes is 65,536 Bytes
         HN: High Byte
         LN: Low Byte
    T: packet type or MsgID
    """
    # print(f"payload: {len(payload)}")
    # if id == 42 and len(payload) == 69:
    #     fmt = Struct("<BQ15f")
    #     info = fmt.unpack(payload)
    #     return IMU(*info)
    # elif id == 30 and len(payload) == 5:
    # return None
    try:
        fmt, msg, size = msgdb[id]
        if len(payload) != size:
            # print(id, ": ", len(payload), " != ", size)
            return None
        info = fmt.unpack(payload)
        return msg(*info)
    except KeyError:
        return None