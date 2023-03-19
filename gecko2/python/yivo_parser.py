
from enum import IntEnum

def printp(s):
    if 0:
        print(s)


ReadState_t = IntEnum("ReadState_t", [
    "NONE_STATE",
    "H0_STATE",
    "H1_STATE",
    "S0_STATE",
    "S1_STATE",
    "TYPE_STATE",
    "DATA_STATE",
    "CS_STATE"]
)

class YivoParser:

    def __init__(self, header):
        self.readState = ReadState_t.NONE_STATE
        self.buff = []
        # self.yivo = y
        self.header = header
        self.buffer_msgid = -1

    def get_info(self):
        return b''.join(self.buff), self.buffer_msgid

    def checksum(self,size,msgid,msg):
        # if size == 0 and msg == None:
        #     return msgid

        # a,b = struct.pack('H', size)
        a = 0x00FF & size
        b = size >> 8

        cs = (a ^ b)^msgid
        # msg = [cs] + msg
        # cs = reduce(xor, msg)
        for m in msg:
            cs ^= ord(m)
        # printp("cs", cs, cs.to_bytes(1,'little'))
        return cs

    def parse(self, c):
        ret = False
        # printp(c)
        # c = ord(c)
        if self.readState == ReadState_t.NONE_STATE:
            if c == self.header[0]:
                self.buff = []
                # self.buff.append(c) # h0
                self.readState = ReadState_t.H0_STATE
                self.payload_size = 0
                # self.index = 0
                # printp("h0")
        elif self.readState == ReadState_t.H0_STATE:
            if c == self.header[1]:
                # self.buff.append(c) # h1
                self.readState = ReadState_t.H1_STATE
                # printp("h1")
            else:
                self.readState = ReadState_t.NONE_STATE
        elif self.readState == ReadState_t.H1_STATE:
            # self.buff.append(c) # s0
            self.readState = ReadState_t.S0_STATE
            self.payload_size = ord(c)
            # printp("s0")
        elif self.readState == ReadState_t.S0_STATE:
            # self.buff.append(c) # s1
            # cc = ord(c)
            # ccc = ord(self.buff[2])
            # self.payload_size = (cc << 8) | ccc
            self.payload_size |= ord(c) << 8
            self.readState = ReadState_t.S1_STATE
            # printp("s1")
            # printp(f"size: {self.payload_size}")
        elif self.readState == ReadState_t.S1_STATE:
            # self.buff.append(c) # type
            self.buffer_msgid = ord(c)
            self.readState = ReadState_t.TYPE_STATE
            # printp("t")
        elif self.readState == ReadState_t.TYPE_STATE:
            # c = ord(c)
            self.buff.append(c) # data0
            # self.index = 1
            self.readState = ReadState_t.DATA_STATE
            # printp("d0")
        elif self.readState == ReadState_t.DATA_STATE:
            # c = ord(c)
            self.buff.append(c) # data1-dataN
            # self.index += 1
            # if self.index == self.payload_size:
            if len(self.buff) == self.payload_size:
                self.readState = ReadState_t.CS_STATE
                # printp(f"data buff size: {len(self.buff)}")
        elif self.readState == ReadState_t.CS_STATE:
            c = ord(c)
            # self.buff.append(c) # cs
            # cs = self.checksum(self.payload_size, self.buffer_msgid, self.buff[5:-1])
            cs = self.checksum(self.payload_size, self.buffer_msgid, self.buff)
            if cs == c:
                ret = True
                self.readState = ReadState_t.NONE_STATE

        return ret