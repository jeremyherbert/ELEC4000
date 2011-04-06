from Queue import *
import socket
import thread

class Packet:
    """
    The Packet class offers a handy way to build pack and unpack
    binary data based on a given pattern.
    """

    def _decode(self, v):
        r = long(0)
        for i in v:
            r = (r << 8) + i
        return r

    def _encode(self, val, dim):
        output = []
        for i in range(dim):
            output.append(int(val & 0xFF))
            val = val >> 8
        output.reverse()
        return output

    def _sign(self, val, dim):
        if val > (1 << (dim * 8 - 1)):
            return val - (1 << (dim * 8))
        return val

    def __init__(self, desc, packet = None):
        offset = 0
        boffset = 0
        sum = 0
        for i in range(len(desc)-1, -1, -1):
            (n, t, s) = desc[i]
            if s == None:
                if sum > 0:
                    desc[i] = (n, t, -sum)
                break
            sum += s
        self.__dict__['_schema'] = [(t, s) for (n, t, s) in desc]
        self.__dict__['_names'] = [n for (n, t, s) in desc]
        self.__dict__['_values'] = []
        if type(packet) == type([]):
            for (t, s) in self._schema:
                if t == 'int':
                    self._values.append(self._decode(packet[offset:offset + s]))
                    offset += s
                elif t == 'sint':
                    self._values.append(self._sign(self._decode(packet[offset:offset + s]), s))
                    offset += s
                elif t == 'bint':
                    doffset = 8 - (boffset + s)
                    self._values.append((packet[offset] >> doffset) & ((1<<s) - 1))
                    boffset += s
                    if boffset == 8:
                        offset += 1
                        boffset = 0
                elif t == 'string':
                    self._values.append(''.join([chr(i) for i in packet[offset:offset + s]]))
                    offset += s
                elif t == 'blob':
                    if s:
                        if s > 0:
                            self._values.append(packet[offset:offset + s])
                            offset += s
                        else:
                            self._values.append(packet[offset:s])
                            offset = len(packet) + s
                    else:
                        self._values.append(packet[offset:])
        elif type(packet) == type(()):
            for i in packet:
                self._values.append(i)
        else:
            for v in self._schema:
                self._values.append(None)

    def __repr__(self):
        return self._values.__repr__()

    def __str__(self):
        r = ""
        for i in range(len(self._names)):
            r += "%s: %s " % (self._names[i], self._values[i])
        for i in range(len(self._names), len(self._values)):
            r += "%s" % self._values[i]
        return r

    # Implement the struct behavior
    def __getattr__(self, name):
        if type(name) == type(0):
            return self._names[name]
        else:
            return self._values[self._names.index(name)]

    def __setattr__(self, name, value):
        if type(name) == type(0):
            self._values[name] = value
        else:
            self._values[self._names.index(name)] = value

    def __ne__(self, other):
        if other.__class__ == self.__class__:
            return self._values != other._values
        else:
            return True

    def __eq__(self, other):
        if other.__class__ == self.__class__:
            return self._values == other._values
        else:
            return False

    def __nonzero__(self):
        return True;

    # Implement the map behavior
    def __getitem__(self, key):
        return self.__getattr__(key)

    def __setitem__(self, key, value):
        self.__setattr__(key, value)

    def __len__(self):
        return len(self._values)

    def keys(self):
        return self._names

    def values(self):
        return self._values

    # Custom functions
    def names(self):
        return self._names

    def sizes(self):
        return self._schema

    def payload(self):
        r = []
        boffset = 0
        for i in range(len(self._schema)):
            (t, s) = self._schema[i]
            if t == 'int':
                r += self._encode(self._values[i], s)
                boffset = 0
            elif t == 'bint':
                doffset = 8 - (boffset + s)
                if boffset == 0:
                    r += [self._values[i] << doffset]
                else:
                    r[-1] |= self._values[i] << doffset
                boffset += s
                if boffset == 8:
                    boffset = 0
            elif self._values[i] != []:
                r += self._values[i]
        for i in self._values[len(self._schema):]:
            r += i
        return r

class RawPacket(Packet):
    def __init__(self, ts = None, data = None):
        Packet.__init__(self,
                        [('ts' ,  'int', 4),
                         ('data', 'blob', None)],
                        None)
        self.ts = ts;
        self.data = data

class SFClient:
    def __init__(self, host, port, qsize=10):
        self._in_queue = Queue(qsize)
        self._s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._s.connect((host, port))
        data = self._s.recv(2)
        if data != 'U ':
            print "Wrong handshake"
        self._s.send("U ")
        print "Connected"
        thread.start_new_thread(self.run, ())

    def run(self):
        while True:
            length = ord(self._s.recv(1))
            data = self._s.recv(length)
            data = [ord(c) for c in data][1:]
            #print "Recv %d bytes" % (length), ActiveMessage(data)
            if self._in_queue.full():
                #print "Warning: Buffer overflow"
                self._in_queue.get()
            p = RawPacket()
            p.data = data
            self._in_queue.put(p, block=False)

    def read(self, timeout=0):
        return self._in_queue.get()

    def write(self, payload):
        print "SFClient: write:", payload
        if type(payload) != type([]):
            # Assume this will be derived from Packet
            payload = payload.payload()
        payload = [0] + payload
        self._s.send(chr(len(payload)))
        self._s.send(''.join([chr(c) for c in payload]))
        return True


