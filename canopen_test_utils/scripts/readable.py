#!/usr/bin/env python2
import sys, importlib

if len(sys.argv) < 2:
    print("please provide mapping modulde as first argument, e.g schunk_mapping or elmo_mapping");
    exit()

PDOs = importlib.import_module(sys.argv[1]).PDOs

def hex_reverse(data):
    return ''.join(reversed([data[i:i+2] for i in xrange(0, len(data), 2)]))
    
def decode_state(id,data):
    state = int(data[0:2],16)

    if state & 128:
        toggle =" T=1"
        state ^=128
    else:
        toggle = "T=0"
    if state == 0:
        state = 'boot-up'
    elif state == 4:
        state = 'stopped'
    elif state == 5:
        state = 'operational'
    elif state == 127:
        state = 'pre-op'
    elif state == 130:
        state = 'recom'
    
    return['State', id - 1792, state, toggle]

def decode_nmt(id,data):
    command = int(data[0:2],16)
    if command == 1:
        command = 'start'
    elif command == 2:
        command = 'stop'
    elif command == 128:
        command = 'pre'
    elif command == 129:
        command = 'reset'
    elif command == 130:
        command = 'recom'

    node = int(data[2:4],16)
    if node == 0:
        node ="all"
    return['NMT', command, node]
    
def decode_sync(id,data):
    if len(data):
        return ["SYNC", data]
    else:
        return ["SYNC"]
        
def decode_pdo(id,data, name, start_id):
    out = []

    i = 0
    for d in PDOs[name][1:]:
        out += [d[1],hex_reverse(data[i:i+2*d[2] ])]
        i +=2*d[2]
    return [name, id-start_id]+out
    
def decode_sdo(id,data, name, start_id):
    command = int(data[0:2],16) >> 5
    out = [name, id-start_id]
    if command == 1 or command == 2 or command == 3:
        out += [hex_reverse(data[2:6]), data[6:8], hex_reverse(data[8:])]
    return out

def decode_emcy(id,data):
    return [ "EMCY", id - 0x80,  "EEC:", hex_reverse(data[0:4]), "Reg:", data[4:6],"Msef: ",data[6:16]]
    pass
    
def decode_canopen(id,data):
    if id == 0:
        return decode_nmt(id,data)
    elif id > 1792 and id <= 1792 + 127:
        return decode_state(id,data)
    elif id == 0x80:
        return decode_sync(id,data)
    elif id > 0x80 and id <= 0x80 + 127:
        return decode_emcy(id,data)

    elif id > 0x180 and id <= 0x180 + 127:
        return decode_pdo(id,data,"TPDO1", 0x180)
    elif id > 0x200 and id <= 0x200 + 127:
        return decode_pdo(id,data,"RPDO1", 0x200)

    elif id > 0x280 and id <= 0x280 + 127:
        return decode_pdo(id,data,"TPDO2", 0x280)
    elif id > 0x300 and id <= 0x300 + 127:
        return decode_pdo(id,data,"RPDO2", 0x300)
        
    elif id > 0x380 and id <= 0x380 + 127:
        return decode_pdo(id,data,"TPDO3", 0x380)
    elif id > 0x400 and id <= 0x400 + 127:
        return decode_pdo(id,data,"RPDO3", 0x400)

    elif id > 0x480 and id <= 0x480 + 127:
        return decode_pdo(id,data,"TPDO4", 0x480)
    elif id > 0x500 and id <= 0x500 + 127:
        return decode_pdo(id,data,"RPDO4", 0x500)

    elif id > 0x580 and id <= 0x580 + 127:
        return decode_sdo(id,data,"TSDO", 0x580)
    elif id > 0x600 and id <= 0x600 + 127:
        return decode_sdo(id,data,"RSDO", 0x600)
    return [id,data]

start = -1
for line in sys.stdin:
    parts = line.split()
    if start < 0:
        for i in range(len(parts)):
            if 'can' in parts[i]:
                start = i
                break
    if '#' in parts[start+1]:
        otherparts = parts[start+1].split("#")
        id = int(otherparts[0],16)
        data = otherparts[1]
    else:
        id = int(parts[start+1],16)
        data = ''.join(parts[start+3:])
    print parts[:start+1] + decode_canopen(id,data)
