#!/usr/bin/env python3

from collections import OrderedDict
import configparser, sys
import argparse

bitlengths = {
0x0002: 8, # DEFTYPE_INTEGER8
0x0003: 16, #D EFTYPE_INTEGER16
0x0004: 32, # DEFTYPE_INTEGER32
0x0005: 8, # DEFTYPE_UNSIGNED8
0x0006: 16, # DEFTYPE_UNSIGNED16
0x0007: 32, # DEFTYPE_UNSIGNED32
0x0008: 32, # DEFTYPE_REAL32
0x0010: 64, # DEFTYPE_REAL64
0x0015: 64, # DEFTYPE_INTEGER64
0x001B: 64, #DEFTYPE_UNSIGNED64
}

def to_hex(value, digits):
    return ("%0"+str(digits)+ "x") % value

def get_mapping(ini, name, writable):
    if not ini[name].getboolean("PDOMapping"):
        print (name, "is not mappable")
        exit(1)
    try:
        can_write = 'w' in ini[name]["AccessType"]
    except:
        can_write = False
        
    try:
        can_read = 'r' in ini[name]["AccessType"]
    except:
        can_read = False
    if (writable and not can_write) or (not writable and not can_read):
        print (name, "is not accesable")
        exit(1)
            
    parts = name.split("sub")
    obj = int(parts[0],16)
    sub = int(parts[1],16) if len(parts) == 2 else 0
    return (obj, sub, bitlengths[int(ini[name]["DataType"],0)])
    
def set_pdo(ini, writable, nr, objs, transmission):
    if len(objs) > len(set(objs)):
        print ("objetcs are not unique")
        exit(1)
    comobj = to_hex(nr + (0x1400 if writable else 0x1800), 4).upper()
    mapobj = to_hex(nr + (0x1600 if writable else 0x1A00), 4).upper()

    if not comobj in ini or not mapobj in ini:
        print (writable , nr , " is invalid")
        exit(1)
    mapping = []
    bits = 0
    for o in objs:
        m = get_mapping(ini, o, writable)
        bits += m[2]
        if bits > 64:
            print ("mapping exceeds 64 bits")
            exit(1)
        mapping.append("0x" + to_hex(m[0],4) + to_hex(m[1],2) +to_hex(m[2],2))

    ini[comobj+"sub1"]["ParameterValue"]="$NODEID+0x%x" % ((0x200 if writable else 0x180)+nr*0x100)
    ini[comobj+"sub2"]["ParameterValue"]="0x%x" % transmission

    ini[mapobj+"sub0"]["ParameterValue"]=str(len(mapping))
    for i in range(len(mapping)):
        ini[mapobj+"sub%x" % (i+1)]["ParameterValue"]=mapping[i]
    for i in range(len(mapping),8):
        try:
            del ini[mapobj+"sub%x" % (i+1)]["ParameterValue"]
        except:
            pass

def patch(fname,out, writable, nr, objs, transmission):
    ini = configparser.RawConfigParser(dict_type=OrderedDict,allow_no_value=True)
    ini.optionxform = lambda option: option
    ini.readfp(open(fname))

    set_pdo(ini, writable, nr, objs, transmission)
    ini.write(open(out,'w'), False)

def patch_all(fname,out, pdos):
    ini = configparser.RawConfigParser(dict_type=OrderedDict,allow_no_value=True)
    ini.optionxform = lambda option: option
    ini.readfp(open(fname))

    for (k,v) in pdos.items():
        parts = k.split("PDO");
        objs =  [ o[0] for o in v[1:] ]
        set_pdo(ini, parts[0] == "R", int(parts[1])-1, objs, v[0])
    ini.write(open(out,'w'), False)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("file")
    parser.add_argument("pdo", choices=["tpdo","rpdo"])
    parser.add_argument("nr", type=int)
    parser.add_argument("objs",nargs='+',metavar='OBJ')
    parser.add_argument("--transmission", type=lambda x:int(x,0),default=1)
    parser.add_argument("--out")

    args = parser.parse_args(args)


    out = args.out
    if not out:
        out = args.file
    patch(args.file, out, args.nr, args.pdo == "rpdo", args.objs, args.transmission)