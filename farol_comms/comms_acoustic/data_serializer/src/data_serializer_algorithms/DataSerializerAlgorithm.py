#!/usr/bin/env python

from __future__ import division
import rospy
import roslib.message
import std_msgs.msg
import sys

def payload_to_bytes(bits_string):
    zero_padded_bits=bits_string+'0'*(8 - len(bits_string) % 8)
    hex_representation=format(int(zero_padded_bits,2), '0'+str(len(zero_padded_bits)//4)+'X') 
    if sys.version_info[0] == 2:
        return hex_representation.decode("hex")
    elif sys.version_info[0] == 3: 
        return hex_representation

def payload_to_bits(bytes_string):
    if sys.version_info[0] == 2:
        hex_representation=bytes_string.encode("hex")
    elif sys.version_info[0] == 3:
        hex_representation=bytes_string
    zero_padded_bits=format(int(hex_representation,16), '0'+str(len(hex_representation)*4)+'b')
    return zero_padded_bits

def dec_to_bin(dec_value,low,high,bits,is_int=False):
    #saturate value
    dec_value=min(dec_value,high)
    dec_value=max(dec_value,low)

    #adjust high value if it's an integer and the quantization is too small, to avoid rounding issues
    if is_int:
        high=max(high,low+2**bits-1)

    quant=(high-low)/(2**bits-1)
    value_quant=int(round((dec_value-low)/quant))
    if value_quant>2**bits-1:
        return bits*'1'
    elif value_quant<0:
        return bits*'0'
    else:
        return format(value_quant, '0'+str(bits)+'b')

def bin_to_dec(bin_value,low,high,bits,is_int=False):
    dec_value=int(bin_value,2)

    #adjust high value if it's an integer and the quantization is too small, to avoid rounding issues
    if is_int:
        high=max(high,low+2**bits-1)

    quant=(high-low)/(2**bits-1)
    value=low+dec_value*quant
    if is_int:
        return int(round(value))
    else:
        return value

def extract_field(msg_data,field_address):
    if len(field_address)==1:
        return getattr(msg_data, field_address[0])
    else:
        return extract_field(getattr(msg_data,field_address[0]), field_address[1:])

def add_field_to_dict(final_dict,field_address,field_value):
    aux_dict=final_dict
    for ind,val in enumerate(field_address):
        if ind==len(field_address)-1:
            aux_dict[field_address[ind]]=field_value
        else:
            if field_address[ind] not in aux_dict.keys():
                aux_dict[val]={}
            aux_dict=aux_dict[val]
