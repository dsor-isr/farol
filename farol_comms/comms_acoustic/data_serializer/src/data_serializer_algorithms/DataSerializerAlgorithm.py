#!/usr/bin/env python

from __future__ import division
import rospy
import roslib.message
import std_msgs.msg
import sys
import base64

def payload_to_bytes(bits_string):
    # Ensure the binary string length is a multiple of 8
    zero_padded_bits = bits_string + '0' * (8 - len(bits_string) % 8)
    
    # Convert binary string to bytes
    byte_length = len(zero_padded_bits) // 8
    raw_bytes = int(zero_padded_bits, 2).to_bytes(byte_length, byteorder='big')
    
    # Encode the bytes to Base85
    base85_string = base64.a85encode(raw_bytes).decode('ascii')
    
    return base85_string

def payload_to_bits(base85_string):
    # Decode the Base85 string into bytes
    decoded_bytes = base64.a85decode(base85_string)
    
    # Convert the bytes to a binary string
    bits_string = ''.join(f'{byte:08b}' for byte in decoded_bytes)
    
    return bits_string

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
        # print("SERIALIZE: " + str(dec_value) + " -> " + bits*'1')
        return bits*'1'
    elif value_quant<0:
        # print("SERIALIZE: " + str(dec_value) + " -> "  + bits*'0')
        return bits*'0'
    else:
        # print("SERIALIZE: " + str(dec_value) + " -> " + format(value_quant, '0'+str(bits)+'b') + " -> " + hex(int(format(value_quant, '0'+str(bits)+'b'),2)))
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
        # print("DE-SERIALIZE: " + hex(dec_value) + " -> " + str(bin_value) + " -> " + str(value))
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
