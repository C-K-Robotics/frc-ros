#!/usr/bin/env python3

import numpy as np
import ntcore

nt_dict = {
    "BooleanArray":"BooleanArray",         # Do not support BooleanArray
    "Bool":"Boolean",
    "Float64MultiArray":"DoubleArray",
    "Float64":"Double",
    "Float32MultiArray":"FloatArray",
    "Float32":"Float",
    "Int32MultiArray":"IntegerArray",
    "Int32":"Integer",
    "StringArray":"StringArray",        # Do not support StringArray 
    "String":"String"
}

ros_dict = dict()
for key, value in nt_dict.items():
    ros_dict[value].append(key)         

def nt_type_dict(ros_type:str):
    return nt_dict[ros_type]

def ros_type_dict(nt_type:str):
    return ros_dict[nt_type]

def nt_create_topic(inst:ntcore.NetworkTableInstance, topic_type:str, topic_name:str):
    name = topic_name[(topic_name.rfind("/")) + 1:]
    table_name = topic_name[:(topic_name.rfind("/"))]
    table = inst.getTable(table_name)

    if topic_type == "BooleanArray":
        return table.getBooleanArrayTopic(name)
    elif topic_type == "Boolean":
        return table.getBooleanTopic(name)

    elif topic_type == "DoubleArray":
        return table.getDoubleArrayTopic(name)
    elif topic_type == "Double":
        return table.getDoubleTopic(name)

    elif topic_type == "FloatArray":
        return table.getFloatArrayTopic(name)
    elif topic_type == "Float":
        return table.getFloatTopic(name)
        
    elif topic_type == "IntegerArray":
        return table.getIntegerArrayTopic(name)
    elif topic_type == "Integer":
        return table.getIntegerTopic(name)
    
    elif topic_type == "StringArray":
        return table.getStringArrayTopic(name)
    elif topic_type == "String":
        return table.getStringTopic(name)

def nt_read(topic, default_value):
    sub = topic.subscribe(default_value)
    value = sub.get()
    sub.close()
    return value
    
def nt_write(topic, value):
    pub = topic.publish()
    value = pub.set(value)
    pub.close()
