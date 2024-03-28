# Data Serializer Package

## Description

This package handles the serialization/deserialization of data to/from the acoustic modems.

The *acoustic_data.yaml* configuration file is essential for this package, since it specifies from which ROS topics the data meant to be sent and received should be obtained. It is important that this file is the same in all vehicles that participate in the acoustic communications, so that the acoustic data channels defined are the same.

### Serialization

The *data_serializer* node automatically subscribes to the ROS topics specified in the *acoustic_data.yaml* file as *topic_in*, keeping the most updated data from each of them.

When the */#vehicle#/acomms/scheme/trigger_serialization* topic is published on, the data stored is serialized and published on the */#vehicle#/acomms/serializer/payload_to_transmit*, which will be sent by the acoustic modem afterwards.

### Deserialization

The *data_serializer* is subscribed to the */#vehicle#/acomms/scheme/payload_to_deserialize*, from where it gets the data to deserialize.

Afterwards, it automatically publishes the deserialized data to the ROS topics specified in the *acoustic_data.yaml* file as *topic_out*.

## Nodes
* [data_serializer](data_serializer.md)
