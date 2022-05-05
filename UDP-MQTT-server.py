#! /usr/bin/env python

#------------------------------------------------------------#
# UDP example to forward data from a local IPv6 DODAG
# Antonio Lignan <alinan@zolertia.com>
#------------------------------------------------------------#
import sys
import json
import datetime
from socket import*
from socket import error
from time import sleep
import struct
from ctypes import *
import paho.mqtt.client as paho
from paho import mqtt
#------------------------------------------------------------#
ID_STRING      = "V0.1"
#------------------------------------------------------------#
PORT              = 5678
CMD_PORT          = 8765
BUFSIZE           = 1024
#------------------------------------------------------------#
ENABLE_MQTT       = 1
ENABLE_LOG        = 1
#------------------------------------------------------------#
DEBUG_PRINT_JSON  = 1
#------------------------------------------------------------#
MQTT_URL          = "efe545b723c6413a94f9b2910b00c39a.s1.eu.hivemq.cloud"
MQTT_PORT         = 8883
MQTT_KEEPALIVE    = 60
MQTT_URL_PUB      = "vominhtri/pi/"
MQTT_URL_TOPIC    = "/cmd"
#------------------------------------------------------------#
# If using a client based on the Z1 mote, then enable by equal to 1, else if
# using the RE-Mote equal to 0
EXAMPLE_WITH_Z1   = 0
#------------------------------------------------------------#
# Message structure
#------------------------------------------------------------#
class SENSOR(Structure):
    _fields_ = [
                 ("sfd",                         c_uint16),
                 ("counter",                     c_uint16),
                 ("type",                        c_uint16),
                 ("temper1",                     c_uint16),
                 ("temper2",                     c_uint16),
                 ("spo2bpm",                     c_uint16),
                 ("systolic",                    c_uint16),
		 ("diastolic",			 c_uint16),
		 ("spo2",		         c_uint16)
               ]

    def __new__(self, socket_buffer):
        return self.from_buffer_copy(socket_buffer)

    def __init__(self, socket_buffer):
        pass
#------------------------------------------------------------#
# Helper functions
#------------------------------------------------------------#
def print_recv_data(msg):
  print "***"
  for f_name, f_type in msg._fields_:
    print "{0}:{1} ".format(f_name, getattr(msg, f_name)),
  print
  print "***"
# -----------------------------------------------------------#
def publish_recv_data(data, pubid, conn, addr):
  try:
    res, mid = conn.publish(MQTT_URL_PUB + str(pubid), payload=data, qos=1)
    print "MQTT: Publishing to {0}... " + "{1} ({2})".format(mid, res, str(pubid))
  except Exception as error:
    print error
# -----------------------------------------------------------#
def jsonify(keyval, val):
  return json.dumps(dict(value=val, key=keyval))
# -----------------------------------------------------------#
def jsonify_recv_data(msg):
  sensordata = '{"values":['
  for f_name, f_type in msg._fields_:
    sensordata += jsonify(f_name, getattr(msg, f_name)) + ","
  sensordata = sensordata[:-1]
  sensordata += ']}'
  
  # Paho MQTT client doesn't support sending JSON objects
  json_parsed = json.loads(sensordata)
  if DEBUG_PRINT_JSON:
    print json.dumps(json_parsed, indent=2)

  return sensordata
# -----------------------------------------------------------#
def send_udp_cmd(addr):
  client = socket(AF_INET6, SOCK_DGRAM)
  print "Sending reply to " + addr

  try:
    client.sendto("Hello from the server", (addr, CMD_PORT))
  except Exception as error:
    print error

  client.close()
# -----------------------------------------------------------#
# MQTT related functions
# -----------------------------------------------------------#
def on_connect(client, userdata, flags, rc, properties=None):
  print("MQTT: Connected ({0}) ").format(str(rc))
  client.subscribe(MQTT_URL_PUB + MQTT_URL_TOPIC)
#------------------------------------------------------------#
def on_message(client, userdata, msg):
  print("MQTT: RX: " + msg.topic + " : " + str(msg.payload))
#------------------------------------------------------------#
def on_publish(client, userdata, mid, properties=None):
  print("MQTT: Published {0}").format(mid)
#------------------------------------------------------------#
# UDP6 and MQTT client session
# -----------------------------------------------------------#
def start_client():
  now = datetime.datetime.now()
  print "UDP6-MQTT server side application "  + ID_STRING
  print "Started " + str(now)
  try:
    s = socket(AF_INET6, SOCK_DGRAM)
    s.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)

    # Replace address below with "aaaa::1" if tunslip6 has created a tunnel
    # interface with this address
    s.bind(('', PORT))

  except Exception:
    print "ERROR: Server Port Binding Failed"
    return
  print 'UDP6-MQTT server ready: %s'% PORT
  print "msg structure size: ", sizeof(SENSOR)
  print
  
  if ENABLE_MQTT:
    # Initialize MQTT connection
    try:
      client = paho.Client(client_id="", userdata=None, protocol=paho.MQTTv5)
    except Exception as error:
      print error
      raise

    # enable TLS
    client.tls_set(tls_version=mqtt.client.ssl.PROTOCOL_TLS)
    # set username and password
    client.username_pw_set("vominhtri", "Vominhtrithcsdhtp1")

    client.on_connect = on_connect
    client.on_message = on_message
    client.on_publish = on_publish

    try:
      # connect to HiveMQ Cloud on port 8883 (default for MQTT)
      client.connect(MQTT_URL, MQTT_PORT, MQTT_KEEPALIVE)
    except Exception as error:
      print error
      raise

    # Start the MQTT thread and handle reconnections, also ensures the callbacks
    # being triggered
    client.loop_start()

  while True:
    data, addr = s.recvfrom(BUFSIZE)
    now = datetime.datetime.now()
    print str(now)[:19] + " -> " + str(addr[0]) + ":" + str(addr[1]) + " " + str(len(data))

    msg_recv = SENSOR(data)
    if ENABLE_LOG:
      print_recv_data(msg_recv)
    sensordata = jsonify_recv_data(msg_recv)

    if ENABLE_MQTT:
      publish_recv_data(sensordata, msg_recv.sfd, client, addr[0])

    send_udp_cmd(addr[0])

  client.loop_stop()


#------------------------------------------------------------#
# MAIN APP
#------------------------------------------------------------#
if __name__ == "__main__":
  start_client()