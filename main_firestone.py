#upload IMU_RAW data to document gimbal id: barcode
import pyrebase, random
import time
import uuid
from threading import Thread
import threading
from pymavlink import mavutil
import os, sys
import math
import calendar;
import urllib.request
import string
import wiringpi as gpio
import requests
import sys
import requests
import json
import firebase_admin
from firebase_admin import credentials
from firebase_admin import firestore
cred = credentials.Certificate("kien-226a6-firebase-adminsdk-b0z33-dd646ac48e.json")
firebase_admin.initialize_app(cred)
db=firestore.client()
import datetime
x = datetime.datetime(2018, 6, 1).now()
from datetime import datetime
def barcode_reader(a):
    """Barcode code obtained from 'brechmos'
    https://www.raspberrypi.org/forums/viewtopic.php?f=45&t=55100"""
    hid = {4: 'a', 5: 'b', 6: 'c', 7: 'd', 8: 'e', 9: 'f', 10: 'g', 11: 'h', 12: 'i', 13: 'j', 14: 'k', 15: 'l', 16: 'm',
           17: 'n', 18: 'o', 19: 'p', 20: 'q', 21: 'r', 22: 's', 23: 't', 24: 'u', 25: 'v', 26: 'w', 27: 'x', 28: 'y',
           29: 'z', 30: '1', 31: '2', 32: '3', 33: '4', 34: '5', 35: '6', 36: '7', 37: '8', 38: '9', 39: '0', 44: ' ',
           45: '-', 46: '=', 47: '[', 48: ']', 49: '\\', 51: ';', 52: '\'', 53: '~', 54: ',', 55: '.', 56: '/'}
    hid2 = {4: 'A', 5: 'B', 6: 'C', 7: 'D', 8: 'E', 9: 'F', 10: 'G', 11: 'H', 12: 'I', 13: 'J', 14: 'K', 15: 'L', 16: 'M',
            17: 'N', 18: 'O', 19: 'P', 20: 'Q', 21: 'R', 22: 'S', 23: 'T', 24: 'U', 25: 'V', 26: 'W', 27: 'X', 28: 'Y',
            29: 'Z', 30: '!', 31: '@', 32: '#', 33: '$', 34: '%', 35: '^', 36: '&', 37: '*', 38: '(', 39: ')', 44: ' ',
            45: '_', 46: '+', 47: '{', 48: '}', 49: '|', 51: ':', 52: '"', 53: '~', 54: '<', 55: '>', 56: '?'}
    fp = open('/dev/hidraw0', 'r') #read data from barcode scanner machine
    ss = "" #create black string to store understanding data
    shift = False
    done = False
    while not done:
        ## Get the character from the HID
        buffer = fp.read(8)
        for c in buffer:
            if ord(c) > 0:
                ##  40 is carriage return which signifies
                ##  we are done looking for characters
                if int(ord(c)) == 40:
                    done = True
                    break;
                ##  If we are shifted then we have to
                ##  use the hid2 characters.
                if shift:
                    ## If it is a '2' then it is the shift key
                    if int(ord(c)) == 2:
                        shift = True
                    ## if not a 2 then lookup the mapping
                    else:
                        ss += hid2[int(ord(c))]
                        shift = False
                ##  If we are not shifted then use
                ##  the hid characters
                else:
                    ## If it is a '2' then it is the shift key
                    if int(ord(c)) == 2:
                        shift = True
                    ## if not a 2 then lookup the mapping
                    else:
                        ss += hid[int(ord(c))]
    fp.close()
    print("barcode"+ ss)
    now = datetime.now()  # current date and time
    data_update = {
        "testResults": [{"msg_1": {"xacc": a[0], "yacc": a[1], "zacc": a[2]}}, {"offsetEncodeTilt": 1}],
        "uploadDate": now.strftime("%d/%m/%Y"),
        "Testdevice_ID": hex(uuid.getnode()),
        "epochtime": calendar.timegm(time.gmtime())
    }
    data_init = {"status": "ok"}


    # ham check
    a = db.collection('gimbaldemo').document(ss).get().exists

    if (a):
        print("found")
        db.collection('gimbaldemo').document(ss).collection('fcQc').add(
            data_update)  # yeu cau moi lan ghi la tao ra cai moi

    else:
        print("not found")
        gimbal_872767007673 = db.collection('gimbaldemo').document(ss)
        gimbal_872767007673.set(data_init)
        gimbal_872767007673.collection('fcQc').document('fcQc_id').set(data_update)
PIN_LED = 2
PIN_BUZZER = 7

global main_state

#LOG_DEVICE = "gimbal"
#LOG_TYPE = "debugs"

#arr_debugs_childname = ["0", "1", "2", "3", "4", "5", "6", "7"]
arr_debugs = [0, 0, 0, 0, 0, 0, 0, 0]
# define the Firebase as per your settings
'''  
config = {
    "apiKey": "AIzaSyB7zPFFcKzfc7YMVgOR8tXgGJocjPmHDok",
    "authDomain": "realtime-92bf3.firebaseapp.com",
    "databaseURL": "https://realtime-92bf3-default-rtdb.firebaseio.com",
    "storageBucket": "realtime-92bf3.appspot.com"
}

print("Gimbal data logger starting... v1.1.0")

firebase = pyrebase.initialize_app(config)
db = firebase.database()
'''

### gpio helper
def toggle_led():
    gpio.digitalWrite(PIN_LED, 1)
    time.sleep(0.5)
    gpio.digitalWrite(PIN_LED, 0)
    time.sleep(0.3)


def buzzer_beep():
    gpio.digitalWrite(PIN_BUZZER, 1)
    time.sleep(0.15)
    gpio.digitalWrite(PIN_BUZZER, 0)
    time.sleep(0.05)


def buzzer_doublebeep():
    buzzer_beep()
    buzzer_beep()


def buzzer_triplebeep():
    buzzer_beep()
    buzzer_beep()
    buzzer_beep()


def convert(s):
    # initialization of string to ""
    new = ""

    # traverse in the string
    for x in s:
        new += x

        # return string
    return new


def is_internet(host='http://google.com'):
    # url = "http://www.kite.com"
    timeout = 5
    try:
        request = requests.get(host, timeout=timeout)
        return True
    except (requests.ConnectionError, requests.Timeout) as exception:
        return False


def current_time():
    # ts = time.time()
    # st = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')
    # return st

    ts = calendar.timegm(time.gmtime())
    return ts


flight_time = current_time()


#def push_new_gimbal():
#   db.child("flightLogger").child(GIMBAL_ID).child("addedAt").set(flight_time)


#def push_update_time():
#    db.child("debugDataTest").child(LOG_DEVICE).child("updatedAt").set(current_time())


#def push_debug():
#    time = current_time()
#    db.child("debugDataTest").child(LOG_DEVICE).child(LOG_TYPE).set(arr_debugs)
#    buzzer_beep()


# print("Debugs data pushed.")

def request_message_stream(msg_id, rate, status):
    mav_connection.mav.request_data_stream_send(
        mav_connection.target_system,
        mav_connection.target_component,
        msg_id,
        rate, status)


#def request_gimbal_uuid():
#   mav_connection.mav.command_long_send(
#        10,
#        250,
#        mavutil.mavlink.MAV_CMD_USER_4,
#        0, 0, 0, 0, 0, 0, 0, 0)


def mav_init():
    global mav_connection
    global time_exit

    # Start a connection listening to a serial port
    mav_connected = False
    while not mav_connected:
        try:
            # mav_connection = mavutil.mavlink_connection("/dev/ttyS0",baud=115200)
            mav_connection = mavutil.mavlink_connection("/dev/ttyUSB0", baud=115200)
            # mav_connection = mavutil.mavlink_connection("udp:192.168.11.8:14550")
            mav_connected = True
        except:
            print("Can not init mavlink. Try again...")
            buzzer_triplebeep()
            time.sleep(3)

    while not time_exit:
        # request heartbeat from gimbal
        request_message_stream(mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, 500, 1)

        msg = mav_connection.recv_match(type="HEARTBEAT", blocking=True, timeout=5)

        if msg is not None:
            print("Mavlink connected.")
            break
        else:
            buzzer_triplebeep()
            print("Mavlink timeout. Try again...")
        # sys.exit()


def mavlink_wait_message():
    global time_exit
    global a
    while not time_exit:
        global mav_connection
        try:
            msg = mav_connection.recv_match(blocking=True)
            pass
        except:
            time_exit = True
            sys.exit()
        if not msg:
            continue

        type = msg.get_type()
        if type == "BAD_DATA":
            pass
        # if mavutil.all_printable(msg.data):
        #     sys.stdout.write(msg.data)
        #     sys.stdout.flush()
        elif type == "RAW_IMU":
            #print("msg.xmag:{:.2f} msg.ymag:{:.2f} msg.zmag:{:.2f}". format(msg.xacc, msg.yacc, msg.zacc))
            a = [msg.xacc, msg.yacc, msg.zacc]

            pass
        elif type == "ATTITUDE":
            # print("roll:{:.2f} pitch:{:.2f} yaw:{:.2f}". format(msg.roll / math.pi * 180, msg.pitch / math.pi * 180, msg.yaw / math.pi * 180))
            pass
        elif type == "DEBUG":
            arr_debugs[msg.ind] = msg.value

        # print(msg)
        elif type == "AUTH_KEY":
            # print(msg)
            pass
    # else:
    # 	print(msg.get_type())


def mavlink_send_hb():
    global time_exit
    while not time_exit:
        global mav_connection
        # mav_connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
        #                                        mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

        # write custom data to serial port
        _bytes = mav_connection.port_forward.inWaiting()
        if _bytes:
            in_bin = mav_connection.port_forward.read(_bytes)
            mav_connection.port.write(in_bin)
        # print("Write custom data")

        time.sleep(0.01)


def toggle_led_handle():
    global time_exit
    while not time_exit:
        toggle_led()


time_exit = False
thr_heartbeat_send = threading.Thread(target=mavlink_send_hb)
thrd_check_message = threading.Thread(target=mavlink_wait_message)
#thrd_send_data_firebase = threading.Thread(target=barcode_reader,args=(a,))
thrd_led = threading.Thread(target=toggle_led_handle)
thrd_led.start()


def mainloop():
    main_state = 0

    gpio.wiringPiSetup()
    gpio.pinMode(PIN_LED, 1)
    gpio.pinMode(PIN_BUZZER, 1)

    global time_exit
    time_tick = 0
    while not time_exit:
        if main_state == 0:
            # init
            main_state = 1
            buzzer_triplebeep()

        elif main_state == 1:
            # check mavlink
            mav_init()

            thrd_check_message.start()
            # start forward data
            thr_heartbeat_send.start()

            #thrd_send_data_firebase.start()
            main_state = 2

        elif main_state == 2:
            # mavlink ok, check internet
            if is_internet():
                print("Internet connected")
                main_state = 3
            else:
                buzzer_doublebeep()
                print("No intertnet connection")

        elif main_state == 3:
            # intertnet ok, mavlink ok, start

            main_state = 4

        elif main_state == 4:
            # request debug data to push to firebase server
            request_message_stream(mavutil.mavlink.MAVLINK_MSG_ID_RAW_IMU, 200, 1)

            main_state = 5

        elif main_state == 5:
            # push debugs
            #push_update_time()
            #push_debug()
            print(a)
            barcode_reader(a)

            # recheck internet connection after every 10 secs
            '''
            time_tick = time_tick + 1
            if time_tick > 20:
                time_tick = 0
                main_state = 6
            else:
                main_state = 5"
            '''

        elif main_state == 6:
            # check internet
            if is_internet():
                main_state = 5
            else:
                buzzer_doublebeep()
                print("No intertnet connection")

        #
        time.sleep(0.5)

    print("Gimbal data logger stopped...")


thrd_mainloop = threading.Thread(target=mainloop)
thrd_mainloop.start()

