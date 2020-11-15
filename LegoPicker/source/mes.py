#!/usr/bin/env python

import requests
import json
import rospy
import time
import threading
from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray


globalurl =  "http://10.10.19.20" #"http://127.0.0.1:5000"
order_done = ""
ticket_done = 0
signal = ""
flag = False
state = 0
total_exe_time = 0
execution_timer = 0
exe_flag = False
print_flag = True
total = 0
success = 0
curr_order = ""
success = 0
total = 0
stateFlag = False


############################
#       MES STATES         #
############################
idle = 0
clear = 1
ready = 2
taken = 3
error = 4
completed = 5
deleted = 6

#############################
#       PackML STATES       #
#############################
STOPPED = 2
STARTING = 3
IDLE = 4
SUSPENDED = 5
EXECUTE = 6
STOPPING = 7
ABORTING = 8
ABORTED = 9
HOLDING = 10
HELD = 11
RESETTING = 100
SUSPENDING = 101
UNSUSPENDING = 102
CLEARING = 103
UNHOLDING = 104
COMPLETING = 105
COMPLETE = 106


globalheader = {
    'Content-Type': "application/json",
    'Accept-Language': "en_US",
    'Cache-Control': "no-cache"
}


def callback_id(data):
    global order_done
    order_done = data.data


def callback_ticket(data):
    global ticket_done
    ticket_done = data.data


def callback_sig(data):
    global signal, flag
    signal = data.data
    flag = True;

def callback_packml(data):
    global state, stateFlag
    state = data.data
    stateFlag = True

# Creating publishers
sigMESPub = rospy.Publisher('signalMESTopic', Int32, queue_size=1)
idPub = rospy.Publisher('idTopic', Int32, queue_size=1)
ticketPub = rospy.Publisher('ticketTopic', String, queue_size=1)
brickPub = rospy.Publisher('brickTopic', String, queue_size=1)
runPub = rospy.Publisher('runTopic', Int32MultiArray, queue_size=1)

# Creating a subscriber
sigRoboSub = rospy.Subscriber('signalRoboTopic', Int32, callback_sig)
idSub = rospy.Subscriber('idTopic', Int32, callback_id)
ticketSub = rospy.Subscriber('ticketTopic', String, callback_ticket)
packMLSub = rospy.Subscriber('packMLTopic', Int32, callback_packml)

rospy.init_node('comspy', anonymous=True)
rate = rospy.Rate(10)

total_run_timer = time.time()


def get_orders():
    url = globalurl + "/orders"
    headers = globalheader
    response = requests.request("GET", url, headers=headers)
    response = json.loads(response.text)
    return response


def take_order(order):
    url = globalurl + "/orders/" + str(order)
    headers = globalheader
    response = requests.request("PUT", url, headers=headers)
    response = json.loads(response.text)
    return response["ticket"]


def fill_bricks(order):
    brick_vector = ""
    orders = get_orders()
    _id = order
    for i in orders["orders"]:
        if i["id"] == _id:
            for j in range(i["blue"]):
                brick_vector += "B"
            for j in range(i["yellow"]):
                brick_vector += "Y"
            for j in range(i["red"]):
                brick_vector += "R"
    return brick_vector


def delete_order(order, ticket):
    url = globalurl + "/orders/" + str(order) + "/" + str(ticket)
    headers = globalheader
    response = requests.request("DELETE", url, headers=headers)
    print(response)
    try:
        response = json.loads(response.text)
    except:
        response = "Order deleted correctly"
    return response


def get_free_order_id():
    orders = get_orders()
    for i in orders["orders"]:
        if i["status"] == "ready":
            return i["id"]
    print str("Fail")
    return str("fail")


def run_MES():
    global flag, total, success, curr_order
    if signal == idle and flag == True:
        flag = False
        sigMESPub.publish(clear)
    elif signal == ready and flag == True:
        free_order = get_free_order_id()
        log_order_start(free_order)
        total = total + 1
        if free_order != "fail":
            flag = False
            ticket = take_order(free_order)
            brick_vector = fill_bricks(free_order)
            idPub.publish(free_order)
            brickPub.publish(brick_vector)
            ticketPub.publish(str(ticket))
            sigMESPub.publish(taken)
        else:
            flag = False
            sigMESPub.publish(error)
    elif signal == completed and flag == True:
        flag = False
        delete_order(order_done, ticket_done)
        success = success + 1
        log_order_done(order_done)
        sigMESPub.publish(deleted)


def calc_execution_time(packml_state):
    global exe_flag, execution_timer, total_exe_time
    if packml_state == EXECUTE and exe_flag == False:
        execution_timer = time.time()
        exe_flag = True
    if packml_state != EXECUTE and exe_flag == True:
        end = time.time()
        total_exe_time = total_exe_time + (end-execution_timer)
        exe_flag = False


def log_state(state):
    payload = None

    #PML_Idle
    if state == IDLE:
        payload = {
            "cell_id": 12,
            "comment": "State: IDLE",
            "event": "PML_Idle"
        }

    #PML_Execute
    if state == EXECUTE:
        payload = {
            "cell_id": 12,
            "comment": "State: EXECUTE",
            "event": "PML_Execute"
        }

    #PML_Complete ???

    #PML_Held
    if state == HOLDING:
        payload = {
            "cell_id": 12,
            "comment": "State: HELD",
            "event": "PML_Held"
        }

    #PML_Suspended
    if state == SUSPENDED:
        payload = {
            "cell_id": 12,
            "comment": "State: SUSPENDED",
            "event": "PML_Suspended"
        }

    #PML_Aborted
    if state == ABORTED:
        payload = {
            "cell_id": 12,
            "comment": "State: ABORTED",
            "event": "PML_Aborted"
        }

    #PML_Stopped
    if state == STOPPED:
        payload = {
            "cell_id": 12,
            "comment": "State: STOPPED",
            "event": "PML_Stopped"
        }


    payload = json.dumps(payload)
    post_log(payload)
    return

def log_order_start(order):
    payload = {
        "cell_id": 12,
        "comment": "We started on "+str(order),
        "event": "Order_Start"
    }
    payload = json.dumps(payload)
    post_log(payload)
    return

def log_order_done(order):
    payload = {
        "cell_id": 12,
        "comment": "We are done with "+str(order),
        "event": "Order_Done"
    }
    payload = json.dumps(payload)
    post_log(payload)
    return

def post_log(payload):
    url = globalurl+"/log"
    headers = globalheader
    response = requests.request("POST", url, data=payload, headers=headers)
    print(response.text)
    logs = json.loads(response.text)
    return logs



oee = Int32MultiArray()

print("Started MES")
while not rospy.is_shutdown():
    run_MES()
    if stateFlag:
        log_state(state)
        stateFlag = False
    calc_execution_time(state)
    oee.data = [total_run_timer,total,success,total_exe_time]
    runPub.publish(oee)
    time.sleep(0.25)