import requests
import time
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32

language = {'Accept-Language': "en_US"}

signal = ""
flag = False

def callback_sig(data):
    global signal, flag
    signal = data.data
    flag = True;


sigMirPub = rospy.Publisher('mirRoboTopic', Int32, queue_size=1)
sigMirSub = rospy.Subscriber('roboMirTopic', Int32, callback_sig)

rospy.init_node('mirNode', anonymous=True)
rate = rospy.Rate(10)

class RestMiR():
    def __init__(self):
        self.authorization = {
            'Authorization': "Basic ZGlzdHJpYnV0b3I6NjJmMmYwZjFlZmYxMGQzMTUyYzk1ZjZmMDU5NjU3NmU0ODJiYjhlNDQ4MDY0MzNmNGNmOTI5NzkyODM0YjAxNA=="}
        self.HOST = 'http://10.10.19.42/api/v2.0.0/'
    #We need to put the name of our mission
    def get_mission(self, mission_name="MISSION_NAME"):
        response = requests.get(self.HOST + 'missions', headers=self.authorization)
        mission = ""
        if response.status_code != 200:
            print(response.status_code)
        for counter in response.json():
            if counter["name"] == mission_name:
                mission = counter
                print(mission)
        return mission


    def add_mission_to_queue(self, mission):
        r = requests.get(self.HOST + 'mission_queue', headers=self.authorization)
        data = {"mission_id": str(mission["guid"])}
        for i in[-1,-2,-3,-4]:
            #print(data['mission_id'])
            _id = requests.get(self.HOST + 'mission_queue/'+str(r.json()[i]['id']), headers=self.authorization)
            #print(_id.json()['mission_id'])
            if data['mission_id'] == _id.json()['mission_id']:
                if r.json()[i]['state'] == 'Pending' or  r.json()[i]['state'] == 'Executing':
                    print("ERROR" + " Already Pending or Executing")
                    return 0
        response = requests.post(self.HOST + "mission_queue", json=data, headers=self.authorization)
        if response.status_code != 201:
            print("ERROR" + str(response.status_code))
            return 0
        return 1


    #In the mission we will have to set coils (plc registers) in order to get information if robot has docked and etc.
    def read_register(self, register_id):
        response = requests.get(self.HOST + 'registers/' + str(register_id), headers=self.authorization)
        register_value = 0
        if response.status_code != 200:
            print(response.status_code)
            print(response.text)

        if (response.json()['id'] == register_id):
            register_value = response.json()['value']
        return register_value

    #As above, we can set the register when loading on the robot is ready to move to main table
    def write_register(self, register_id, value):
        data = {"value": value}
        response = requests.put(self.HOST + 'registers/' + str(register_id), json = data, headers=self.authorization)

        if response.status_code != 200:
            print(response.status_code)
        return 0

robot = RestMiR()
guid = robot.get_mission("G12Mission")
#print(robot.add_mission_to_queue(guid))
charge = robot.get_mission("G9G10G11G12Recharging")
checkBat = robot.get_mission("G11BatCheck2")

while not rospy.is_shutdown():
    if signal == 1 and flag == True:
        robot.write_register(24,0)
        if robot.read_register(90) == 0:
            robot.add_mission_to_queue(guid)
            while robot.read_register(24) != 1: # wait for MIR to arrive
                print(robot.read_register(24))
            # add function to put boxes on MIR & flag to make sure we are done with packing
            sigMirPub.publish(1) #Publish that the robot has arrived
            print("Robot arrived")

            #robot.write_register(8, 0)
            #sigMirPub.publish(0) 
            flag = False
            time.sleep(2)
        else: 
            sigMirPub.publish(3) #GO TO SUSPEND WHILE WAITING FOR MIR TO CHARGE 
            print("Robot unavailable")


        ##OBS!!!Check Registers!!!
    if signal == 2 and flag == True:
        robot.add_mission_to_queue(checkBat)
        if robot.read_register(89) == 0: #If bat is higher than 30%, just continue mission
            robot.write_register(31,0) #Unset ready to charge 
            robot.write_register(24, 0)
            robot.write_register(22, 1)  # MIR can go
            #robot.write_register(1,1)
            print("Bye MIR, no recharge")
            flag = False

        if robot.read_register(89) == 1 and robot.read_register(1) == 1 and robot.read_register(11) == 1 and robot.read_register(21) == 1: #If all groups are ready to charge, and MiR bat is low, send MiR to charge
             
            robot.write_register(24, 0)
            robot.write_register(22, 1)  # MIR can go
            #robot.write_register(1,1)
            robot.write_register(90) == 1
            if robot.read_register(31)==0:
                robot.add_mission_to_queue(charge)
	    robot.write_register(31, 1) #We are ready to charge
            print("Bye MIR, go charge")
            flag = False
        elif robot.read_register(89) ==1: 
            robot.write_register(31, 1) #We are ready to charge 
            robot.write_register(24, 0)
            robot.write_register(22, 1)  # MIR can go
            print("Bye MIR, we are ready for charging")
            flag = False
        
        
    time.sleep(0.25)
