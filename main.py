#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import json

def main():
    rospy.init_node('main', anonymous=True)
    pub_p1_start_command = rospy.Publisher('/p1_start_command', String, queue_size=10)

    while not rospy.is_shutdown():
        print("===================")
        input_question = str(input("Enter the question you would like to practice: "))
        duration = input("Enter practice duration (seconds): ")
        try:
            duration = int(duration)
            data = {'question':input_question, 'record_duration': duration}
            json_data = json.dumps(data)
            pub_p1_start_command.publish(json_data)
        except:
            print("[main.py] Error, please try again")
            continue


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass