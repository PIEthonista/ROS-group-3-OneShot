#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from gtts import gTTS
import os
import json

class Main:
    def __init__(self):
        rospy.init_node('main_node')
        
        self.is_first = True
        self.RECORD_DURATION = None
        self.QUESTION = None
        self.MIXED = None
        self.VIDEO = None
        self.AUDIO = None
        
        # subscribers & publishers
        self.sub_p1_start_command = rospy.Subscriber("/p1_start_command", String, self.sequence_1)
        self.pub_p2_record_video  = rospy.Publisher("/p2_record_video",  String, queue_size=10)
        self.sub_p3_record_video  = rospy.Subscriber("/p3_record_video",  String, self.sequence_2)
        self.pub_p4_deepFace      = rospy.Publisher("/p4_deepFace",      String, queue_size=10)
        self.sub_p6_results       = rospy.Subscriber("/p6_results",       String, self.sequence_3)


    def text2audio(self, text):
        tts = gTTS(text)
        tts.save("main_audio.mp3")
        os.system("mpg321 main_audio.mp3")
        os.remove("main_audio.mp3")


    def sequence_1(self, received_payload):
        rospy.loginfo(received_payload.data)
        try:
            data = json.loads(received_payload.data)
            
            # main process ==========================
            
            if self.is_first:
                self.text2audio('Hi there! I am your helpful assistant. Practice interviewing with me!')
                self.is_first = False
                
            self.RECORD_DURATION = int(data['record_duration'])
            self.QUESTION = str(data['question'])
            
            self.text2audio(f"Your stated question is as follows: {self.QUESTION}")
            self.text2audio(f"You have {str(self.RECORD_DURATION)} seconds to answer. You may start.")
            payload = {'duration':self.RECORD_DURATION}
            
            # =======================================
            
            payload = json.dumps(payload)
            self.pub_p2_record_video.publish(payload)
        except:
            rospy.loginfo("[main_node.py] Sequence 1: Error parsing json payload.")
    
    
    def sequence_2(self, received_payload):
        rospy.loginfo(received_payload.data)
        try:
            data = json.loads(received_payload.data)
            
            # main process ==========================
            
            self.MIXED = data['mixed']
            self.VIDEO = data['video']
            self.AUDIO = data['audio']
            
            payload = {'video':self.VIDEO, 
                       'audio':self.AUDIO, 
                       'question':self.QUESTION}
            
            # =======================================
            
            payload = json.dumps(payload)
            self.pub_p4_deepFace.publish(payload)
        except:
            rospy.loginfo("[main_node.py] Sequence 2: Error parsing json payload.")


    def sequence_3(self, received_payload):
        rospy.loginfo(received_payload.data)
        try:
            data = json.loads(received_payload.data)
            
            # main process ==========================
            
            conf_score = int(data['conf'])
            summary = str(data['summary'])
            feedback = str(data['feedback'])
            
            rospy.loginfo(f"\n================================")
            rospy.loginfo(f"\nFacial Confidence Score: {conf_score}%")
            rospy.loginfo(f"\nSummary of your response: \n{summary}%")
            rospy.loginfo(f"\nFeedback for your response: \n{feedback}%")
            
            self.text2audio(f"Your facial confidence score for the practice was {str(conf_score)}%")
            self.text2audio(f"Feedback is as follows. {feedback}")
            
            # =======================================
        except:
            rospy.loginfo("[main_node.py] Sequence 3: Error parsing json payload.")



if __name__ == "__main__":
    try:
        Main()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("[main_node.py] Process Terminated.")
        