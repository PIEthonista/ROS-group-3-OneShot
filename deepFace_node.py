#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import json
import cv2
import numpy as np
from deepface import DeepFace


class deepFace:
    def __init__(self):
        rospy.init_node('deepFace_node')
        
        self.FRAME_SAMPLE_RATE = 2 # samples a frame from every N frames for emotion detection
        
        # subscribers & publishers
        self.sub_p4_deepFace  = rospy.Subscriber("/p4_deepFace",  String, self.handle_payload)
        self.pub_p5_text_eval = rospy.Publisher("/p5_text_eval", String, queue_size=10)


    def handle_payload(self, received_payload):
        rospy.loginfo(received_payload.data)
        try:
            data = json.loads(received_payload.data)
            
            # main process ==========================
            
            # Step 2: sample frames and analyze emotions
            frames = self.sample_frames(data['video'], sample_rate=self.FRAME_SAMPLE_RATE)
            rospy.loginfo(f"Processing {len(frames)} frames...")
            emotions = self.detect_face_emotions(frames)
            conf_scores = self.process_emotions(emotions)
            
            payload = {'conf': int(conf_scores['conf']),
                       'question': str(data['question']),
                       'audio': str(data['audio'])}
            
            # =======================================
            
            payload = json.dumps(payload)
            self.pub_p5_text_eval.publish(payload)
        except:
            rospy.loginfo("[deepFace_node.py] Error parsing json payload.")


    def sample_frames(self, video_file, sample_rate=2):
        cap = cv2.VideoCapture(video_file)
        frames = []
        count = 0

        while(cap.isOpened()):
            ret, frame = cap.read()
            if not ret:
                break
            if count % sample_rate == 0:
                # frames.append(frame)
                frames.append(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            count += 1
        cap.release()
        
        return frames


    # run deepface model for emotion detection (local)
    def detect_face_emotions(self, frames):
        emotions = []
        for frame in frames:
            frame_result = DeepFace.analyze(frame, actions=['emotion'], enforce_detection=False)
            emotions.append(frame_result)
            
        return emotions


    def process_emotions(self, emotions):
        count = 0
        emots = {'sad':0, 
                'angry':0, 
                'surprise':0, 
                'fear':0, 
                'happy':0, 
                'disgust':0, 
                'neutral':0}

        for frame_result in emotions:
            if len(frame_result) > 0:
                emot = frame_result[0]['emotion']
                emots['sad'] = emots.get('sad', 0) + emot['sad']
                emots['angry'] = emots.get('angry', 0) + emot['angry']
                emots['surprise'] = emots.get('surprise', 0) + emot['surprise']
                emots['fear'] = emots.get('fear', 0) + emot['fear']
                emots['happy'] = emots.get('happy', 0) + emot['happy']
                emots['disgust'] = emots.get('disgust', 0) + emot['disgust']
                emots['neutral'] = emots.get('neutral', 0) + emot['neutral']
                count += 1
                
        # prevent zero division
        if count == 0: count = 1
        
        for i in list(emots.keys()):
            emots[i] /= (count*100)

        dominant = 'sad'
        for i in list(emots.keys()):
            if emots[i] > emots[dominant]:
                dominant = i
        emots["dominant"] = dominant

        # refactor according to custom weightage
        sad_score      = emots['sad']*1.3
        angry_score    = emots['angry']*1.3
        surprise_score = emots['surprise']*1.4
        fear_score     = emots['fear']*1.3
        happy_score    = emots['happy']*1.7
        disgust_score  = emots['disgust']*10
        neutral_score  = emots['neutral']/1.2
        
        score_list = [sad_score,angry_score,surprise_score,fear_score,happy_score,disgust_score,neutral_score]
        min_value = min(score_list)
        max_value = max(score_list)
        normalized_scores = [(score - min_value) / (max_value - min_value) for score in score_list]
        mean = np.mean(normalized_scores)
        
        result_scores = [(-sad_score), (-angry_score), surprise_score, (-fear_score), happy_score, (-disgust_score), neutral_score]
        min_value = min(result_scores)
        max_value = max(result_scores)
        normalized_result_scores = [(score - min_value) / (max_value - min_value) for score in result_scores]
        result = np.mean(normalized_result_scores)
        
        difference = abs((mean-result)/mean)*100
        # maintain between 0-100
        if difference>50:
            difference = 50
        
        if mean>result:
            conf = 50-difference
        else:
            conf = 50+difference

        return {"mean":mean, "result":result, "conf":conf}



if __name__ == "__main__":
    try:
        deepFace()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("[deepFace_node.py] Process Terminated.")
        