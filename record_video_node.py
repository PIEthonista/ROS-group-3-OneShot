#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import json
import os
import cv2
import wave
import pyaudio
import threading
import subprocess


class RecordVideo:
    def __init__(self):
        rospy.init_node('record_video_node')
        
        self.RECORD_DURATION = None
        self.FPS               = 30.0   # default ffmpeg FPS, do not change
        self.FRAME_SAMPLE_RATE = 2      # samples a frame from every N frames for emotion detection
        self.CAM_ID            = 0      # device specific, say if you have 3 cameras connected, please try 0 | 1 | 2
        self.OUTPUT_DIR        = os.getcwd()
        self.FILE_NAMES        = {'mixed':'mixed.mp4', 
                                  'video':'vid_only.avi', 
                                  'audio':'aud_only.wav'
                                 }     # file formats are specific, do not change
        
        # subscribers & publishers
        self.sub_p2_record_video = rospy.Subscriber("/p2_record_video", String, self.handle_payload)
        self.pub_p3_record_video = rospy.Publisher("/p3_record_video", String, queue_size=10)


    def handle_payload(self, received_payload):
        rospy.loginfo(received_payload.data)
        try:
            data = json.loads(received_payload.data)
            
            # main process ==========================
            
            # Step 1: record video, audio, & mix them up
            self.RECORD_DURATION = int(data['duration'])
            payload = self.record_video_and_audio()
            
            # =======================================
            
            payload = json.dumps(payload)
            self.pub_p3_record_video.publish(payload)
        except:
            rospy.loginfo("[record_video_node.py] Error parsing json payload.")
    
    
    # main driver method for combined video and audio recording, & mixing
    def record_video_and_audio(self):
        
        MIXED_VID_AUD_FILE = os.path.join(self.OUTPUT_DIR, self.FILE_NAMES['mixed'])
        TMP_VID_FILE       = os.path.join(self.OUTPUT_DIR, self.FILE_NAMES['video'])
        TMP_AUD_FILE       = os.path.join(self.OUTPUT_DIR, self.FILE_NAMES['audio'])
        
        video_thread = threading.Thread(target=self.record_video_util, args=(self.RECORD_DURATION, TMP_VID_FILE))
        audio_thread = threading.Thread(target=self.record_audio_util, args=(self.RECORD_DURATION, TMP_AUD_FILE))

        video_thread.start()
        audio_thread.start()

        video_thread.join()
        audio_thread.join()

        # # combine video and audio using ffmpeg
        # cmd = f'ffmpeg -y -i {TMP_VID_FILE} -i {TMP_AUD_FILE} -c:v copy -c:a aac -strict experimental {MIXED_VID_AUD_FILE}'
        # subprocess.call(cmd, shell=True)
        
        return {'mixed': MIXED_VID_AUD_FILE, 'video':TMP_VID_FILE, 'audio':TMP_AUD_FILE}
    
    
    # utility method for video only recording
    def record_video_util(self, duration, output_file='output.mp4'):
        
        cap = cv2.VideoCapture(self.CAM_ID)
        
        fourcc = cv2.VideoWriter_fourcc(*'XVID')   # video codec, machine specific
        
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        out = cv2.VideoWriter(output_file, fourcc, self.FPS, (w,h))

        start_time = cv2.getTickCount()
        while(int((cv2.getTickCount() - start_time)/cv2.getTickFrequency()) < duration):
            ret, frame = cap.read()
            if not ret:
                rospy.loginfo("Can't receive frame (stream end?). Exiting ...")
                break
            frame = cv2.flip(frame, 1)  # flip frame, default cv2 capture is mirrored
            out.write(frame)
            
            # cv2.imshow('frame', frame)    # TODO: fix can't run imshow in python multithreading
            if cv2.waitKey(1) == ord('q'):
                break

        cap.release()
        out.release()
        cv2.destroyAllWindows()


    # utility method for audio only recording
    def record_audio_util(self, duration, audio_file):
        
        CHUNK         = 1024  # record in chunks of 1024 samples
        SAMPLE_FORMAT = pyaudio.paInt16  # 16 bits per sample
        CHANNELS      = 1
        FS            = 44100  # record at 44100 samples per second

        p = pyaudio.PyAudio()

        stream = p.open(format=SAMPLE_FORMAT,
                        channels=CHANNELS,
                        rate=FS,
                        frames_per_buffer=CHUNK,
                        input=True)

        frames = []
        for _ in range(0, int(FS / CHUNK * duration)):
            data = stream.read(CHUNK)
            frames.append(data)

        stream.stop_stream()
        stream.close()
        p.terminate()

        wf = wave.open(audio_file, 'wb')
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(p.get_sample_size(SAMPLE_FORMAT))
        wf.setframerate(FS)
        wf.writeframes(b''.join(frames))
        wf.close()


if __name__ == "__main__":
    try:
        RecordVideo()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("[record_video_node.py] Process Terminated.")
        

