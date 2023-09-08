#!/usr/bin/env python
import rospy
import whisper
import gradio as gr
import speech_recognition as sr
from std_msgs.msg import String, Bool

model = whisper.load_model("small")
microphone_active = False  # 마이크로폰 활성화 변수 추가

def transcribe(audio):
    audio = whisper.load_audio(audio)
    audio = whisper.pad_or_trim(audio)
    mel = whisper.log_mel_spectrogram(audio).to(model.device)
    _, probs = model.detect_language(mel)
    detected_language = max(probs, key=probs.get)
    options = whisper.DecodingOptions(fp16=False)
    result = whisper.decode(model, mel, options)
    return result.text, detected_language

def microphone_active_callback(data):  # 마이크로폰 활성화 콜백 함수
    global microphone_active
    microphone_active = data.data

def timer_callback(event):
    # 주기적으로 마이크로폰 상태 업데이트를 수신                           
    rospy.Subscriber('microphone', Bool, microphone_active_callback)

def talker():
    global microphone_active
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rate = rospy.Rate(1)  # 1hz
    r = sr.Recognizer()
    microphone_timeout = 5

    # 1초마다 `timer_callback` 함수를 호출하여 마이크로폰 상태를 업데이트
    rospy.Timer(rospy.Duration(1), timer_callback)

    while not rospy.is_shutdown():
        if microphone_active:  # microphone_active가 True일 때만 마이크로폰 입력을 받음
            with sr.Microphone() as source:
                print("Say something!")
                audio = r.listen(source)
            with open("/home/junho/catkin_ws/src/whisper/scripts/microphone.wav", "wb") as f:
                f.write(audio.get_wav_data())

            result_text, detected_language = transcribe("/home/junho/catkin_ws/src/whisper/scripts/microphone.wav")
            microphone_counter = 0

            while microphone_counter < microphone_timeout:
                rospy.loginfo(result_text)
                pub.publish(result_text)
                microphone_counter += 1
                rospy.sleep(1)
            microphone_active = False
        else:
            rospy.loginfo("Microphone is not active.")
            pub.publish("Microphone is not active.")

        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('microphone_active')
    try:
        talker()
        rospy.spin()  # ROS 노드 실행
    except rospy.ROSInterruptException:
        pass

