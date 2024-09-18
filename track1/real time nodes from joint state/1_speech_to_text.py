from gtts import gTTS
from playsound import playsound
import speech_recognition as sr
import rospy
from std_msgs.msg import String

def publisher_node():
    rospy.init_node('publisher_node', anonymous=True)
    pub = rospy.Publisher('voice', String, queue_size=10)
    rate = rospy.Rate(10)  # 1 Hz

    while not rospy.is_shutdown():
        r = sr.Recognizer()
        t = None 
        try:
            with sr.Microphone() as src:
                print('Say something, brother')
                audio = r.record(src, duration=3)

            t =  r.recognize_google(audio, language='ar-AR')
            print(t)

            #obj = gTTS(text=t, lang='ar', slow=False)
            #obj.save('text.mp3')
            #playsound('text.mp3')

        except sr.UnknownValueError as U:
            # Handle cases where speech cannot be recognized
            print(U)

        except sr.RequestError as R:
            # Handle cases where there is an issue with the recognition service (e.g., no internet connection)
            print(R)

        rospy.loginfo("Publishing: {}".format(t))
        pub.publish(t)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher_node()
    except rospy.ROSInterruptException:
        pass
