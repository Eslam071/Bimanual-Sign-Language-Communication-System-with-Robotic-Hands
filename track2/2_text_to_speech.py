from gtts import gTTS
from playsound import playsound
import rospy
from std_msgs.msg import String
import os

received_data = None
received_data_new = None

def my_callback(my_string):
    global received_data
    global received_data_new
    rospy.loginfo("%s", my_string.data)
    received_data_new = my_string.data
    try:
        if os.path.exists('text.mp3'):
            os.remove('text.mp3')

        if received_data_new is not None:
            if received_data != received_data_new:
                obj = gTTS(received_data_new, lang='ar', slow=False)
                obj.save('text.mp3')
                playsound('text.mp3')
            received_data = received_data_new
    except :
        pass


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('vision', String, my_callback)

    # Wait until the first message is received
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass