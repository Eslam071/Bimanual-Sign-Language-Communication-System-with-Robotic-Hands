#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from ultralytics import YOLO
import cv2
import numpy as np
import torch
from PIL import ImageFont, ImageDraw, Image

# Define labels for detection
label = ["ع", "ال", "ه", "ح", "ج", "ك", "خ", "احبك", "ل", "م", "ن", "ر", "أ", "ص", "س", "ش", "ت", "ط", "ث", "ذ", "ة", "و", "يا", "ب", "ي", "ز", "د", "ظ", "ض", "ف", "ق", "غ"]

# Initialize the webcam
cap = cv2.VideoCapture(2)
cap.set(3, 640)  # Set width
cap.set(4, 480)  # Set height

device = 'cuda' if torch.cuda.is_available() else 'cpu'
model = YOLO("/home/eslam/catkin_workspace/src/t2/scripts/best_5.pt")  # Change to your model path

current_class_label = ""

def detection_loop():
    global current_class_label
    while not rospy.is_shutdown():
        success, img = cap.read()
        if not success:
            break

        # Resize image to meet the input requirements of the model
        resized_img = cv2.resize(img, (640, 640))

        # Convert the image to the correct format and device
        img_tensor = torch.from_numpy(resized_img).permute(2, 0, 1).unsqueeze(0).float().to(device)  # CHW to BCHW
        img_tensor /= 255.0

        results = model.predict(img_tensor)

        for r in results:
            for box in r.boxes:
                label_text = label[int(box.cls)]
                confidence = box.conf.item()

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                label_with_conf = f"{label_text} {confidence:.2f}%"

                # Draw the bounding box and label on the resized_img directly
                cv2.rectangle(resized_img, (x1, y1), (x2, y2), (255, 0, 255), 3)
                cv2.putText(resized_img, label_with_conf, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

                current_class_label = label[int(box.cls)]
                font_size = 36
                font_color = (0, 0, 255)
                font_path = "/home/eslam/catkin_workspace/src/t2/scripts/Hacen-Liner-Print-out.ttf"  # Change to your font path
                font = ImageFont.truetype(font_path, font_size)

                pil_img = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
                draw = ImageDraw.Draw(pil_img)
                draw.text((x1, y1 - 30), current_class_label, font=font, fill=font_color)
                cv_img_with_text = cv2.cvtColor(np.array(pil_img), cv2.COLOR_RGB2BGR)
                cv2.imshow("Image", cv_img_with_text)
                print("Detected Class label:", current_class_label)
                print("Detected Class number:", int(box.cls))

        # Display the image
        cv2.imshow("YOLO Object Detection", resized_img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

# define a function to publish the current_class_label in the required time without affecting the FPS
def publish_class_label(event):
    global current_class_label
    rospy.loginfo("Published: %s", current_class_label)
    pub.publish(current_class_label)

if __name__ == '__main__':
    try:
        rospy.init_node('publisher_node', anonymous=True)
        pub = rospy.Publisher('vision', String, queue_size=10)
        rospy.Timer(rospy.Duration(1.5), publish_class_label)  # Timer to call publish_class_label function every specific period
        detection_loop()
    except rospy.ROSInterruptException:
        pass
