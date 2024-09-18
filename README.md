# Bimanual-Sign-Language-Communication-System-with-Robotic-Hands

## Project Overview

The **Bimanual Sign Language Communication System** enables communication between Deaf individuals and normal people. It has two main tracks:

1. **Track 1**: Converts spoken language into sign language using robotic arms, helping normal people communicate with the Deaf.
2. **Track 2**: Detects sign language gestures from Deaf individuals and translates them into audible speech for reciprocal communication.

## Key Features

- **Track 1 (Robotic Arm to Sign Language)**:
  - Uses robotic arms and simulation to translate spoken language into sign language.
  - Uses Voice recognition to translate the speech into a message that can be understood.

- **Track 2 (Sign Language to Speech)**:
  - A camera system captures and interprets sign language gestures.
  - Sign language is converted into audible words for normal people.

- **ROS Integration**: Handles communication between the robotic hand, simulation, vision and voice recognition to ensure real-time functionality.

## Team Structure

- **Kareem Salah**: Robot Control & Communication
- **Eslam Wael**: Robot Control & Communication
- **Ziad Yasser**: Computer Vision
- **Mohammed Younes**: Voice Recognition

## Folder Structure

### Track 1:
- **`hand_urdf/`**: Contains URDF files for the robotic hand (final version: `hand_urdf_error`).
- **`moveit_hand2/`**: Package generated from MoveIt Setup Assistant.
- **Real-time Nodes** (in the `joint_state` folder): Nodes that take values from the real `/joint_state` topic.
- **ROS Node Files**:
  1. **`1_speech_to_text.py`**: Receives spoken input and publishes the corresponding string.
  2. **`2_text_to_move.py`**: Receives the string, plans and executes the corresponding motion, and publishes it to the math model.
  3. **`3_math_model.py`**: Converts 16 joint angles into 6 motor values.
  4. **`4_multiservo.ino`**: Receives the 6 motor values and sends them to the motor.

### Track 2:
- **`best_5.pt`**: Pre-trained model for vision-based sign language detection.
- **`Hacen-Liner-Print-out.ttf`**: Font used in the computer vision code.
- **ROS Node Files**:
  1. **`1_yolo_Node_A.py`**: Opens the camera, detects sign language gestures, and publishes the detected sign as a string.
  2. **`2_text_to_speech.py`**: Receives the string of the sign and converts it into speech.

