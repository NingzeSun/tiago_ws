# Operator state from the Quest, published by quest_webxr_node once per client packet.
# Poses are in the headset floor frame with WebXR axes (x right, y up, z toward the operator),
# like /quest/{left,right}/pose. A group whose *_valid flag is false holds zeros.
std_msgs/Header header          # bridge receive time; frame_id quest_local_floor
string client                   # "webxr" or "unity <headset>"
float64 headset_time            # headset clock, s

bool head_valid
geometry_msgs/Pose head         # between the eyes, looking along -z

bool eyes_valid                 # both eyes tracked
geometry_msgs/Pose left_eye     # gaze origin; the gaze points along the pose's -z
geometry_msgs/Pose right_eye
float32 left_eye_confidence     # 0..1
float32 right_eye_confidence

bool gaze_valid                 # the combined gaze hits a camera tile of the headset mosaic
string gaze_camera              # head_camera | left_camera | right_camera
float32[2] gaze_pixel           # (u, v) in that camera's image, px

bool face_valid
string[] face_names             # Meta face expression names (constant per client)
float32[] face_weights          # 0..1, in face_names order
