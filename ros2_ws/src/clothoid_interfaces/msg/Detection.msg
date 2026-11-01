# One YOLO detection. Pixel coordinates are in the source image
# (/camera/color/image_raw, 640x480), origin at the top-left corner.
#
# class_name is one of:
#   traffic_light_red, traffic_light_yellow, traffic_light_green,
#   sign_left, sign_right, sign_pedestrian, sign_slow, sign_parking
string class_name
float32 confidence
int32 x_min
int32 y_min
int32 x_max
int32 y_max
