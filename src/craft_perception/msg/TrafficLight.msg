# Traffic-light blink count (발제 04): one message per accepted blink and on reset.
std_msgs/Header header       # stamp of the frame that closed the last blink
uint8 RED=0
uint8 YELLOW=1
uint8 GREEN=2
int32 red
int32 yellow
int32 green
uint8[] sequence             # accepted blinks in order (RED/YELLOW/GREEN)
bool complete                # the light went dark (end_off_s) after 12 valid blinks; counting stopped
bool valid                   # 12 blinks and each color 1..6 (true whenever complete)
int32 discarded_cycles       # cycles that ended with a wrong count and were recounted
