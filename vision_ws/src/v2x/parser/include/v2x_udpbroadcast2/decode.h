#include <j2735/MessageFrame.h>
#include <v2x_msgs/v2x_info.h>
int decode_avc2019(const uint8_t *buf, int len, MessageFrame_t *frame, v2x_msgs::v2x_info *msg_ptr);
