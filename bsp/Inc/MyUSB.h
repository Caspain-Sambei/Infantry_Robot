//
// Created by 18796 on 2026/2/24.
//

#ifndef GIMBAL_MYUSB_H
#define GIMBAL_MYUSB_H

#include "reg.h"
#define USB_HEAD    0xFF
#define USB_TAIL    0xFE
#define USB_RX_HEAD  0xFF
#define USB_RX_TAIL  0xFE

#define USB_FRAME_LEN    10

void USB_Send(SENDPACKET *Data);

#endif //GIMBAL_MYUSB_H