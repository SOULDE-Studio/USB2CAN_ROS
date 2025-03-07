#include <stdint.h>

#ifdef __cplusplus
extern "C"{
#endif
#define STANDARD 0
#define EXTENDED 1
#pragma pack(1)
typedef struct FrameInfo {
    uint32_t canID; // CAN frame ID
    uint8_t frameType; // 0 for Standard frame and 1 for Extended frame
    uint8_t dataLength; //CAN frame data length
} FrameInfo;
#pragma pack()

//!
//! \param devName "/dev/USB2CAN*" in Linux or "COM*" in Windows
//! \return device handler for further operations
int32_t openUSBCAN(const char *devName);

//!
//! \param dev device handler returned by openUSBCAN
//! \return serial close return value
int32_t closeUSBCAN(int32_t dev);

//!
//! \param dev device handler
//! \param channel the CAN transmitter used to send data, USB2CAN-CAN-Dual can choose 1 or 2.
//! \param info the pointer of FrameInfo structure, which contains the CAN ID, frame type and data length
//! \param data CAN frame data
//! \return transmit return value, 8 for success and -1 for failure
int32_t sendUSBCAN(int32_t dev, uint8_t channel, FrameInfo *info, uint8_t *data);

//!
//! \param dev device handler
//! \param channel the CAN transmitter received data, USB2CAN-CAN-Dual can be 1 or 2.
//! \param info the pointer of FrameInfo structure, which contains the CAN ID, frame type and data length of the received frame
//! \param data CAN frame data received
//! \param timeout timeout value in us
//! \return -1 for time out and 0 for success
int32_t readUSBCAN(int32_t dev, uint8_t *channel, FrameInfo *info, uint8_t *data, int32_t timeout);


#ifdef __cplusplus
}
#endif
