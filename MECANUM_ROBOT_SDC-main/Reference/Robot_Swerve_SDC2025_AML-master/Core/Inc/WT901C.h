// Cần cấu hình UART mode Rx DMA normal!

#ifndef WT901C_H
#define WT901C_H

#include "stdint.h"

/*================= Board Selection =================*/
#ifdef STM32H7
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_uart.h"
#elif defined(STM32F4)
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"
#endif

/*================= Configuration Defines =================*/
// Maximum time (ms) to wait for request (sử dụng khi cần)
#define WT901C_REQ_MAX_TIME 100

// Chọn giao thức truyền thông: TTL hoặc RS485
#define TTL_MODE
// #define RS485_MODE

/*================= Helper Macros =================*/
// Lấy byte thấp và byte cao từ 16-bit word
inline uint8_t Low_Byte(uint16_t w) { return (uint8_t)(w & 0xFF); }
inline uint8_t High_Byte(uint16_t w) { return (uint8_t)(w >> 8); }

/*================= Data Structure =================*/
/**
 * @brief Cấu trúc dữ liệu cho cảm biến WT901C
 */
typedef struct
{
  float Roll;                      // Góc Roll (độ)
  float Pitch;                     // Góc Pitch (độ)
  float Yaw;                       // Góc Yaw (độ)
  UART_HandleTypeDef *WT901C_UART; // Con trỏ tới UART handler
} WT901C;

/*================= Protocol Frame Defines =================*/
// Dành cho giao thức MODBUS RTU (RS485)
#define WT901C_ADDR 0x01 // Địa chỉ thiết bị (cập nhật nếu cần)
#define READ_FUNC 0x03
#define WRITE_FUNC 0x06

#ifdef RS485_MODE
#define RX_BUFFER_SIZE 11
#define ANGLE_RESET_BUFFER_SIZE 24
#define ANGLE_RESET_COMMAND {                                    \
    WT901C_ADDR, WRITE_FUNC, 0x00, 0x69, 0xB5, 0x88, 0x2F, 0x20, \
    WT901C_ADDR, WRITE_FUNC, 0x00, 0x01, 0x00, 0x04, 0xD9, 0xC9, \
    WT901C_ADDR, WRITE_FUNC, 0x00, 0x00, 0x00, 0x00, 0x89, 0xCA}
#endif

#ifdef TTL_MODE
// Cấu hình frame cho TTL UART
#define RX_BUFFER_SIZE 25
#define ANGLE_RESET_BUFFER_SIZE 15
#define FRAME_LEN 11
#define ANGLE_RESET_COMMAND {     \
    0xFF, 0xAA, 0x69, 0x88, 0xB5, \
    0xFF, 0xAA, 0x01, 0x04, 0x00, \
    0xFF, 0xAA, 0x00, 0x00, 0x00}

// Buffer phục vụ khôi phục frame bị cắt 
static uint8_t overflowBuffer[FRAME_LEN];
static uint8_t overflowLength = 0;
#endif

/*================= Public Function Prototypes =================*/

/**
 * @brief Khởi tạo cảm biến WT901C với UART handler và bắt đầu nhận dữ liệu
 * @param imu Con trỏ tới cấu trúc WT901C
 * @param wt901_uart Con trỏ tới UART handler sử dụng cho WT901C
 */
void WT901C_Init(WT901C *imu, UART_HandleTypeDef *wt901_uart);

/**
 * @brief Reset các góc của cảm biến WT901C
 * @param imu Con trỏ tới cấu trúc WT901C
 */
void WT901C_Reset_Angles(WT901C *imu);

#ifdef RS485_MODE
/**
 * @brief Gửi yêu cầu cập nhật góc (sử dụng trong Timer interrupt)
 * @param imu Con trỏ tới cấu trúc WT901C
 */
void WT901C_Angle_Request(WT901C *imu);
#endif

/**
 * @brief Bắt đầu nhận dữ liệu từ cảm biến
 * @param imu Con trỏ tới cấu trúc WT901C
 */
void WT901C_Begin_Recieve(WT901C *imu);

/**
 * @brief Xử lý callback khi UART nhận được trạng thái IDLE
 * @param imu Con trỏ tới cấu trúc WT901C
 */
void WT901C_UART_Rx_IDLE_Hanlde(WT901C *imu);

/**
 * @brief Hiệu chuẩn gia tốc của cảm biến WT901C
 * @param imu Con trỏ tới cấu trúc WT901C
 */
void WT901C_Calibrate_Accel(WT901C *imu);

/**
 * @brief Dừng nhận dữ liệu từ cảm biến
 * @param imu Con trỏ tới cấu trúc WT901C
 */
void WT901C_Stop_Recieve(WT901C *imu);

#endif // WT901C_H
