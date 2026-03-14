#include "WT901C.h"
#include "CRC_16.h"
#include "stm32h7xx_hal_uart.h"
#include <string.h>

// ================== Buffer Nhận Dữ Liệu ==================
// Buffer DMA nhận dữ liệu từ WT901C
volatile uint8_t rx_buffer_imu[RX_BUFFER_SIZE];
// Buffer tạm để xử lý trường hợp frame dữ liệu bị cắt

// ================== BUFFER LỆNH ==================
// Buffer gửi yêu cầu cập nhật góc
static const uint8_t ANGLE_RQST_BUFFER[] = {0x55, 0x00, 0x00, 0x3D, 0x00, 0x03, 0x94, 0x07};

// Buffer gửi lệnh reset góc (đối với TTL mode)
static const uint8_t tx_angle_rst_buffer[ANGLE_RESET_BUFFER_SIZE] = {
    0xFF, 0xAA, 0x52, 0x00, 0x00,
    0xFF, 0xAA, 0x52, 0x01, 0x00,
    0xFF, 0xAA, 0x52, 0x02, 0x00};

// ================== BIẾN ĐỔI GÓC YAW ==================
static float Yaw_Continuous_Angle = 0.0f;
static int16_t Yaw_Roll_Count = 0;
static float Prev_Yaw_Angle = 0.0f;
static float offset_angle = 0.0f;
static uint8_t yaw_initialized = 0;
static uint8_t yaw_zero_pending = 1;

#ifdef TTL_MODE
/**
 * @brief Tính checksum cho frame dữ liệu
 * @param frame Con trỏ tới frame dữ liệu
 * @param length Độ dài của frame (bao gồm checksum cuối)
 * @return Giá trị checksum tính được
 */
static uint8_t computeChecksum(const uint8_t *frame, uint8_t length)
{
    uint8_t cs = 0;
    for (uint8_t i = 0; i < length - 1; i++)
        cs += frame[i];
    return cs;
}
#endif

/**
 * @brief Cập nhật giá trị góc yaw liên tục (không bị giới hạn trong [0,360])
 * @param current_angle Góc yaw hiện tại
 */
static void Infinite_Yaw(float current_angle)
{
    if (!yaw_initialized)
    {
        Prev_Yaw_Angle = current_angle;
        Yaw_Continuous_Angle = current_angle;
        yaw_initialized = 1;
        return;
    }

    float delta = current_angle - Prev_Yaw_Angle;
    if (delta > 180.0f)
        Yaw_Roll_Count--;
    else if (delta < -180.0f)
        Yaw_Roll_Count++;

    Yaw_Continuous_Angle = current_angle + (Yaw_Roll_Count * 360.0f);
    Prev_Yaw_Angle = current_angle;
}

// ================== KHỞI TẠO VÀ HIỆN THỰC ==================

void WT901C_Init(WT901C *imu, UART_HandleTypeDef *wt901_uart)
{
    imu->WT901C_UART = wt901_uart;
    imu->Yaw = imu->Roll = imu->Pitch = 0.0f;

    WT901C_Calibrate_Accel(imu);
    HAL_Delay(1000);
    WT901C_Reset_Angles(imu);
    HAL_Delay(100);
}

void WT901C_Reset_Angles(WT901C *imu)
{
    WT901C_Stop_Recieve(imu);
#ifdef TTL_MODE
    // Gửi lệnh reset góc theo từng phần (mỗi phần 5 byte)
    for (int i = 0; i < ANGLE_RESET_BUFFER_SIZE; i += 5)
    {
        HAL_UART_Transmit(imu->WT901C_UART, (uint8_t *)(tx_angle_rst_buffer + i), 5, 100);
        HAL_Delay(1);
    }
#elif defined RS485_MODE
    HAL_UART_Transmit_DMA(imu->WT901C_UART, (uint8_t *)tx_angle_rst_buffer, ANGLE_RESET_BUFFER_SIZE);
#endif

    // Reset bo loc yaw, dat moc zero tai frame hop le dau tien sau reset
    offset_angle = 0.0f;
    Yaw_Roll_Count = 0;
    Prev_Yaw_Angle = 0.0f;
    Yaw_Continuous_Angle = 0.0f;
    yaw_initialized = 0;
    yaw_zero_pending = 1;
    imu->Yaw = imu->Roll = imu->Pitch = 0.0f;

    WT901C_Begin_Recieve(imu);
}

void WT901C_Start_Receive(WT901C *imu)
{
    HAL_UART_Receive_DMA(imu->WT901C_UART, rx_buffer_imu, RX_BUFFER_SIZE);
}

void WT901C_Begin_Recieve(WT901C *imu)
{
#ifdef TTL_MODE
    WT901C_Start_Receive(imu);
#elif defined RS485_MODE
    WT901C_Angle_Request(imu);
#endif
}

void WT901C_Stop_Recieve(WT901C *imu)
{
    HAL_UART_DMAStop(imu->WT901C_UART);
}

#ifdef RS485_MODE
void WT901C_Angle_Request(WT901C *imu)
{
    HAL_UART_Transmit_DMA(imu->WT901C_UART, (uint8_t *)ANGLE_RQST_BUFFER, sizeof(ANGLE_RQST_BUFFER));
    HAL_UART_Receive_DMA(imu->WT901C_UART, rx_buffer_imu, RX_BUFFER_SIZE);
}
#endif

/**
 * @brief Xử lý buffer dữ liệu nhận được từ UART, tách các frame hợp lệ và cập nhật các góc.
 * @param imu Con trỏ tới cấu trúc WT901C
 * @param buffer Buffer dữ liệu nhận được
 * @param length Số byte nhận được
 */
void WT901C_Process_Buffer(WT901C *imu, uint8_t *buffer, uint16_t length)
{
    uint8_t tempBuffer[RX_BUFFER_SIZE + FRAME_LEN];
    uint16_t totalLength = 0;

    // Nếu có dữ liệu dư từ lần nhận trước, copy sang tempBuffer
    if (overflowLength > 0)
    {
        memcpy(tempBuffer, overflowBuffer, overflowLength);
        totalLength = overflowLength;
        overflowLength = 0;
    }

    // Copy dữ liệu mới nhận vào tempBuffer
    memcpy(&tempBuffer[totalLength], buffer, length);
    totalLength += length;

    uint16_t i = 0;
    while (i <= totalLength - FRAME_LEN)
    {
        if (tempBuffer[i] == 0x55 && tempBuffer[i + 1] == 0x53)
        {
#ifdef TTL_MODE
            if (computeChecksum(&tempBuffer[i], FRAME_LEN) == tempBuffer[i + FRAME_LEN - 1])
#else
            uint16_t frameCrc = (tempBuffer[i + FRAME_LEN - 1] << 8) | tempBuffer[i + FRAME_LEN - 2];
            if (CRC_16(&tempBuffer[i], FRAME_LEN - 2) == frameCrc)
#endif
            {
                // Giải mã góc Roll, Pitch và Yaw
                imu->Roll = ((int16_t)((tempBuffer[i + 3] << 8) | tempBuffer[i + 4])) * (180.0f / 32768.0f);
                imu->Pitch = ((int16_t)((tempBuffer[i + 5] << 8) | tempBuffer[i + 6])) * (180.0f / 32768.0f);
                float currentYaw = ((int16_t)((tempBuffer[i + 7] << 8) | tempBuffer[i + 8])) * (180.0f / 32768.0f);
                Infinite_Yaw(currentYaw);
                if (yaw_zero_pending)
                {
                    offset_angle = Yaw_Continuous_Angle;
                    yaw_zero_pending = 0;
                }
                imu->Yaw = Yaw_Continuous_Angle - offset_angle;
                i += FRAME_LEN;
                continue;
            }
        }
        i++;
    }
    // Lưu lại dữ liệu dư (nếu frame chưa hoàn chỉnh) để xử lý ở lần nhận sau
    if (i < totalLength)
    {
        overflowLength = totalLength - i;
        memcpy(overflowBuffer, &tempBuffer[i], overflowLength);
    }
}

/**
 * @brief Callback xử lý khi UART nhận được trạng thái IDLE.
 * @param imu Con trỏ tới cấu trúc WT901C
 */
void WT901C_UART_Rx_IDLE_Hanlde(WT901C *imu)
{
    uint16_t rx_length = RX_BUFFER_SIZE;

    if ((imu != NULL) && (imu->WT901C_UART != NULL) && (imu->WT901C_UART->hdmarx != NULL))
    {
        uint32_t remain = __HAL_DMA_GET_COUNTER(imu->WT901C_UART->hdmarx);
        if (remain <= RX_BUFFER_SIZE)
        {
            rx_length = (uint16_t)(RX_BUFFER_SIZE - remain);
        }
    }

    if (rx_length > 0u)
    {
        WT901C_Process_Buffer(imu, (uint8_t *)rx_buffer_imu, rx_length);
    }

#ifdef TTL_MODE
    memset((void *)rx_buffer_imu, 0, sizeof(rx_buffer_imu));
    WT901C_Start_Receive(imu);
#endif
}

/**
 * @brief Hiệu chuẩn gia tốc của cảm biến WT901C.
 * @param imu Con trỏ tới cấu trúc WT901C
 */
void WT901C_Calibrate_Accel(WT901C *imu)
{
    const uint8_t cmd1[] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
    const uint8_t cmd2[] = {0xFF, 0xAA, 0x01, 0x01, 0x00};
    const uint8_t cmd3[] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
    const uint8_t cmd4[] = {0xFF, 0xAA, 0x00, 0x00, 0x00};

    WT901C_Stop_Recieve(imu);

    HAL_UART_Transmit(imu->WT901C_UART, cmd1, sizeof(cmd1), 100);
    HAL_Delay(100);
    HAL_UART_Transmit(imu->WT901C_UART, cmd2, sizeof(cmd2), 100);
    HAL_Delay(10000);
    HAL_UART_Transmit(imu->WT901C_UART, cmd3, sizeof(cmd3), 100);
    HAL_Delay(100);
    HAL_UART_Transmit(imu->WT901C_UART, cmd4, sizeof(cmd4), 100);
    HAL_Delay(100);
}
