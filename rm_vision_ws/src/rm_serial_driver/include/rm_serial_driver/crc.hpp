// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__CRC_HPP_
#define RM_SERIAL_DRIVER__CRC_HPP_

#include <cstdint>

namespace crc8
{
/**
  * @brief CRC8 Caculation function (polynomial 0x07)
  * @param[in] data : Data to calculate
  * @param[in] len : Data length (excluding CRC byte)
  * @return : CRC8 checksum
  */
uint8_t Get_CRC8_Check_Sum(const uint8_t * data, uint32_t len);

/**
  * @brief Append CRC8 value to the end of the buffer
  * @param[in] data : Data buffer
  * @param[in] len : Total length (including CRC byte position)
  * @return none
  */
void Append_CRC8_Check_Sum(uint8_t * data, uint32_t len);

}  // namespace crc8

namespace crc16
{
/**
  * @brief CRC16 Verify function
  * @param[in] pchMessage : Data to Verify,
  * @param[in] dwLength : Stream length = Data + checksum
  * @return : True or False (CRC Verify Result)
  */
uint32_t Verify_CRC16_Check_Sum(const uint8_t * pchMessage, uint32_t dwLength);

/**
  * @brief Append CRC16 value to the end of the buffer
  * @param[in] pchMessage : Data to Verify,
  * @param[in] dwLength : Stream length = Data + checksum
  * @return none
  */
void Append_CRC16_Check_Sum(uint8_t * pchMessage, uint32_t dwLength);

}  // namespace crc16

#endif  // RM_SERIAL_DRIVER__CRC_HPP_
