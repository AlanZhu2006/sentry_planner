#include "bsp_can.h"

#include "bsp_dwt.h"
#include "bsp_log.h"

#include <stdlib.h>
#include <string.h>

static CANInstance *can_instance[CAN_MX_REGISTER_CNT];
static uint8_t instance_count;

static uint32_t CANLengthToDlc(uint8_t length)
{
    static const uint32_t dlc[9] = {
        FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2,
        FDCAN_DLC_BYTES_3, FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5,
        FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7, FDCAN_DLC_BYTES_8,
    };
    return dlc[length];
}

static uint8_t CANDlcToLength(uint32_t dlc)
{
    return (uint8_t)(dlc & 0x0FU);
}

static HAL_StatusTypeDef CANConfigureReceive(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_FilterTypeDef filter = {
        .IdType = FDCAN_STANDARD_ID,
        .FilterIndex = 0,
        .FilterType = FDCAN_FILTER_MASK,
        .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
        .FilterID1 = 0,
        .FilterID2 = 0,
    };

    if (HAL_FDCAN_ConfigFilter(hfdcan, &filter) != HAL_OK)
        return HAL_ERROR;

    return HAL_FDCAN_ConfigGlobalFilter(hfdcan,
                                        FDCAN_REJECT,
                                        FDCAN_REJECT,
                                        FDCAN_REJECT_REMOTE,
                                        FDCAN_REJECT_REMOTE);
}

static HAL_StatusTypeDef CANStartBus(FDCAN_HandleTypeDef *hfdcan)
{
    if (CANConfigureReceive(hfdcan) != HAL_OK)
        return HAL_ERROR;
    if (HAL_FDCAN_Start(hfdcan) != HAL_OK)
        return HAL_ERROR;
    return HAL_FDCAN_ActivateNotification(hfdcan,
                                          FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                                          0);
}

static void CANServiceInit(void)
{
    if (CANStartBus(&hfdcan1) != HAL_OK ||
        CANStartBus(&hfdcan2) != HAL_OK ||
        CANStartBus(&hfdcan3) != HAL_OK)
    {
        LOGERROR("[bsp_can] failed to start one or more FDCAN buses");
    }
    else
    {
        LOGINFO("[bsp_can] FDCAN1/2/3 service initialized");
    }
}

static void CANRestart(FDCAN_HandleTypeDef *hfdcan)
{
    (void)HAL_FDCAN_Stop(hfdcan);
    if (HAL_FDCAN_Init(hfdcan) != HAL_OK || CANStartBus(hfdcan) != HAL_OK)
        LOGERROR("[bsp_can] FDCAN restart failed");
}

CANInstance *CANRegister(CAN_Init_Config_s *config)
{
    if (config == NULL || config->can_handle == NULL || config->rx_id > 0x7FFU || config->tx_id > 0x7FFU)
    {
        LOGERROR("[bsp_can] invalid registration config");
        return NULL;
    }

    if (instance_count == 0U)
        CANServiceInit();

    if (instance_count >= CAN_MX_REGISTER_CNT)
    {
        LOGERROR("[bsp_can] CAN instance count exceeded %u", CAN_MX_REGISTER_CNT);
        return NULL;
    }

    for (uint8_t i = 0; i < instance_count; i++)
    {
        if (can_instance[i]->rx_id == config->rx_id &&
            can_instance[i]->can_handle == config->can_handle)
        {
            LOGERROR("[bsp_can] duplicate RX ID 0x%03lx", (unsigned long)config->rx_id);
            return NULL;
        }
    }

    CANInstance *instance = malloc(sizeof(*instance));
    if (instance == NULL)
    {
        LOGERROR("[bsp_can] allocation failed");
        return NULL;
    }
    memset(instance, 0, sizeof(*instance));

    instance->txconf.Identifier = config->tx_id;
    instance->txconf.IdType = FDCAN_STANDARD_ID;
    instance->txconf.TxFrameType = FDCAN_DATA_FRAME;
    instance->txconf.DataLength = FDCAN_DLC_BYTES_8;
    instance->txconf.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    instance->txconf.BitRateSwitch = FDCAN_BRS_OFF;
    instance->txconf.FDFormat = FDCAN_CLASSIC_CAN;
    instance->txconf.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    instance->txconf.MessageMarker = 0;
    instance->can_handle = config->can_handle;
    instance->tx_id = config->tx_id;
    instance->rx_id = config->rx_id;
    instance->can_module_callback = config->can_module_callback;
    instance->id = config->id;

    can_instance[instance_count++] = instance;
    return instance;
}

uint8_t CANTransmit(CANInstance *instance, float timeout)
{
    static uint32_t busy_count;
    float start;

    if (instance == NULL || instance->can_handle == NULL)
        return 0;

    start = DWT_GetTimeline_ms();
    while (HAL_FDCAN_GetTxFifoFreeLevel(instance->can_handle) == 0U)
    {
        if ((DWT_GetTimeline_ms() - start) > timeout)
        {
            LOGWARNING("[bsp_can] FDCAN TX FIFO full, count=%lu", (unsigned long)busy_count++);
            return 0;
        }
    }

    if (HAL_FDCAN_AddMessageToTxFifoQ(instance->can_handle,
                                      &instance->txconf,
                                      instance->tx_buff) != HAL_OK)
    {
        LOGWARNING("[bsp_can] failed to enqueue FDCAN frame, count=%lu", (unsigned long)busy_count++);
        return 0;
    }
    return 1;
}

void CANGetDebugInfo(FDCAN_HandleTypeDef *hfdcan, CAN_Debug_Info_s *out)
{
    if (hfdcan == NULL || out == NULL)
        return;

    out->tx_free_level = (uint8_t)HAL_FDCAN_GetTxFifoFreeLevel(hfdcan);
    out->hal_error = HAL_FDCAN_GetError(hfdcan);
    out->state = (uint32_t)HAL_FDCAN_GetState(hfdcan);
    out->esr = hfdcan->Instance->ECR;
    out->tsr = hfdcan->Instance->TXFQS;
    out->msr = hfdcan->Instance->PSR;
}

void CANSetAutoRetransmission(FDCAN_HandleTypeDef *hfdcan, uint8_t enable)
{
    if (hfdcan == NULL)
        return;
    hfdcan->Init.AutoRetransmission = enable ? ENABLE : DISABLE;
    CANRestart(hfdcan);
}

void CANSetAutoBusOff(FDCAN_HandleTypeDef *hfdcan, uint8_t enable)
{
    (void)hfdcan;
    (void)enable;
    LOGWARNING("[bsp_can] FDCAN has no bxCAN AutoBusOff setting; request preserved without reconfiguration");
}

void CANSetMode(FDCAN_HandleTypeDef *hfdcan, uint32_t mode)
{
    if (hfdcan == NULL)
        return;
    hfdcan->Init.Mode = mode;
    CANRestart(hfdcan);
}

void CANSetDLC(CANInstance *instance, uint8_t length)
{
    if (instance == NULL || length == 0U || length > 8U)
    {
        LOGERROR("[bsp_can] invalid classic CAN payload length");
        return;
    }
    instance->txconf.DataLength = CANLengthToDlc(length);
}

static void CANFIFO0Callback(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) != 0U)
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
            return;

        uint8_t rx_length = CANDlcToLength(rx_header.DataLength);
        if (rx_length > sizeof(rx_data))
            rx_length = sizeof(rx_data);

        for (uint8_t i = 0; i < instance_count; i++)
        {
            CANInstance *instance = can_instance[i];
            if (instance->can_handle == hfdcan && instance->rx_id == rx_header.Identifier)
            {
                instance->rx_len = rx_length;
                memcpy(instance->rx_buff, rx_data, rx_length);
                if (instance->can_module_callback != NULL)
                    instance->can_module_callback(instance);
            }
        }
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t rx_fifo0_its)
{
    if ((rx_fifo0_its & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0U)
        CANFIFO0Callback(hfdcan);
}
