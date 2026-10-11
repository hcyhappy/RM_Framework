#include "bsp_can.hpp"
#include "bsp_critical.hpp"
#include <cstring>

namespace {
constexpr uint8_t capacity = 16;
struct Bus {
    bool initialized;
    CAN_RxFrame frames[capacity];
    uint8_t head, tail, count;
    CAN_Stats stats;
};
Bus buses[2]{};
Bus *bus_for(CAN_HandleTypeDef *hcan) {
    if (hcan == &hcan1 && hcan->Instance == CAN1) return &buses[0];
    if (hcan == &hcan2 && hcan->Instance == CAN2) return &buses[1];
    return nullptr;
}
bool clock_enabled(CAN_HandleTypeDef *hcan) {
    return __HAL_RCC_CAN1_IS_CLK_ENABLED() &&
           (hcan == &hcan1 || __HAL_RCC_CAN2_IS_CLK_ENABLED());
}
bool ready(CAN_HandleTypeDef *hcan, Bus *bus) {
    return bus && bus->initialized && clock_enabled(hcan) &&
           HAL_CAN_GetState(hcan) == HAL_CAN_STATE_LISTENING;
}
HAL_StatusTypeDef initialize(CAN_HandleTypeDef *hcan, uint32_t bank) {
    Bus *bus = bus_for(hcan);
    if (!bus || !clock_enabled(hcan) ||
        !NVIC_GetEnableIRQ(hcan == &hcan1 ? CAN1_RX0_IRQn : CAN2_RX0_IRQn)) return HAL_ERROR;
    if (ready(hcan, bus)) return HAL_OK;
    if (HAL_CAN_GetState(hcan) != HAL_CAN_STATE_READY) return HAL_ERROR;
    CAN_FilterTypeDef filter{};
    filter.FilterActivation = ENABLE;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterBank = bank;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.SlaveStartFilterBank = 14; // Before configuring either controller.
    // Accept all in hardware; callback rejects extended and remote frames.
    HAL_StatusTypeDef status = HAL_CAN_ConfigFilter(hcan, &filter);
    if (status != HAL_OK) return status;
    status = HAL_CAN_Start(hcan); // HAL timeout needs IRQs enabled.
    if (status != HAL_OK) return status;
    {
        BspCritical lock;
        bus->initialized = true;
        status = HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
        if (status != HAL_OK) bus->initialized = false;
    }
    if (status != HAL_OK) (void)HAL_CAN_Stop(hcan);
    return status;
}
}
HAL_StatusTypeDef CAN_Init(void) {
    if (__get_IPSR() || __get_PRIMASK()) return HAL_ERROR;
    HAL_StatusTypeDef status = initialize(&hcan1, 0);
    return status == HAL_OK ? initialize(&hcan2, 14) : status;
}
HAL_StatusTypeDef CAN_Transmit(CAN_HandleTypeDef *hcan, uint32_t id,
                             const uint8_t *data, uint16_t length) {
    if (id > 0x7ffU || length > 8U || (!data && length)) return HAL_ERROR;
    CAN_TxHeaderTypeDef header{};
    header.StdId = id;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = length;
    uint8_t payload[8]{}; // HAL reads eight bytes regardless of DLC.
    if (length) std::memcpy(payload, data, length);
    uint32_t mailbox;
    BspCritical lock; // Serialize mailbox selection and register writes.
    if (!ready(hcan, bus_for(hcan))) return HAL_ERROR;
    if (hcan->Instance->ESR & CAN_ESR_BOFF) return HAL_ERROR;
    if (!HAL_CAN_GetTxMailboxesFreeLevel(hcan)) return HAL_BUSY;
    return HAL_CAN_AddTxMessage(hcan, &header, payload, &mailbox);
}
HAL_StatusTypeDef CAN_Read(CAN_HandleTypeDef *hcan, CAN_RxFrame *frame) {
    if (!frame) return HAL_ERROR;
    BspCritical lock;
    Bus *bus = bus_for(hcan);
    if (!ready(hcan, bus)) return HAL_ERROR;
    if (!bus->count) return HAL_BUSY;
    *frame = bus->frames[bus->tail];
    bus->tail = (bus->tail + 1U) % capacity;
    --bus->count;
    return HAL_OK;
}
HAL_StatusTypeDef CAN_GetStats(CAN_HandleTypeDef *hcan, CAN_Stats *stats) {
    if (!stats) return HAL_ERROR;
    BspCritical lock;
    Bus *bus = bus_for(hcan);
    if (!bus || !bus->initialized) return HAL_ERROR;
    *stats = bus->stats;
    return HAL_OK;
}
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    BspCritical lock;
    Bus *bus = bus_for(hcan);
    if (!ready(hcan, bus)) return;
    // Hardware FIFO depth is 3; bound work even under continuous traffic.
    for (unsigned i = 0; i < 3 && HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0); ++i) {
        CAN_RxHeaderTypeDef header{};
        CAN_RxFrame frame{};
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, frame.data) != HAL_OK) {
            ++bus->stats.errors;
            break;
        }
        if (header.IDE != CAN_ID_STD || header.RTR != CAN_RTR_DATA ||
            header.StdId > 0x7ffU || header.DLC > 8U || bus->count == capacity) {
            ++bus->stats.dropped;
            continue;
        }
        frame.id = header.StdId;
        frame.length = static_cast<uint8_t>(header.DLC);
        bus->frames[bus->head] = frame;
        bus->head = (bus->head + 1U) % capacity;
        ++bus->count;
        ++bus->stats.received;
    }
}
