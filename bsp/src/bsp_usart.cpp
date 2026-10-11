#include "bsp_usart.hpp"
#include "bsp_critical.hpp"
#include <cstring>
#include <cstdint>

namespace {
constexpr uint16_t transfer_capacity = 256, ring_capacity = 512;
struct Port {
    bool initialized, receiving, tx_busy, tx_async, tx_needs_abort;
    uint8_t rx[transfer_capacity], tx[transfer_capacity], ring[ring_capacity];
    uint8_t *rx_buffer;
    uint16_t rx_capacity, head, tail;
    USART_Stats stats;
};
Port ports[2]{};
Port *port_for(UART_HandleTypeDef *huart) {
    if (huart == &huart1 && huart->Instance == USART1) return &ports[0];
    if (huart == &huart6 && huart->Instance == USART6) return &ports[1];
    return nullptr;
}
IRQn_Type uart_irq(UART_HandleTypeDef *huart) {
    return huart == &huart1 ? USART1_IRQn : USART6_IRQn;
}
bool hardware_ready(UART_HandleTypeDef *huart) {
    return port_for(huart) &&
        (huart == &huart1 ? __HAL_RCC_USART1_IS_CLK_ENABLED() : __HAL_RCC_USART6_IS_CLK_ENABLED()) &&
        huart->gState != HAL_UART_STATE_RESET && huart->gState != HAL_UART_STATE_ERROR &&
        huart->RxState != HAL_UART_STATE_RESET &&
        huart->Init.WordLength == UART_WORDLENGTH_8B &&
        huart->Init.Parity == UART_PARITY_NONE && huart->Init.Mode == UART_MODE_TX_RX;
}
bool dma_ready(UART_HandleTypeDef *huart, bool receive) {
    DMA_HandleTypeDef *dma = receive ? huart->hdmarx : huart->hdmatx;
    DMA_Stream_TypeDef *stream;
    IRQn_Type irq;
    if (huart == &huart1) {
        stream = receive ? DMA2_Stream2 : DMA2_Stream7;
        irq = receive ? DMA2_Stream2_IRQn : DMA2_Stream7_IRQn;
    } else {
        stream = receive ? DMA2_Stream1 : DMA2_Stream6;
        irq = receive ? DMA2_Stream1_IRQn : DMA2_Stream6_IRQn;
    }
    return __HAL_RCC_DMA2_IS_CLK_ENABLED() && dma && dma->Instance == stream &&
        dma->Parent == huart && dma->State != HAL_DMA_STATE_RESET &&
        dma->Init.Channel == (huart == &huart1 ? DMA_CHANNEL_4 : DMA_CHANNEL_5) &&
        dma->Init.Direction == (receive ? DMA_PERIPH_TO_MEMORY : DMA_MEMORY_TO_PERIPH) &&
        dma->Init.Mode == DMA_NORMAL && dma->Init.PeriphInc == DMA_PINC_DISABLE &&
        dma->Init.MemInc == DMA_MINC_ENABLE &&
        dma->Init.PeriphDataAlignment == DMA_PDATAALIGN_BYTE &&
        dma->Init.MemDataAlignment == DMA_MDATAALIGN_BYTE && NVIC_GetEnableIRQ(irq) &&
        NVIC_GetEnableIRQ(uart_irq(huart));
}
bool dma_memory(const uint8_t *data, uint16_t size) {
    const uintptr_t address = reinterpret_cast<uintptr_t>(data);
    return size && address >= 0x20000000U && address < 0x20020000U &&
           size <= 0x20020000U - address; // CCM/FLASH cannot be DMA RX storage.
}
HAL_StatusTypeDef arm_receive(UART_HandleTypeDef *huart, Port *port) {
    if (!hardware_ready(huart) || !dma_ready(huart, true) ||
        !dma_memory(port->rx_buffer, port->rx_capacity)) return HAL_ERROR;
    HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(huart, port->rx_buffer,
                                                          port->rx_capacity);
    if (status == HAL_OK) __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    return status;
}
HAL_StatusTypeDef initialize(UART_HandleTypeDef *huart) {
    BspCritical lock;
    Port *port = port_for(huart);
    if (!hardware_ready(huart) || !dma_ready(huart, true) || !dma_ready(huart, false))
        return HAL_ERROR;
    if (port->initialized) return HAL_OK;
    port->rx_buffer = port->rx;
    port->rx_capacity = transfer_capacity;
    port->receiving = true;
    port->stats.receive_status = arm_receive(huart, port);
    port->initialized = port->stats.receive_status == HAL_OK;
    if (!port->initialized) port->receiving = false;
    return port->stats.receive_status;
}
}
HAL_StatusTypeDef USART1_Init(void) { return initialize(&huart1); }
HAL_StatusTypeDef USART6_Init(void) { return initialize(&huart6); }
HAL_StatusTypeDef USART_Init(void) {
    HAL_StatusTypeDef status = USART1_Init();
    return status == HAL_OK ? USART6_Init() : status;
}
HAL_StatusTypeDef USART_Transmit(UART_HandleTypeDef *huart, const uint8_t *data,
                               uint16_t size, USART_Mode mode, uint32_t timeout_ms) {
    if (!data || !size || (mode != USART_MODE_BLOCK && mode != USART_MODE_DMA &&
        mode != USART_MODE_IT)) return HAL_ERROR;
    if (mode == USART_MODE_BLOCK &&
        (__get_IPSR() || __get_PRIMASK() || timeout_ms == HAL_MAX_DELAY)) return HAL_ERROR;
    Port *port = port_for(huart);
    {
        BspCritical lock;
        if (!port || !port->initialized || !hardware_ready(huart)) return HAL_ERROR;
        if (port->tx_busy || huart->gState != HAL_UART_STATE_READY) return HAL_BUSY;
        if (mode != USART_MODE_BLOCK &&
            (size > transfer_capacity || !NVIC_GetEnableIRQ(uart_irq(huart)))) return HAL_ERROR;
        if (mode == USART_MODE_DMA &&
            (!dma_ready(huart, false) || !dma_memory(port->tx, size))) return HAL_ERROR;
        port->tx_busy = true;
        port->tx_async = mode != USART_MODE_BLOCK;
        if (mode != USART_MODE_BLOCK) {
            std::memcpy(port->tx, data, size);
            HAL_StatusTypeDef status = mode == USART_MODE_DMA
                ? HAL_UART_Transmit_DMA(huart, port->tx, size)
                : HAL_UART_Transmit_IT(huart, port->tx, size);
            if (status != HAL_OK) port->tx_busy = port->tx_async = false;
            return status;
        }
    }
    // No IRQ masking while waiting: HAL timeout needs TIM6.
    HAL_StatusTypeDef status = HAL_UART_Transmit(huart, data, size, timeout_ms);
    {
        BspCritical lock;
        port->tx_busy = false;
    }
    return status;
}
HAL_StatusTypeDef USART_Receive(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size) {
    if (!dma_memory(data, size) || size > transfer_capacity) return HAL_ERROR;
    BspCritical lock;
    Port *port = port_for(huart);
    if (!port || !port->initialized || !hardware_ready(huart)) return HAL_ERROR;
    if (port->receiving || huart->RxState != HAL_UART_STATE_READY) return HAL_BUSY;
    port->rx_buffer = data;
    port->rx_capacity = size;
    port->receiving = true;
    port->stats.receive_status = arm_receive(huart, port);
    if (port->stats.receive_status != HAL_OK) port->receiving = false;
    return port->stats.receive_status;
}
HAL_StatusTypeDef USART_StopReceive(UART_HandleTypeDef *huart) {
    // Abort may wait for DMA EN to clear. Never do this in an ISR/critical section.
    if (__get_IPSR() || __get_PRIMASK()) return HAL_ERROR;
    Port *port = port_for(huart);
    {
        BspCritical lock;
        if (!port || !port->initialized || !hardware_ready(huart)) return HAL_ERROR;
        port->receiving = false;
    }
    HAL_StatusTypeDef status = HAL_UART_AbortReceive(huart);
    {
        BspCritical lock;
        port->stats.receive_status = status;
    }
    return status;
}
HAL_StatusTypeDef USART_Service(UART_HandleTypeDef *huart) {
    if (__get_IPSR() || __get_PRIMASK()) return HAL_ERROR;
    Port *port = port_for(huart);
    bool abort_tx;
    {
        BspCritical lock;
        if (!port || !port->initialized || !hardware_ready(huart)) return HAL_ERROR;
        abort_tx = port->tx_needs_abort;
    }
    if (abort_tx) {
        // A DMA error can restore UART READY while its TX stream is still live.
        // Keep TX storage reserved until HAL has actually stopped that stream.
        HAL_StatusTypeDef status = HAL_UART_AbortTransmit(huart);
        BspCritical lock;
        if (status != HAL_OK) return status;
        port->tx_busy = port->tx_async = port->tx_needs_abort = false;
    }
    BspCritical lock;
    if (!port || !port->initialized || !hardware_ready(huart)) return HAL_ERROR;
    if (!port->receiving) return HAL_OK;
    if (huart->RxState == HAL_UART_STATE_BUSY_RX) return HAL_OK;
    if (huart->hdmarx && huart->hdmarx->State == HAL_DMA_STATE_ERROR) return HAL_ERROR;
    port->stats.receive_status = arm_receive(huart, port);
    return port->stats.receive_status;
}
uint16_t USART_Read(UART_HandleTypeDef *huart, uint8_t *data, uint16_t capacity) {
    if (!data || !capacity) return 0;
    BspCritical lock;
    Port *port = port_for(huart);
    if (!port || !port->initialized) return 0;
    uint16_t read = 0;
    while (read < capacity && read < transfer_capacity && port->tail != port->head) {
        data[read++] = port->ring[port->tail];
        port->tail = (port->tail + 1U) % ring_capacity;
    }
    return read;
}
HAL_StatusTypeDef USART_GetStats(UART_HandleTypeDef *huart, USART_Stats *stats) {
    if (!stats) return HAL_ERROR;
    BspCritical lock;
    Port *port = port_for(huart);
    if (!port || !port->initialized) return HAL_ERROR;
    *stats = port->stats;
    return HAL_OK;
}
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size) {
    BspCritical lock;
    Port *port = port_for(huart);
    if (!port || !port->initialized || !port->receiving) return;
    if (HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_HT) return;
    if (size > port->rx_capacity) ++port->stats.errors;
    else {
        for (uint16_t i = 0; i < size; ++i) {
            const uint16_t next = (port->head + 1U) % ring_capacity;
            if (next == port->tail) ++port->stats.dropped_bytes;
            else {
                port->ring[port->head] = port->rx_buffer[i];
                port->head = next;
                ++port->stats.received_bytes;
            }
        }
    }
    // Normal DMA stops on IDLE/TC: copy first, then reuse its buffer.
    port->stats.receive_status = arm_receive(huart, port);
    if (port->stats.receive_status != HAL_OK) ++port->stats.errors;
}
extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    BspCritical lock;
    Port *port = port_for(huart);
    if (port && port->tx_async && !port->tx_needs_abort)
        port->tx_busy = port->tx_async = false;
}
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    BspCritical lock;
    Port *port = port_for(huart);
    if (!port || !port->initialized) return;
    ++port->stats.errors;
    port->stats.last_error = huart->ErrorCode;
    port->stats.receive_status = HAL_ERROR;
    if (port->tx_async && huart->gState == HAL_UART_STATE_READY)
        port->tx_needs_abort = true;
    // Retry from USART_Service, after HAL abort/error handling has completed.
}
