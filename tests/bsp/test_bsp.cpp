#include "bsp.hpp"
#include "bsp_can.hpp"
#include "bsp_usart.hpp"
#include "bsp_pwm.hpp"
#include <cassert>
#include <cstring>
#include <cstdio>
#include <limits>
#include <initializer_list>
CAN_TypeDef can_regs[2]{};
USART_TypeDef uart_regs[2]{};
DMA_Stream_TypeDef dma_regs[4]{};
TIM_TypeDef tim_regs[2]{};
CAN_HandleTypeDef hcan1{}, hcan2{};
UART_HandleTypeDef huart1{}, huart6{};
TIM_HandleTypeDef htim1{}, htim8{};
DMA_HandleTypeDef mock_dma[4]{};
bool clocks[7]{}, enabled[8]{};
uint32_t primask=0, ipsr=0;
CAN_FilterTypeDef filters[2]{};
CAN_TxHeaderTypeDef sent_header{};
uint8_t sent_payload[8]{};
uint32_t mailbox_free=3, rx_pending=0, config_calls=0;
CAN_RxHeaderTypeDef incoming{};
HAL_StatusTypeDef rx_result=HAL_OK, tx_result=HAL_OK, filter_result=HAL_OK;
unsigned rx_calls=0, tx_calls=0;
int HAL_CAN_GetState(CAN_HandleTypeDef *h) { return h->State; }
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *h, const CAN_FilterTypeDef *f) {
    ++config_calls; filters[h==&hcan1?0:1]=*f; return filter_result;
}
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *h) {
    assert(!primask); h->State=HAL_CAN_STATE_LISTENING; return HAL_OK;
}
HAL_StatusTypeDef HAL_CAN_Stop(CAN_HandleTypeDef *h) { h->State=HAL_CAN_STATE_READY; return HAL_OK; }
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *, uint32_t) { return HAL_OK; }
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *) { return mailbox_free; }
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *, const CAN_TxHeaderTypeDef *h,
                                     const uint8_t *d, uint32_t *m) {
    ++tx_calls; sent_header=*h; std::memcpy(sent_payload,d,8); *m=1; return HAL_OK;
}
uint32_t HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef *, uint32_t) { return rx_pending; }
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *, uint32_t, CAN_RxHeaderTypeDef *h, uint8_t *d) {
    --rx_pending; *h=incoming; std::memset(d,0x42,8); return HAL_OK;
}
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *h, uint8_t *d, uint16_t n) {
    ++rx_calls;
    if (h->RxState!=HAL_UART_STATE_READY) return HAL_BUSY;
    if (rx_result!=HAL_OK) return rx_result;
    h->rx=d; h->rx_size=n; h->RxState=HAL_UART_STATE_BUSY_RX;
    h->hdmarx->State=HAL_DMA_STATE_BUSY; return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *h, const uint8_t *d, uint16_t n) {
    if(tx_result!=HAL_OK) return tx_result;
    h->tx=d; h->tx_size=n; h->gState=HAL_UART_STATE_BUSY_TX; return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *h, const uint8_t *d, uint16_t n) {
    return HAL_UART_Transmit_DMA(h,d,n);
}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *, const uint8_t *, uint16_t, uint32_t) {
    assert(!primask); return tx_result;
}
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *h) {
    assert(!primask); h->RxState=HAL_UART_STATE_READY; h->hdmarx->State=HAL_DMA_STATE_READY; return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *h) {
    assert(!primask); h->gState=HAL_UART_STATE_READY; h->hdmatx->State=HAL_DMA_STATE_READY; return HAL_OK;
}
uint32_t HAL_UARTEx_GetRxEventType(UART_HandleTypeDef *h) { return h->RxEventType; }
void HAL_RCC_GetClockConfig(RCC_ClkInitTypeDef *c, uint32_t *l) { c->APB2CLKDivider=2; *l=5; }
uint32_t HAL_RCC_GetPCLK2Freq() { return 84000000; }
int HAL_TIM_GetChannelState(TIM_HandleTypeDef *h, uint32_t c) { return h->channels[c/4]; }
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *h, uint32_t c) {
    h->channels[c/4]=HAL_TIM_CHANNEL_STATE_BUSY; h->Instance->CR1|=TIM_CR1_CEN; return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *h, uint32_t c) {
    h->channels[c/4]=HAL_TIM_CHANNEL_STATE_READY;
    bool any=false; for(int s:h->channels) any|=s==HAL_TIM_CHANNEL_STATE_BUSY;
    if(!any) h->Instance->CR1&=~TIM_CR1_CEN;
    return HAL_OK;
}
void setup() {
    for(auto &c:clocks) c=true;
    for(auto &e:enabled) e=true;
    hcan1={CAN1,HAL_CAN_STATE_READY}; hcan2={CAN2,HAL_CAN_STATE_READY};
    huart1.Instance=USART1; huart6.Instance=USART6;
    UART_HandleTypeDef *ports[]={&huart1,&huart6};
    for(unsigned i=0;i<2;++i) {
        auto *h=ports[i]; h->gState=h->RxState=HAL_UART_STATE_READY;
        h->Init={UART_WORDLENGTH_8B,UART_PARITY_NONE,UART_MODE_TX_RX};
        for(unsigned j=0;j<2;++j) {
            auto &d=mock_dma[i*2+j]; d.Instance=&dma_regs[i*2+j]; d.Parent=h;
            d.State=HAL_DMA_STATE_READY;
            d.Init={i==0?DMA_CHANNEL_4:DMA_CHANNEL_5,j==0?DMA_PERIPH_TO_MEMORY:DMA_MEMORY_TO_PERIPH,
                    DMA_NORMAL,DMA_PINC_DISABLE,DMA_MINC_ENABLE,DMA_PDATAALIGN_BYTE,DMA_MDATAALIGN_BYTE};
        }
        h->hdmarx=&mock_dma[i*2]; h->hdmatx=&mock_dma[i*2+1];
    }
    htim1.Instance=TIM1; htim8.Instance=TIM8;
    for(auto *h:{&htim1,&htim8}) {
        h->State=HAL_TIM_STATE_READY; h->Init={TIM_COUNTERMODE_UP,19999};
        h->Instance->PSC=167; h->Instance->ARR=19999;
        for(auto &c:h->channels) c=HAL_TIM_CHANNEL_STATE_READY;
    }
}
void inject_uart(UART_HandleTypeDef *h,uint16_t n,uint8_t value,uint32_t event=HAL_UART_RXEVENT_IDLE) {
    std::memset(h->rx,value,n); h->RxEventType=event;
    if(event!=HAL_UART_RXEVENT_HT) {
        h->RxState=HAL_UART_STATE_READY; h->hdmarx->State=HAL_DMA_STATE_READY;
    }
    HAL_UARTEx_RxEventCallback(h,n);
}
int main() {
    uint8_t data[300]={1,2,3}, read[512]{};
    CAN_RxFrame frame{}; USART_Stats uart_stats{}; CAN_Stats can_stats{};
    assert(CAN_Transmit(nullptr,1,data,1)==HAL_ERROR);
    assert(USART_Transmit(nullptr,data,1,USART_MODE_DMA)==HAL_ERROR);
    assert(PWM_Start(nullptr,TIM_CHANNEL_1)==HAL_ERROR);
    assert(bsp_Init()==HAL_ERROR);
    setup();
    assert(CAN_Transmit(&hcan1,1,data,1)==HAL_ERROR);
    assert(USART_Transmit(&huart1,data,1,USART_MODE_DMA)==HAL_ERROR);
    assert(PWM_Start(&htim1,TIM_CHANNEL_1)==HAL_ERROR);
    enabled[DMA2_Stream2_IRQn]=false;
    assert(USART_Init()==HAL_ERROR); enabled[DMA2_Stream2_IRQn]=true;
    mock_dma[0].Init.Channel=5;
    assert(USART_Init()==HAL_ERROR); mock_dma[0].Init.Channel=DMA_CHANNEL_4;
    filter_result=HAL_ERROR;
    assert(CAN_Init()==HAL_ERROR); filter_result=HAL_OK;
    assert(bsp_Init()==HAL_OK);
    assert(filters[0].SlaveStartFilterBank==14 && filters[1].SlaveStartFilterBank==14);
    assert(filters[0].FilterBank==0 && filters[1].FilterBank==14);
    auto previous=config_calls; assert(CAN_Init()==HAL_OK && config_calls==previous);
    assert(CAN_Transmit(&hcan1,0x800,data,1)==HAL_ERROR);
    assert(CAN_Transmit(&hcan1,1,data,9)==HAL_ERROR);
    assert(CAN_Transmit(&hcan1,1,nullptr,1)==HAL_ERROR);
    assert(CAN_Transmit(&hcan1,0x200,data,3)==HAL_OK);
    assert(sent_header.DLC==3 && sent_header.StdId==0x200);
    assert(sent_payload[0]==1 && sent_payload[2]==3 && sent_payload[3]==0 && sent_payload[7]==0);
    assert(CAN_Transmit(&hcan2,1,nullptr,0)==HAL_OK);
    mailbox_free=0; assert(CAN_Transmit(&hcan1,1,data,1)==HAL_BUSY); mailbox_free=3;
    CAN1->ESR=CAN_ESR_BOFF; assert(CAN_Transmit(&hcan1,1,data,1)==HAL_ERROR); CAN1->ESR=0;
    incoming={0x201,CAN_ID_STD,CAN_RTR_DATA,8};
    rx_pending=6; HAL_CAN_RxFifo0MsgPendingCallback(&hcan1); assert(rx_pending==3);
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
    for(unsigned i=0;i<12;++i) { rx_pending=1; HAL_CAN_RxFifo0MsgPendingCallback(&hcan1); }
    assert(CAN_GetStats(&hcan1,&can_stats)==HAL_OK && can_stats.received==16 && can_stats.dropped==2);
    for(unsigned i=0;i<16;++i) assert(CAN_Read(&hcan1,&frame)==HAL_OK && frame.id==0x201 && frame.data[7]==0x42);
    assert(CAN_Read(&hcan1,&frame)==HAL_BUSY && CAN_Read(&hcan2,&frame)==HAL_BUSY);
    incoming.IDE=1; rx_pending=1; HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
    assert(CAN_Read(&hcan1,&frame)==HAL_BUSY);
    assert(USART_Transmit(&huart1,nullptr,1,USART_MODE_IT)==HAL_ERROR);
    assert(USART_Transmit(&huart1,data,0,USART_MODE_DMA)==HAL_ERROR);
    assert(USART_Transmit(&huart1,data,257,USART_MODE_DMA)==HAL_ERROR);
    assert(USART_Transmit(&huart1,data,1,static_cast<USART_Mode>(8))==HAL_ERROR);
    tx_result=HAL_ERROR; assert(USART_Transmit(&huart1,data,3,USART_MODE_DMA)==HAL_ERROR);
    tx_result=HAL_OK;
    assert(USART_Transmit(&huart1,data,3,USART_MODE_DMA)==HAL_OK);
    data[0]=9; assert(huart1.tx[0]==1 && huart1.tx!=data);
    assert(USART_Transmit(&huart1,data,3,USART_MODE_IT)==HAL_BUSY);
    huart1.gState=HAL_UART_STATE_READY; HAL_UART_TxCpltCallback(&huart1);
    assert(USART_Transmit(&huart1,data,3,USART_MODE_IT)==HAL_OK);
    huart1.gState=HAL_UART_STATE_READY; HAL_UART_TxCpltCallback(&huart1);
    ipsr=1; assert(USART_Transmit(&huart1,data,3,USART_MODE_BLOCK)==HAL_ERROR); ipsr=0;
    assert(USART_Transmit(&huart1,data,3,USART_MODE_BLOCK,HAL_MAX_DELAY)==HAL_ERROR);
    tx_result=HAL_TIMEOUT; assert(USART_Transmit(&huart1,data,3,USART_MODE_BLOCK)==HAL_TIMEOUT); tx_result=HAL_OK;
    auto calls=rx_calls;
    inject_uart(&huart1,128,0x12,HAL_UART_RXEVENT_HT);
    assert(rx_calls==calls && USART_Read(&huart1,read,512)==0);
    inject_uart(&huart1,256,0x34,HAL_UART_RXEVENT_TC);
    inject_uart(&huart1,256,0x56);
    assert(USART_GetStats(&huart1,&uart_stats)==HAL_OK && uart_stats.received_bytes==511 && uart_stats.dropped_bytes==1);
    assert(USART_Read(&huart1,read,512)==256 && read[0]==0x34 && read[255]==0x34);
    assert(USART_Read(&huart1,read,512)==255 && read[0]==0x56);
    assert(USART_Read(&huart6,read,512)==0);
    huart1.RxState=HAL_UART_STATE_READY; huart1.hdmarx->State=HAL_DMA_STATE_READY;
    huart1.ErrorCode=8; HAL_UART_ErrorCallback(&huart1);
    rx_result=HAL_ERROR; assert(USART_Service(&huart1)==HAL_ERROR);
    rx_result=HAL_OK; assert(USART_Service(&huart1)==HAL_OK && huart1.RxState==HAL_UART_STATE_BUSY_RX);
    assert(USART_Transmit(&huart1,data,3,USART_MODE_DMA)==HAL_OK);
    huart1.hdmatx->State=HAL_DMA_STATE_BUSY; huart1.gState=HAL_UART_STATE_READY;
    HAL_UART_ErrorCallback(&huart1);
    HAL_UART_TxCpltCallback(&huart1); // Late completion must not release errored storage.
    assert(USART_Transmit(&huart1,data,3,USART_MODE_DMA)==HAL_BUSY);
    assert(USART_Service(&huart1)==HAL_OK && huart1.hdmatx->State==HAL_DMA_STATE_READY);
    assert(USART_Transmit(&huart1,data,3,USART_MODE_DMA)==HAL_OK);
    huart1.gState=HAL_UART_STATE_READY; HAL_UART_TxCpltCallback(&huart1);
    assert(USART_Receive(&huart1,data,1)==HAL_ERROR); // Host stack isn't DMA SRAM.
    assert(USART_StopReceive(&huart1)==HAL_OK);
    auto *persistent=huart1.rx;
    assert(USART_Receive(&huart1,persistent,16)==HAL_OK);
    inject_uart(&huart1,3,0x78); assert(USART_Read(&huart1,read,10)==3 && read[2]==0x78);
    assert(PWM_SetDutyRatio(&htim1,-0.1f,TIM_CHANNEL_1)==HAL_ERROR);
    assert(PWM_SetDutyRatio(&htim1,std::numeric_limits<float>::quiet_NaN(),TIM_CHANNEL_1)==HAL_ERROR);
    assert(PWM_SetDutyRatio(&htim8,0.5f,TIM_CHANNEL_4)==HAL_ERROR);
    assert(PWM_SetDutyRatio(&htim1,1.0f,TIM_CHANNEL_1)==HAL_OK && TIM1->CCR[0]==20000);
    assert(PWM_SetPulseUs(&htim1,1500,TIM_CHANNEL_1)==HAL_OK && TIM1->CCR[0]==1500);
    assert(PWM_SetPulseUs(&htim1,21000,TIM_CHANNEL_1)==HAL_ERROR);
    assert(PWM_SetPeriod(&htim1,0.01f)==HAL_OK && TIM1->ARR==9999 && TIM1->CCR[0]==750);
    assert(PWM_SetPeriod(&htim1,0)==HAL_ERROR && PWM_SetPeriod(&htim1,1)==HAL_ERROR);
    assert(PWM_Start(&htim1,TIM_CHANNEL_1)==HAL_OK);
    assert(PWM_Start(&htim1,TIM_CHANNEL_1)==HAL_BUSY);
    assert(PWM_SetPeriod(&htim1,0.02f)==HAL_BUSY);
    assert(PWM_Start(&htim1,TIM_CHANNEL_2)==HAL_OK);
    assert(PWM_Stop(&htim1,TIM_CHANNEL_1)==HAL_OK && (TIM1->CR1&TIM_CR1_CEN));
    assert(PWM_Stop(&htim1,TIM_CHANNEL_2)==HAL_OK && !(TIM1->CR1&TIM_CR1_CEN));
    assert(PWM_SetPeriod(&htim1,0.02f)==HAL_OK && TIM1->ARR==19999);
    clocks[2]=false; assert(USART_Transmit(&huart1,data,3,USART_MODE_DMA)==HAL_ERROR);
    primask=1; assert(CAN_Read(&hcan1,&frame)==HAL_BUSY && primask==1); primask=0;
    std::puts("PASS: BSP guards, CAN queues, UART DMA lifecycle and PWM boundaries");
}
