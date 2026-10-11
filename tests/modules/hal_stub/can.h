#pragma once
#include "main.h"
struct CAN_HandleTypeDef {
    int bus;
};
extern CAN_HandleTypeDef hcan1, hcan2;
