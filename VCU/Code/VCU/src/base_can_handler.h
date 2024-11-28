// base_can_handler.h
#pragma once
#include "config.h"

class BaseCANHandler {
protected:
    mcp2515_can& canBus;
    byte readBuffer[MAX_DATA_SIZE] = {0};

    struct CANMessage {
        uint32_t id;
        uint8_t len;
        byte* data;
    };

    BaseCANHandler(mcp2515_can& can) : canBus(can) {}
    
    bool receiveMessage(CANMessage& msg) {
        if (canBus.checkReceive() != CAN_MSGAVAIL) return false;
        canBus.readMsgBuf(&msg.len, readBuffer);
        msg.id = canBus.getCanId();
        msg.data = readBuffer;
        return true;
    }

    void sendMessage(uint32_t id, uint8_t len, const byte* data) {
        canBus.sendMsgBuf(id, 0, len, data);
    }
};