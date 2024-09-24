#include "datadesc.h"

uint8_t rs485Header::get_actual_packet_length() const noexcept {
  uint8_t v_result = 0;
  switch ( msgCode ) {
    case CMD_GET_INFO:
    case CMD_GET_EVENT:
    case CMD_GET_STATES:
      v_result = (uint8_t)sizeof(cmdSimple);
      break;
      
    case CMD_SET_OUTPUT_STATE:
      v_result = (uint8_t)sizeof(cmdSetOutputState);
      break;
      
    case CMD_GET_INPUT_VALUE:
      v_result = (uint8_t)sizeof(cmdGetInputState);
      break;

    case CMD_SET_DATETIME:
      v_result = (uint8_t)sizeof(cmdSetDateTime);
      break;

    case ANSWER_ACK:
    case ANSWER_NACK:
      v_result = (uint8_t)sizeof(aswModuleAck);
      break;
      
    case ANSWER_MODULE_INFO:
      v_result = (uint8_t)sizeof(aswModuleInfo);
      break;
      
    case ANSWER_MODULE_EVENT:
      v_result = (uint8_t)sizeof(aswModuleEvent);
      break;
      
    case ANSWER_MODULE_STATES:
      v_result = (uint8_t)sizeof(aswModuleState);
      break;
      
    default:
      break;
  }
  
  return v_result;
}
