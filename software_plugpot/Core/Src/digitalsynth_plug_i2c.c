//desciption
/*
//include
#include "main.h"
#include "digitalsynth_plug_i2c.h"

//define
//I2C defines
#define TXSIZE 12
#define RXSIZE  3

//variables
//I2C global variables
extern uint8_t RxData[RXSIZE];
extern uint8_t rxcount;
extern int countAddr;
//int is_first_recvd = 0;
//int countrxcplt = 0;
extern uint8_t TxData[TXSIZE];
extern uint8_t txcount;
extern int counterror;

//func..

*/
//*******
//I2C functions
//*******
/*
void process_data(){
  //TODO
  //memcpy(mainbuf, RXDATA+1, rxcount-1);
};

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c){
  HAL_I2C_EnableListen_IT(hi2c);
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode){
  if(TransferDirection == I2C_DIRECTION_TRANSMIT){
    rxcount = 0;
    countAddr++;
    HAL_I2C_Slave_Sequential_Receive_IT(hi2c, RxData+rxcount, 1, I2C_FIRST_FRAME);
  }
  else{
    txcount = 0;
    HAL_I2C_Slave_Seq_Transmit_IT(hi2c, TxData+txcount, 1, I2C_FIRST_FRAME);
  }
*/
/*  
  if(TransferDirection == I2C_DIRECTION_TRANSMIT){
    if(is_first_recvd == 0){
      rxcount = 0;
      countAddr++;
      HAL_I2C_Slave_Sequential_Receive_IT(hi2c, RxData+rxcount, 1, I2C_FIRST_FRAME);
    }
  }
  else{
    txcount = 0;
    HAL_I2C_Slave_Seq_Transmit_IT(hi2c, TxData+txcount, 1, I2C_FIRST_FRAME);
  }
*/
/*
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c){
  txcount++;
  HAL_I2C_Slave_Seq_Transmit_IT(hi2c, TxData+txcount, 1, I2C_NEXT_FRAME);
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c){
  rxcount++;
  if(rxcount < RXSIZE){
    if(rxcount == RXSIZE - 1){
      HAL_I2C_Slave_Sequential_Receive_IT(hi2c, RxData+rxcount, 1, I2C_LAST_FRAME);
    }
    else{
      HAL_I2C_Slave_Sequential_Receive_IT(hi2c, RxData+rxcount, 1, I2C_NEXT_FRAME);
    }
  }
  
  if(rxcount == RXSIZE){
    process_data();
  }
  //countrxcplt++;
*/
  /*
  if(is_first_recvd == 0){
    rxcount++;
    is_first_recvd = 1;
    HAL_I2C_Slave_Seq_Receive_IT(hi2c, RxData+rxcount, RxData[0], I2C_LAST_FRAME);
  }
  else{
    rxcount = rxcount+RxData[0];
    is_first_recvd=0;
    process_data();
  }
*/
/*
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c){
  counterror++;
  uint32_t errorcode = HAL_I2C_GetError(hi2c);
  if(errorcode == 4){
    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_AF);    //clear AF Flag
    if(txcount == 0){
      process_data();
    }
    else{       //error while slave is transmitting
      txcount = txcount - 1;
    }
    process_data();
  }
  HAL_I2C_EnableListen_IT(hi2c);
  
  //testing with LED
  //Set_LED(TESTING_LED_NUM, 1, 0, 0);
  //WS2812_Send();
}
*/

