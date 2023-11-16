/*
 * CAN.c
 *
 *  Created on: Oct 14, 2023
 *      Author: Tarun Singh

 *
 */

#include"CAN.h"
#include "main.h"
#include"string.h"
#include "stdio.h"
#include "math.h"

uint8_t Txdata1[8];
uint8_t Rxdata11[8];
char message[30];
uint32_t Txmailbox;

extern CAN_HandleTypeDef hcan;
extern UART_HandleTypeDef huart2;


void CAN_TXU(unsigned int CAN_ID, uint8_t a)
{     // unsigned int CAN_ID = 0x1A;
	  Txheader.DLC =4;//DATA LENGTH
	  Txheader.IDE= CAN_ID_STD;
	  Txheader.RTR=CAN_RTR_DATA;
	  Txheader.StdId=CAN_ID; //id
      Txdata1[0] = a;



	  HAL_CAN_AddTxMessage(&hcan, &Txheader, Txdata1, &Txmailbox);
	  HAL_Delay(10);
}

void uint8_float(uint8_t Rxdata111[8])
{
		 uint32_t intVal;
	     float receivedFloat;

	     // Convert the received bytes back to a 32-bit integer
	     intVal = 0;
	     for (int i = 0; i < 8; i++) {
	         intVal |= (Rxdata111[i] << (i * 8));
	     }

	     // Convert the integer back to a float
	     receivedFloat = *((float*)&intVal);
	     printf("%f",receivedFloat);
}
float convertBytesToFloat(uint8_t Rxdata111[4]) {
    uint32_t intVal = 0;

    // Convert the received bytes back to a 32-bit integer
    for (int i = 0; i < 4; i++) {
        intVal |= (Rxdata111[i] << (i * 8));
    }

    // Convert the integer back to a float
    float receivedFloat = *((float*)&intVal);

    return receivedFloat;
}
void CAN_RX(void){

	  	if( HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO1, &Rxheader, Rxdata11) == HAL_OK)
	  	{
		if(Rxheader.StdId==0x100)
		{
			Uart(Rxheader.StdId,Rxdata11[0]);

		}
		else if (Rxheader.StdId == 0x102) {

			Uart(Rxheader.StdId,Rxdata11[0]);
		}
		else if (Rxheader.StdId == 0x104) {

					Uart(Rxheader.StdId,Rxdata11[0]);
				}
		else if (Rxheader.StdId == 0x106) {

							Uart(Rxheader.StdId,Rxdata11[0]);
						}
		else if (Rxheader.StdId == 0x108) {

							Uart(Rxheader.StdId,Rxdata11[0]);
						}
		else if (Rxheader.StdId == 0x110) {

							Uart(Rxheader.StdId,Rxdata11[0]);
						}
		else if (Rxheader.StdId == 0x112) {

						Uart(Rxheader.StdId,Rxdata11[0]);
						}
		else if (Rxheader.StdId == 0x116) {

						Uart(Rxheader.StdId,Rxdata11[0]);
						Uart(Rxheader.StdId,Rxdata11[1]);
						Uart(Rxheader.StdId,Rxdata11[2]);

						}

	  	};



};
void Uart( unsigned int a,float b){


	sprintf(message," Sensor Id %x  , %0.4f ",a,b);
	strcat(message, "\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)message, sizeof(message), 100);
}






