#include "NRF24.h"
//------------------------------------------------
extern SPI_HandleTypeDef hspi1;

#define TX_ADR_WIDTH 3
#define TX_PLOAD_WIDTH 2

//------------------------------------------------
uint8_t TX_ADDRESS[TX_ADR_WIDTH] = {0xb3,0xb4,0x01};
uint8_t RX_BUF[TX_PLOAD_WIDTH] = {0};
//------------------------------------------------
 	
//------------------------------------------------
__STATIC_INLINE void DelayMicro(__IO uint32_t micros)
{
  micros *= (SystemCoreClock / 1000000) / 9;
  /* Wait till done */
  while (micros--) ;
}


//--------------------------------------------------
uint8_t NRF24_ReadReg(uint8_t addr)
{
  uint8_t dt=0, cmd;
  CS_ON;
  HAL_SPI_TransmitReceive(&hspi1,&addr,&dt,1,1000);
  if (addr!=STATUS)//IF NOT Status register than ask for data
  {
    cmd=0xFF;
    HAL_SPI_TransmitReceive(&hspi1,&cmd,&dt,1,1000);
  }
  CS_OFF;
  return dt;
}
//------------------------------------------------
void NRF24_WriteReg(uint8_t addr, uint8_t dt)
{
  addr |= W_REGISTER;//Address for writing
  CS_ON;
  HAL_SPI_Transmit(&hspi1,&addr,1,1000);//Transmit Address of Register
  HAL_SPI_Transmit(&hspi1,&dt,1,1000);//Transmit data on specific address
  CS_OFF;
}
//------------------------------------------------

//------------------------------------------------
void NRF24_ToggleFeatures(void)
{
  uint8_t dt[1] = {ACTIVATE};
  CS_ON;
  HAL_SPI_Transmit(&hspi1,dt,1,1000);
  DelayMicro(1);
  dt[0] = 0x73;
  HAL_SPI_Transmit(&hspi1,dt,1,1000);
  CS_OFF;
}
//------------------------------------------------

//------------------------------------------------
void NRF24_FlushRX(void)
{
  uint8_t dt[1] = {FLUSH_RX};
  CS_ON;
  HAL_SPI_Transmit(&hspi1,dt,1,1000);
  DelayMicro(1);
  CS_OFF;
}
//------------------------------------------------
void NRF24_FlushTX(void)
{
  uint8_t dt[1] = {FLUSH_TX};
  CS_ON;
  HAL_SPI_Transmit(&hspi1,dt,1,1000);
  DelayMicro(1);
  CS_OFF;
}
//------------------------------------------------


//------------------------------------------------
void NRF24L01_RX_Mode(void)
{
  uint8_t regval=0x00;
  regval = NRF24_ReadReg(CONFIG);
  
  regval |= (1<<PWR_UP)|(1<<PRIM_RX);
  NRF24_WriteReg(CONFIG,regval);
  CE_SET;
  DelayMicro(150); //???????? ??????? 130 ???
  // Flush buffers
  NRF24_FlushRX();
  NRF24_FlushTX();
}
//------------------------------------------------

//-----------------------------------------------
void NRF24_Read_Buf(uint8_t addr,uint8_t *pBuf,uint8_t bytes)
{
  CS_ON;
  HAL_SPI_Transmit(&hspi1,&addr,1,1000);//???????? ????? ? ????
  HAL_SPI_Receive(&hspi1,pBuf,bytes,1000);//???????? ?????? ? ?????
  CS_OFF;
}
//------------------------------------------------
void NRF24_Write_Buf(uint8_t addr,uint8_t *pBuf,uint8_t bytes)
{
  addr |= W_REGISTER;//??????? ??? ?????? ? ?????
  CS_ON;
  HAL_SPI_Transmit(&hspi1,&addr,1,1000);//???????? ????? ? ????
  DelayMicro(1);
  HAL_SPI_Transmit(&hspi1,pBuf,bytes,1000);//???????? ?????? ? ?????
  CS_OFF;
}
//------------------------------------------------


//--------------------------------------------------
void NRF24_ini(void)
{
  CE_SET;// Set PWR_UP bit, enable CRC(1 byte) &Prim_RX:0 (Transmitter)DelayMicro(5000);
	DelayMicro(5000);
	NRF24_WriteReg(CONFIG, 0x0a); 
	DelayMicro(5000);//
	NRF24_WriteReg(EN_AA, 0x02); // Enable Pipe1 Auto Ack
	DelayMicro(5000);//
	NRF24_WriteReg(EN_RXADDR, 0x02); // Enable Pipe1
	DelayMicro(5000);
	NRF24_WriteReg(SETUP_AW, 0x01); // Setup address width=3 bytes
	DelayMicro(5000);
	NRF24_WriteReg(SETUP_RETR, 0x5F); // // 1500us, 15 retrans
	DelayMicro(5000);
	NRF24_WriteReg(FEATURE, 0);
	DelayMicro(5000);
	NRF24_WriteReg(DYNPD, 0);
	DelayMicro(5000);
	NRF24_WriteReg(STATUS, 0x70); //Reset flags for IRQ
	DelayMicro(5000);
	NRF24_WriteReg(RF_CH, 76); // set channel to 76 so that it has frequency of 2476 MHz
	DelayMicro(5000);
	NRF24_WriteReg(RF_SETUP, 0x06); //TX_PWR:0dBm, Datarate:1Mbps
	DelayMicro(5000);
	NRF24_Write_Buf(TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);//Transmiter address
	DelayMicro(5000);
	NRF24_Write_Buf(RX_ADDR_P1, TX_ADDRESS, TX_ADR_WIDTH);//Receiver address
	DelayMicro(5000);
	NRF24_WriteReg(RX_PW_P1, TX_PLOAD_WIDTH); 
	
 

 

}

//--------------------------------------------------
