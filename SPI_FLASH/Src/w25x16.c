#include "w25x16.h"

extern SPI_HandleTypeDef hspi1;

/*一次读写一个字节*/
uint8_t sFLASH_SendByte(uint8_t byte)
{
	uint8_t TX_DATA = byte;
	uint8_t RX_DATA = 0;
	
	//发送的数据  接收的数据 数据大小 超时时间
	HAL_SPI_TransmitReceive(&hspi1, &TX_DATA, &RX_DATA, 1, 1000);
	
	return RX_DATA;//返回接收的数据
}

/*获取制造商设备ID*/
uint16_t sFLASH_ReadID(void)
{
	uint16_t FLASH_ID = 0;
	uint8_t temp0 = 0;
	uint8_t temp1 = 0;
	
	sFLASH_CS_LOW();
	
	sFLASH_SendByte(W25X_ManufactDeviceID);
	
	sFLASH_SendByte(sFLASH_DUMMY_BYTE);
	sFLASH_SendByte(sFLASH_DUMMY_BYTE);
	sFLASH_SendByte(sFLASH_DUMMY_BYTE);
	
	temp0 = sFLASH_SendByte(sFLASH_DUMMY_BYTE);
	temp1 = sFLASH_SendByte(sFLASH_DUMMY_BYTE);
	
	sFLASH_CS_HIGH();
	
	FLASH_ID = temp0 << 8 | temp1;
	
	return FLASH_ID;
}

/*写使能*/
void sFLASH_WriteEnable(void)
{
	sFLASH_CS_LOW();
	
	//写入启用指令
	sFLASH_SendByte(sFLASH_CMD_WREN);
	
	sFLASH_CS_HIGH();
}

/*等待写或者擦除完成*/
void sFLASH_WaitForEnd(void)
{
	uint8_t sr_value = 0;
	sFLASH_CS_LOW();
	
	//读取状态寄存器指令
	sFLASH_SendByte(sFLASH_CMD_RDSR);
	
	do{
		sr_value = sFLASH_SendByte(sFLASH_DUMMY_BYTE);
	}while(sr_value & sFLASH_BUSY_FLAG);//等待写或者擦除完成
	//sr_value = 1 在忙碌
	//sr_value = 0 空闲
	
	sFLASH_CS_HIGH();
}


/*擦除扇区,4 KB（4096 B）是 最小擦除粒度,每一位写1 一个字节255 0xFF*/
void sFLASH_EraseSector(uint32_t SectorAddr)
{	
	sFLASH_WriteEnable();
	
	sFLASH_CS_LOW();
	
	//发送擦除扇区指令 0x20
	sFLASH_SendByte(sFLASH_CMD_SE);
	
	//从高八位开始
	sFLASH_SendByte((SectorAddr >> 16) & 0xff);
	
	sFLASH_SendByte((SectorAddr >> 8) & 0xff);
	
	sFLASH_SendByte((SectorAddr >> 0) & 0xff);
	
	sFLASH_CS_HIGH();
	
	//等待擦除完成
	sFLASH_WaitForEnd();
}


/*读数据*/
void sFLASH_ReadBuffer(uint8_t* pBuffer, uint32_t ReadAddr, uint32_t NumByteToRead)
{
	sFLASH_CS_LOW();
	//发送读数据指令 0x03 
	sFLASH_SendByte(sFLASH_CMD_READ);
	
	//发送要读的地址，从高到低的地址位
	sFLASH_SendByte((ReadAddr >> 16) & 0xff);
	
	sFLASH_SendByte((ReadAddr >> 8) & 0xff);
	
	sFLASH_SendByte((ReadAddr >> 0) & 0xff);
	
	//循环读
	while(NumByteToRead--) //为0时结束
	{
		//发送空字节 获得连续自增的数据
		*pBuffer = sFLASH_SendByte(sFLASH_DUMMY_BYTE);
		pBuffer++;
	}
		
	sFLASH_CS_HIGH();
}

/*写数据,即页编程*/
void sFLASH_WritePage(uint8_t* pBuffer, uint32_t WriteAddr, uint32_t NumByteToWrite)
{
	
	//256 B 是 页编程缓冲区上限 0x100
	if(NumByteToWrite > sFLASH_SPI_PAGESIZE) 
	{
		NumByteToWrite = sFLASH_SPI_PAGESIZE;
		
		printf("数据量超过一页大小\n");
	}
	
	sFLASH_WriteEnable();
	sFLASH_CS_LOW();
	
	//发送页编程指令
	sFLASH_SendByte(sFLASH_CMD_WRITE);
	
	//发送高8位 中8位 低8位
	sFLASH_SendByte((WriteAddr >> 16) & 0xff);
	sFLASH_SendByte((WriteAddr >> 8) & 0xff);
	sFLASH_SendByte((WriteAddr >> 0) & 0xff);
	
	while(NumByteToWrite--) //直到写完字节数
	{
		sFLASH_SendByte(*pBuffer);
		pBuffer++;//1个字节1个字节的写 地址往后移动
	}
	
	sFLASH_CS_HIGH();
	
	//等待写完
	sFLASH_WaitForEnd();
}


/*任意位置开始写 突破限制*/
void sFLASH_WriteBuffer(uint8_t* pBuffer, uint32_t WriteAddr, uint32_t NumByteToWrite)
{
	/*记录页数，剩余字节，从页中间写*/
	uint16_t NumOfPage, NumOfBytes, count, offset;
	
	/*是否是一页的整数倍*/
	offset = WriteAddr % sFLASH_SPI_PAGESIZE;
	
	/*从页中间开始写的 剩余字节数 = 整页大小 - 地址始位*/
	count = sFLASH_SPI_PAGESIZE - (WriteAddr % sFLASH_SPI_PAGESIZE);
	
	//从页中间写，并且超过本页的边界，进入下一页
	/*页不对齐 且 大于一页的剩余字节数*/
	if(offset && (NumByteToWrite > count))
	{
		sFLASH_WritePage(pBuffer,WriteAddr,sFLASH_SPI_PAGESIZE);
		
		NumByteToWrite -= count;//减掉已经写的字节数
		
		pBuffer += count; //指针指向结束的位置
		WriteAddr += count;//记录地址下一次写开始的地方
	}
	
	
	NumOfPage = NumByteToWrite / sFLASH_SPI_PAGESIZE;
	NumOfBytes = NumByteToWrite % sFLASH_SPI_PAGESIZE;
	
	/*理想的从一页的整数倍开始写*/
	//整数页
	if(NumOfPage)
	{
		/*一页一页的写*/
		while(NumOfPage--)
		{
			/*写一页*/
			sFLASH_WritePage(pBuffer,WriteAddr,sFLASH_SPI_PAGESIZE);
			//pBuffer += sFLASH_SPI_PAGESIZE;
			WriteAddr += sFLASH_SPI_PAGESIZE;
		}
	}
	
	/*不满一页的部分*/
	if(NumOfBytes)
	{
		sFLASH_WritePage(pBuffer,WriteAddr,NumOfBytes);
	}
	
}






