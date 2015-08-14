/**	
 * |----------------------------------------------------------------------
 * | Copyright (C) Tilen Majerle, 2014
 * | 
 * | This program is free software: you can redistribute it and/or modify
 * | it under the terms of the GNU General Public License as published by
 * | the Free Software Foundation, either version 3 of the License, or
 * | any later version.
 * |  
 * | This program is distributed in the hope that it will be useful,
 * | but WITHOUT ANY WARRANTY; without even the implied warranty of
 * | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * | GNU General Public License for more details.
 * | 
 * | You should have received a copy of the GNU General Public License
 * | along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * |----------------------------------------------------------------------
 */
#include "tm_stm32f4_mpu6050.h"

TM_MPU6050_Result_t TM_MPU6050_Init(TM_MPU6050_t* DataStruct, TM_MPU6050_Device_t DeviceNumber, TM_MPU6050_Accelerometer_t AccelerometerSensitivity, TM_MPU6050_Gyroscope_t GyroscopeSensitivity) {
	uint8_t temp;
	
	/* Format I2C address */
	DataStruct->Address = MPU6050_I2C_ADDR | (uint8_t)DeviceNumber;
	
	/* Initialize I2C */
	TM_I2C_Init(MPU6050_I2C, MPU6050_I2C_PINSPACK, MPU6050_I2C_CLOCK);
	
	/* Check if device is connected */
	if (!TM_I2C_IsDeviceConnected(MPU6050_I2C, DataStruct->Address)) {
		/* Return error */
		return TM_MPU6050_Result_DeviceNotConnected;
	}
	
	/* Check who I am */
	if (TM_I2C_Read(MPU6050_I2C, DataStruct->Address, MPU6050_WHO_AM_I) != MPU6050_I_AM) {
		/* Return error */
		return TM_MPU6050_Result_DeviceInvalid;
	}
	
	/* Wakeup MPU6050 */
	TM_I2C_Write(MPU6050_I2C, DataStruct->Address, MPU6050_PWR_MGMT_1, 0x00);
	
	/* Config accelerometer */
	temp = TM_I2C_Read(MPU6050_I2C, DataStruct->Address, MPU6050_ACCEL_CONFIG);
	temp = (temp & 0xE7) | (uint8_t)AccelerometerSensitivity << 3;
	TM_I2C_Write(MPU6050_I2C, DataStruct->Address, MPU6050_ACCEL_CONFIG, temp);
	
	/* Config gyroscope */
	temp = TM_I2C_Read(MPU6050_I2C, DataStruct->Address, MPU6050_GYRO_CONFIG);
	temp = (temp & 0xE7) | (uint8_t)GyroscopeSensitivity << 3;
	TM_I2C_Write(MPU6050_I2C, DataStruct->Address, MPU6050_GYRO_CONFIG, temp);
	
	/* Set sensitivities for multiplying gyro and accelerometer data */
	switch (AccelerometerSensitivity) {
		case TM_MPU6050_Accelerometer_2G:
			DataStruct->Acce_Mult = (float)1 / MPU6050_ACCE_SENS_2; 
			break;
		case TM_MPU6050_Accelerometer_4G:
			DataStruct->Acce_Mult = (float)1 / MPU6050_ACCE_SENS_4; 
			break;
		case TM_MPU6050_Accelerometer_8G:
			DataStruct->Acce_Mult = (float)1 / MPU6050_ACCE_SENS_8; 
			break;
		case TM_MPU6050_Accelerometer_16G:
			DataStruct->Acce_Mult = (float)1 / MPU6050_ACCE_SENS_16; 
		default:
			break;
	}
	
	switch (GyroscopeSensitivity) {
		case TM_MPU6050_Gyroscope_250s:
			DataStruct->Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_250; 
			break;
		case TM_MPU6050_Gyroscope_500s:
			DataStruct->Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_500; 
			break;
		case TM_MPU6050_Gyroscope_1000s:
			DataStruct->Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_1000; 
			break;
		case TM_MPU6050_Gyroscope_2000s:
			DataStruct->Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_2000; 
		default:
			break;
	}
	
        /*Aziz: 20/07/15
            Applying low pass filter to 5Hz*/
        temp = TM_I2C_Read(MPU6050_I2C, DataStruct->Address, MPU6050_CONFIG);
	//temp = (temp & 0xE7) | (uint8_t)AccelerometerSensitivity << 3;
        TM_I2C_Write(MPU6050_I2C, DataStruct->Address, MPU6050_CONFIG, 0x06);
        
        //read gyro scope settings
        temp = TM_I2C_Read(MPU6050_I2C, DataStruct->Address, MPU6050_GYRO_CONFIG); 
        /*Aziz: 20/07/15
            setting gyro offset to zero*/
        temp = TM_I2C_Read(MPU6050_I2C, DataStruct->Address, 0x13);
        TM_I2C_Write(MPU6050_I2C, DataStruct->Address, 0x13, 0x00);
        TM_I2C_Write(MPU6050_I2C, DataStruct->Address, 0x14, 0x00);
        TM_I2C_Write(MPU6050_I2C, DataStruct->Address, 0x15, 0x00);
        TM_I2C_Write(MPU6050_I2C, DataStruct->Address, 0x16, 0x00);
        TM_I2C_Write(MPU6050_I2C, DataStruct->Address, 0x17, 0x00);
        TM_I2C_Write(MPU6050_I2C, DataStruct->Address, 0x18, 0x00);
        
        //Calibrate
         MPU6050_Calibration(DataStruct);
        
        
        
        /* Return OK */
	return TM_MPU6050_Result_Ok;
}

void MPU6050_Calibration(TM_MPU6050_t* DataStruct){
  float GyroAvg_z = 0.0;
  float GyroAvg_x = 0.0;
  float GyroAvg_y = 0.0;
  
  volatile int16_t AcclAvg_x=0;
  volatile int16_t AcclAvg_y=0;
  volatile  int16_t AcclAvg_z=0;
  
  volatile uint8_t XG_OFFS_H;
  volatile uint8_t XG_OFFS_L;
  volatile uint8_t YG_OFFS_H;
  volatile uint8_t YG_OFFS_L;
  volatile uint8_t ZG_OFFS_H;
  volatile uint8_t ZG_OFFS_L;
  
  volatile int16_t AccelFac_x=0;
  volatile int16_t AccelFac_y=0;
  volatile int16_t AccelFac_z=0;
  
  
  float OffsetVal_x = 0.0;
  float OffsetVal_y = 0.0;
  float OffsetVal_z = 0.0;
  uint8_t OffsetValH=0;
  uint8_t OffsetValL=0;
  uint16_t temp=0;
  uint8_t i =0;
  /*attempt to calibrate the Gyro
  TODO: currently only works for 250 dps, to make it work with other ranges change 131.072
  */
  //1 get  readings and get an avarage value
  for(i=0;i<5;i++){
    //Delay(20);
    TM_MPU6050_ReadAll(DataStruct);
    GyroAvg_x +=(float) DataStruct->Gyroscope_X/131.072;
    GyroAvg_y +=(float) DataStruct->Gyroscope_Y/131.072;
    GyroAvg_z +=(float) DataStruct->Gyroscope_Z/131.072;
  }
  GyroAvg_x = GyroAvg_x/5.0;     //get avarage
  GyroAvg_y = GyroAvg_y/5.0;     //get avarage
  GyroAvg_z = GyroAvg_z/5.0;     //get avarage
  //2 convert the value into offset value
  GyroAvg_x = -GyroAvg_x;        //invert the sign
  GyroAvg_y = -GyroAvg_y;        //invert the sign
  GyroAvg_z = -GyroAvg_z;        //invert the sign
  
  OffsetVal_x= 32.8 * GyroAvg_x;  //1 dps = 32.8 in offset register
  OffsetVal_y= 32.8 * GyroAvg_y;  //1 dps = 32.8 in offset register 
  OffsetVal_z= 32.8 * GyroAvg_z;  //1 dps = 32.8 in offset register 
  
  //3 apply the offsets into the corect registers
  //Apply to X offset register
  temp = (uint16_t)OffsetVal_x;
  OffsetValH = (temp >> 8) ; 
  OffsetValL = temp & 0x00FF;
  TM_I2C_Write(MPU6050_I2C, DataStruct->Address, 0x13, OffsetValH);   
  TM_I2C_Write(MPU6050_I2C, DataStruct->Address, 0x14, OffsetValL);
  
  //Apply to Y offset register
  temp = (uint16_t)OffsetVal_y;
  OffsetValH = (temp >> 8) ; 
  OffsetValL = temp & 0x00FF;
  TM_I2C_Write(MPU6050_I2C, DataStruct->Address, 0x15, OffsetValH);   
  TM_I2C_Write(MPU6050_I2C, DataStruct->Address, 0x16, OffsetValL);
  
  //Apply to Z offset register
  temp = (uint16_t)OffsetVal_z;
  OffsetValH = (temp >> 8) ; 
  OffsetValL = temp & 0x00FF;
  TM_I2C_Write(MPU6050_I2C, DataStruct->Address, 0x17, OffsetValH);   
  TM_I2C_Write(MPU6050_I2C, DataStruct->Address, 0x18, OffsetValL);
  
  
  
  
  /*attempt to calibrate the Accelerometer
  Factory settings +- Current readings = Offset value
  */
  
  //1 read the factory settings from the offet registers
  XG_OFFS_H = TM_I2C_Read(MPU6050_I2C, DataStruct->Address, XG_OFFS_USRH);
  XG_OFFS_L = TM_I2C_Read(MPU6050_I2C, DataStruct->Address, XG_OFFS_USRL);
  //merge the values
  AccelFac_x = (XG_OFFS_H<<8) | XG_OFFS_L;

  YG_OFFS_H = TM_I2C_Read(MPU6050_I2C, DataStruct->Address, YG_OFFS_USRH);
  YG_OFFS_L = TM_I2C_Read(MPU6050_I2C, DataStruct->Address, YG_OFFS_USRL);
  //merge the values
  AccelFac_y = (YG_OFFS_H<<8) | YG_OFFS_L;
  
  ZG_OFFS_H = TM_I2C_Read(MPU6050_I2C, DataStruct->Address, ZG_OFFS_USRH);
  ZG_OFFS_L = TM_I2C_Read(MPU6050_I2C, DataStruct->Address, ZG_OFFS_USRL);
  //merge the values
  AccelFac_z = (ZG_OFFS_H<<8) | ZG_OFFS_L;
  
  //2 read current accl values then calculate offset 
  //  for(i=0;i<5;i++){
  //    Delay(20);
  TM_MPU6050_ReadAll(DataStruct);
  AcclAvg_x =(DataStruct->Accelerometer_X)/16;
  AcclAvg_y =(DataStruct->Accelerometer_Y)/16;
  AcclAvg_z =(DataStruct->Accelerometer_Z)/16;
  //  }
  //  //get avarage
  //  AcclAvg_x = AcclAvg_x/5;
  //  AcclAvg_y = AcclAvg_y/5;
  //  AcclAvg_z = AcclAvg_z/5;
  
  //convert values to +- 8G. where 1mg = 4096 LSB
//  AcclAvg_x = -AcclAvg_x;
//  AcclAvg_y = -AcclAvg_y;
//  AcclAvg_z = -AcclAvg_z;
  
  
  
  
  //calculate X axis offset
  volatile int16_t AccOffset_x = 0;
  AccelFac_x -=(AcclAvg_x & ~1);
 // AccOffset_x= AccelFac_x + AcclAvg_x;
  OffsetValH = AccelFac_x >> 8;
  OffsetValL = AccelFac_x & 0x00FF;
    
  TM_I2C_Write(MPU6050_I2C, DataStruct->Address, XG_OFFS_USRH, OffsetValH);   
  TM_I2C_Write(MPU6050_I2C, DataStruct->Address, XG_OFFS_USRL, OffsetValL);
  
  //calculate Y axis offset
  int16_t AccOffset_y = 0;
  AccelFac_y -=(AcclAvg_y & ~1);
  //AccOffset_y= AccelFac_y + AcclAvg_y;
  OffsetValH = AccelFac_y >> 8;
  OffsetValL = AccelFac_y & 0x00FF;
    
  TM_I2C_Write(MPU6050_I2C, DataStruct->Address, YG_OFFS_USRH, OffsetValH);   
 TM_I2C_Write(MPU6050_I2C, DataStruct->Address, YG_OFFS_USRL, OffsetValL);
  
  //calculate Z axis offset
  int16_t AccOffset_z = 0;
  AccelFac_z -=(AcclAvg_z & ~1);
  //AccOffset_z= AccelFac_z + AcclAvg_z;
  OffsetValH = AccelFac_z >> 8;
  OffsetValL = AccelFac_z & 0x00FF;
  
  TM_I2C_Write(MPU6050_I2C, DataStruct->Address, ZG_OFFS_USRH, OffsetValH);   
  TM_I2C_Write(MPU6050_I2C, DataStruct->Address, ZG_OFFS_USRL, OffsetValL);
  //3 Apply the values

}

TM_MPU6050_Result_t TM_MPU6050_ReadAccelerometer(TM_MPU6050_t* DataStruct) {
	uint8_t data[6];
	
	/* Read accelerometer data */
	TM_I2C_ReadMulti(MPU6050_I2C, DataStruct->Address, MPU6050_ACCEL_XOUT_H, data, 6);
	
	/* Format */
	DataStruct->Accelerometer_X = (int16_t)(data[0] << 8 | data[1]);	
	DataStruct->Accelerometer_Y = (int16_t)(data[2] << 8 | data[3]);
	DataStruct->Accelerometer_Z = (int16_t)(data[4] << 8 | data[5]);
	
	/* Return OK */
	return TM_MPU6050_Result_Ok;
}

TM_MPU6050_Result_t TM_MPU6050_ReadGyroscope(TM_MPU6050_t* DataStruct) {
	uint8_t data[6];
	
	/* Read gyroscope data */
	TM_I2C_ReadMulti(MPU6050_I2C, DataStruct->Address, MPU6050_GYRO_XOUT_H, data, 6);
	
	/* Format */
	DataStruct->Gyroscope_X = (int16_t)(data[0] << 8 | data[1]);
	DataStruct->Gyroscope_Y = (int16_t)(data[2] << 8 | data[3]);
	DataStruct->Gyroscope_Z = (int16_t)(data[4] << 8 | data[5]);

	/* Return OK */
	return TM_MPU6050_Result_Ok;
}

TM_MPU6050_Result_t TM_MPU6050_ReadTemperature(TM_MPU6050_t* DataStruct) {
	uint8_t data[2];
	int16_t temp;
	
	/* Read temperature */
	TM_I2C_ReadMulti(MPU6050_I2C, DataStruct->Address, MPU6050_TEMP_OUT_H, data, 2);
	
	/* Format temperature */
	temp = (data[0] << 8 | data[1]);
	DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);
	
	/* Return OK */
	return TM_MPU6050_Result_Ok;
}

TM_MPU6050_Result_t TM_MPU6050_ReadAll(TM_MPU6050_t* DataStruct) {
	uint8_t data[14];
	int16_t temp;
	
	/* Read full raw data, 14bytes */
	TM_I2C_ReadMulti(MPU6050_I2C, DataStruct->Address, MPU6050_ACCEL_XOUT_H, data, 14);
	
	/* Format accelerometer data */
	DataStruct->Accelerometer_X = (int16_t)(data[0] << 8 | data[1]);	
	DataStruct->Accelerometer_Y = (int16_t)(data[2] << 8 | data[3]);
	DataStruct->Accelerometer_Z = (int16_t)(data[4] << 8 | data[5]);

	/* Format temperature */
	temp = (data[6] << 8 | data[7]);
	DataStruct->Temperature = (float)((float)((int16_t)temp) / (float)340.0 + (float)36.53);
	
	/* Format gyroscope data */
	DataStruct->Gyroscope_X = (int16_t)(data[8] << 8 | data[9]);
	DataStruct->Gyroscope_Y = (int16_t)(data[10] << 8 | data[11]);
	DataStruct->Gyroscope_Z = (int16_t)(data[12] << 8 | data[13]);

	/* Return OK */
	return TM_MPU6050_Result_Ok;
}
