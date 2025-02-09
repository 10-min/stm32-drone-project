#include "mokhw_MPU6050.h"


/*i2c���, MPU6050 ���� Ȯ��*/
void WHO_AM_I(I2C_HandleTypeDef* hi2c)
{
	uint8_t slave_address = 0xD0; //0x68 �������� ��ĭ ������
	uint8_t register_to_access = ADDRESS_WHO_AM_I; //������ ��������(WHO_AM_I)�� �ּ�
	uint8_t temp = 0; //WHO_AM_I�� ����� ���� �� ������ ����

	HAL_I2C_Master_Transmit(hi2c, slave_address, &register_to_access, 1, 1000); //�������� ���������� �ּҰ� ������
	HAL_I2C_Master_Receive(hi2c, slave_address, &temp, 1, 1000); //WHO_AM_I�� ������ִ°�(0x68) temp�� ����

	while (temp != 0x68); //���� ���� 0x68�� �ٸ��� ���� �߸� ����
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); //���� ���� 0x68�̶�� �� ����, LED ����
														//PA5�� STM32f103RBT6�� ���� LED
}

/*sleep mode -> waking up*/
void wake_up(I2C_HandleTypeDef* hi2c)
{
        uint8_t slave_address = 0xD0; //0x68 �������� ��ĭ ������
        uint8_t data_to_write[2]; //�������� �ּҿ� �� �������Ϳ� ���� �־��� ���� �����ϴ� �迭

        data_to_write[0] = ADDRESS_PWR_MGMT_1; //������ ��������(PWR_MGMT_1)�� �ּ�
        data_to_write[1] = 0x00; //�ʱ� �� 0x40 -> 0x00���� �ٲ��� : wake_up
        
        //�迭�� �̿��ؼ� �������� �ּҿ� �ش�Ǵ� ���������� ���ο� ���� �ѹ��� ����
        HAL_I2C_Master_Transmit(hi2c, slave_address, data_to_write, 2, 1000); 
        //�ش� �������� �� �ٲ�
}


/*���ϴ� ���� ����Ʈ ����*/
void set_sample_rate(I2C_HandleTypeDef* hi2c, uint16_t sample_rate_you_want) 
{
	uint8_t slave_address = 0xD0; //0x68 �������� ��ĭ ������
	uint8_t data_to_write[2];
        
	/*sample rate = gyro output rate / (1 + SMPLRT_DIV)*/
    /*���ϴ� ���� ����Ʈ�� �����ϱ� ����, SMPLRT_DIV�� �� ���� ���ϴ� ��*/
    uint8_t calculated_SMPLRT_DIV = (gyro_output_rate/sample_rate_you_want) - 1;
        
	data_to_write[0] = ADDRESS_SMPLRT_DIV; //������ ��������(SMPLRT_DIV)�� �ּ�
	data_to_write[1] = calculated_SMPLRT_DIV; //�ش�Ǵ� ���������� ���ο� ��

    //�迭�� �̿��ؼ� �������� �ּҿ� �ش�Ǵ� ���������� ���ο� ���� �ѹ��� ����
	HAL_I2C_Master_Transmit(hi2c, slave_address, data_to_write, 2, 1000); 
    //�������� �� �ٲ�
}

/*���̷ο� ���ӵ��� ���� ���� ����*/
void set_sensitivity(I2C_HandleTypeDef* hi2c, mpu6050* __my_mpu6050, gyro_full_scale_range gyro_range_you_want, accel_full_scale_range accel_range_you_want)
{
	//���̷� ������ �ΰ��� ���ϱ�
	uint8_t slave_address = 0xD0; //0x68 �������� ��ĭ ������
	uint8_t data_to_write[3];

	data_to_write[0] = ADDRESS_GYRO_CONFIG; //������ ��������(GYRO_CONFIG)�� �ּ�
	data_to_write[1] = gyro_range_you_want <<3; //�ش�Ǵ� ���������� ���ο� ��
	data_to_write[2] = accel_range_you_want <<3; //�ش�Ǵ� ���������� ���ο� ��

    //�迭�� �̿��ؼ� �������� �ּҿ� �ش�Ǵ� ���������� ���ο� ���� �ѹ��� ����
	HAL_I2C_Master_Transmit(hi2c, slave_address, data_to_write, 3, 1000);
    //�������� �� �ٲ�


	//���̷� ���� ������ ���� gyro_change_unit_factor �� �ٲ��ֱ�
	switch (gyro_range_you_want) 
	{
	case gyro_full_scale_range_250 :
		__my_mpu6050->gyro_change_unit_factor = 131;
		break;

	case gyro_full_scale_range_500 :
		__my_mpu6050->gyro_change_unit_factor = 65.5;
		break;

	case gyro_full_scale_range_1000:
		__my_mpu6050->gyro_change_unit_factor = 32.8;
		break;

	case gyro_full_scale_range_2000:
		__my_mpu6050->gyro_change_unit_factor = 16.4;
		break;

	default :
		break;
	}

	//���ӵ� ���� ������ ���� accel_change_unit_factor �� �ٲ��ֱ�
	switch (accel_range_you_want) 
	{
	case accel_full_scale_range_2g :
		__my_mpu6050->accel_change_unit_factor = 16384;
		break;

	case accel_full_scale_range_4g:
		__my_mpu6050->accel_change_unit_factor = 8192;
		break;

	case accel_full_scale_range_8g:
		__my_mpu6050->accel_change_unit_factor = 4096;
		break;

	case accel_full_scale_range_16g:
		__my_mpu6050->accel_change_unit_factor = 2048;
		break;

	default :
		break;
	}

}

void set_DLPF (I2C_HandleTypeDef* hi2c, uint8_t value) {
	uint8_t slave_address = 0xD0;
	uint8_t data_to_write[2];

	data_to_write[0] = ADDRESS_CONFIG;

	for (int i = 2; i <= 7; i++) {
		data_to_write[1] = (i << 3) | value;
		HAL_I2C_Master_Transmit(hi2c, slave_address, data_to_write, 2, 1000);
	}

}



/*���̷ΰ� �б�*/
void read_gyro(I2C_HandleTypeDef* hi2c, mpu6050* __my_mpu6050, unit unit_you_want)
{
	uint8_t slave_address = 0xD0; //0x68 �������� ��ĭ ������
	uint8_t register_to_access = ADDRESS_GYRO_XOUT_H; //������ ��������(GYRO_XOUT_H)�� �ּ�
	uint8_t data_to_read[6]; //���̷� �� �����ϴ� �迭

	//���� �������� �ּ�(GYRO_XOUT_H) ������
	HAL_I2C_Master_Transmit(hi2c, slave_address, &register_to_access, 1, 1000); 

	//�ش��ϴ� ���������� �����͸� �迭�� ����
	HAL_I2C_Master_Receive(hi2c, slave_address, data_to_read, 6, 1000);

	switch (unit_you_want) //���� ���� ���ϴ� ������ �ٲ���
	{
	case raw_data : //�ٷ� ����
		__my_mpu6050->gy_x = (int16_t)(data_to_read[0] << 8 | data_to_read[1]);
		__my_mpu6050->gy_y = (int16_t)(data_to_read[2] << 8 | data_to_read[3]);
		__my_mpu6050->gy_z = (int16_t)(data_to_read[4] << 8 | data_to_read[5]);
		break;

	case deg_per_sec : //���� ��ȯ�� ���� raw data�� gyro_change_unit_factor�� ���� ���� ����
		__my_mpu6050->gy_x_dps = (int16_t)(data_to_read[0] << 8 | data_to_read[1]) / __my_mpu6050->gyro_change_unit_factor;
		__my_mpu6050->gy_y_dps = (int16_t)(data_to_read[2] << 8 | data_to_read[3]) / __my_mpu6050->gyro_change_unit_factor;
		__my_mpu6050->gy_z_dps = (int16_t)(data_to_read[4] << 8 | data_to_read[5]) / __my_mpu6050->gyro_change_unit_factor;
		break;

	default :
		break;
	}
}


/*���ӵ� �� �б�*/
void read_accel(I2C_HandleTypeDef* hi2c, mpu6050* __my_mpu6050, unit unit_you_want)
{
	uint8_t slave_address = 0xD0; //0x68 �������� ��ĭ ������
	uint8_t register_to_access = ADDRESS_ACCEL_XOUT_H;//������ ��������(ACCEL_XOUT_H)�� �ּ�
	uint8_t data_to_read[6]; //���ӵ� �� �����ϴ� �迭

	 //���� �������� �ּ�(ACCEL_XOUT_H) ������
	HAL_I2C_Master_Transmit(hi2c, slave_address, &register_to_access, 1, 1000);

	//�ش��ϴ� ���������� �����͸� �迭�� ����
	HAL_I2C_Master_Receive(hi2c, slave_address, data_to_read, 6, 1000);

    //16��Ʈ ���� �����Ͱ� 8��Ʈ�� �������� �־
    //ó�� ���� �����͸� �������� 8ĭ �������ϰ� 2��°�� ���� �����͸� ��ģ��.

	switch (unit_you_want) //���� ���� ���ϴ� ������ �ٲ���
	{
	case raw_data : //�ٷ� ����
		__my_mpu6050->ac_x = data_to_read[0] << 8 | data_to_read[1];
		__my_mpu6050->ac_y = data_to_read[2] << 8 | data_to_read[3];
		__my_mpu6050->ac_z = data_to_read[4] << 8 | data_to_read[5];
		break;

	case gravity_acceleration : //���� ��ȯ�� ���� raw data�� accel_change_unit_factor�� ���� ���� ����
		__my_mpu6050->ac_x_g = (int16_t)(data_to_read[0] << 8 | data_to_read[1]) / __my_mpu6050->accel_change_unit_factor;
		__my_mpu6050->ac_y_g = (int16_t)(data_to_read[2] << 8 | data_to_read[3]) / __my_mpu6050->accel_change_unit_factor;
		__my_mpu6050->ac_z_g = (int16_t)(data_to_read[4] << 8 | data_to_read[5]) / __my_mpu6050->accel_change_unit_factor;
		break;

	default :
		break;
	}
}
