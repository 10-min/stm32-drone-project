#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_i2c.h"
#include "stm32f1xx_hal_gpio.h" //���̵�

/*MPU6050 registers address*/
/*��κ� �������� �ʱⰪ 0x00 (���� PWR_MGMT_1, WHO_AM_I)*/


/**�����Ϳ��� write �ϴ� ��������**/

#define ADDRESS_SMPLRT_DIV 0x19    //���ø� ����Ʈ ���� -> ���̷� 4Hz ~ 8kHz, ���ӵ� 4Hz ~ 1kHz
#define ADDRESS_GYRO_CONFIG 0x1B   //���̷� ������ �������� ����
#define ADDRESS_ACCEL_CONFIG 0x1C  //�����ι��� �������� ����
#define ADDRESS_PWR_MGMT_1 0x6B    //�ʱ� �� 0x40, �Ŀ� ��� ����
#define ADDRESS_CONFIG 0x1A;

/**�����Ϳ��� read �ϴ� ��������**/

#define ADDRESS_WHO_AM_I 0x75 //�ʱ� �� 0x68, mpu6050�� �ּҰ� ���� �Ǿ� ����

/*���ӵ� �� ����Ǵ� ��������*/
#define ADDRESS_ACCEL_XOUT_H 0x3B  //ACCEL_XOUT[15:8]
#define ADDRESS_ACCEL_XOUT_L 0x3C  //ACCEL_XOUT[7:0]
#define ADDRESS_ACCEL_YOUT_H 0x3D  //ACCEL_YOUT[15:8]
#define ADDRESS_ACCEL_YOUT_L 0x3E  //ACCEL_YOUT[7:0]
#define ADDRESS_ACCEL_ZOUT_H 0x3F  //ACCEL_ZOUT[15:8]
#define ADDRESS_ACCEL_ZOUT_L 0x40  //ACCEL_ZOUT[7:0]

/*���ӵ� �� ����Ǵ� ��������*/
#define ADDRESS_GYRO_XOUT_H 0x43  //GYRO_XOUT[15:8]
#define ADDRESS_GYRO_XOUT_L 0x44  //GYRO_XOUT[7:0]
#define ADDRESS_GYRO_YOUT_H 0x45  //GYRO_YOUT[15:8]
#define ADDRESS_GYRO_YOUT_L 0x46  //GYRO_YOUT[7:0]
#define ADDRESS_GYRO_ZOUT_H 0x47  //GYRO_ZOUT[15:8]
#define ADDRESS_GYRO_ZOUT_L 0x48  //GYRO_ZOUT[7:0]

#define gyro_output_rate 8000 //8kHz(digital low pass filter �Ⱦ� ����) 


typedef struct //�������� ������ȯ ���ڸ� ����� ���� ����ü
{	
    //raw data���� �ٸ� ������ �ٲ��ִ� ����
	//(raw data) / (factor) -> ���� ��ȯ
	float gyro_change_unit_factor; //raw data -> deg/sec
	float accel_change_unit_factor; //raw data -> g (�߷°��ӵ�)

	//read_gyro, read_accel �Լ��� ����ϸ� ���� ������Ʈ ��
	int16_t gy_x;	 //���� raw data ����Ǵ� ����
	int16_t gy_y;
	int16_t gy_z;
	int16_t ac_x;
	int16_t ac_y;
	int16_t ac_z;
        
    float gy_x_dps; //������ deg/sec �϶�, ���� �� ����Ǵ� ����
    float gy_y_dps;
    float gy_z_dps;
        
    float ac_x_g;   //������ �߷� ���ӵ� �϶�, ���� �� ����Ǵ� ����
    float ac_y_g;
    float ac_z_g;
	
}mpu6050;


typedef enum //���� �� ���� ����
{
    raw_data = 0x00, // 16��Ʈ -32768 ~ 32767
    deg_per_sec = 0x01, //1�� �� ������ ����
    gravity_acceleration = 0x02 //�߷� ���ӵ�
          
}unit;

typedef enum //���̷� ���� ������ ���� ����
{
	gyro_full_scale_range_250 = 0x00,  //��250 deg/sec
	gyro_full_scale_range_500 = 0x01,  //��500 deg/sec
	gyro_full_scale_range_1000 = 0x02, //��1000 deg/sec
	gyro_full_scale_range_2000 = 0x03, //��2000 deg/sec

}gyro_full_scale_range;

typedef enum //���ӵ� ���� ������ ���� ����
{
	accel_full_scale_range_2g = 0x00,  //��2g (���� : �߷°��ӵ�)
	accel_full_scale_range_4g = 0x01,  //��4g
	accel_full_scale_range_8g = 0x02,  //��8g
	accel_full_scale_range_16g = 0x03  //��16g

}accel_full_scale_range;



/*i2c���, MPU6050 ���� Ȯ��*/
void WHO_AM_I(I2C_HandleTypeDef* hi2c); 

/*sleep mode -> waking up*/
void wake_up(I2C_HandleTypeDef* hi2c); 

/*���ϴ� ���� ����Ʈ ����*/
void set_sample_rate(I2C_HandleTypeDef* hi2c, uint16_t sample_rate_you_want); 

/*���̷ο� ���ӵ��� ���� ���� ����*/
void set_sensitivity(I2C_HandleTypeDef* hi2c, mpu6050* __my_mpu6050, gyro_full_scale_range gyro_range_you_want, accel_full_scale_range accel_range_you_want); 

/*���̷� ���� �� �б�*/
void read_gyro(I2C_HandleTypeDef* hi2c, mpu6050* __my_mpu6050, unit unit_you_want); 

/*���ӵ� ���� �� �б�*/
void read_accel(I2C_HandleTypeDef* hi2c, mpu6050* __my_mpu6050, unit unit_you_want); 

void set_DLPF(I2C_HandleTypeDef* hi2c, uint8_t value);



