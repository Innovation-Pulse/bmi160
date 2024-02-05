/*!
 * @file accelGyro.ino
 * @brief I2C addr:
 * @n  0x68: connect SDIO pin of the BMI160 to GND which means the default I2C address
 * @n  0x69: set I2C address by parameter
 * @n Through the example, you can get the sensor data by using getSensorData:
 * @n get acell by paremeter onlyAccel;
 * @n get gyro by paremeter onlyGyro;
 * @n get both acell and gyro by paremeter bothAccelGyro.
 * @n With the rotation of the sensor, data changes are visible.
 * @copyright	Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license The MIT License (MIT)
 * @author  DFRobot_haoJ(hao.jiang@dfrobot.com)
 * @version V1.0
 * @date 2017-12-01
 * @url https://github.com/DFRobot/DFRobot_BMI160
 */



#include <SPI.h>
#include <DFRobot_BMI160.h>

DFRobot_BMI160 bmi160;
const int8_t i2c_addr = 0x69;

const float Kp = 3.5, Ki = 0.05;
// #include <SPI.h>
// #include <DFRobot_BMI160.h>

// DFRobot_BMI160 bmi160;
// const int8_t spi_cs_pin = 10; // 选择SPI片选引脚

// const float Kp = 3.5, Ki = 0.05;
// 其他变量保持不变

float exInt, eyInt, ezInt;
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // roll,pitch,yaw 都为 0 时对应的四元数值。

float pitch, yaw, roll;

void IMUupdate(float gx, float gy, float gz, float ax,float ay, float az)
{
    float halfT = 0.005;
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;

    float q0q0 = q0*q0;
    float q0q1 = q0*q1;
    float q0q2 = q0*q2;
    float q1q1 = q1*q1;
    float q1q3 = q1*q3;
    float q2q2 = q2*q2;
    float q2q3 = q2*q3;
    float q3q3 = q3*q3;

    if(ax*ay*az==0)
        return;

    // 第一步：对加速度数据进行归一化
    norm = sqrt(ax*ax + ay*ay + az*az); 
    ax = ax / norm; 
    ay = ay / norm; 
    az = az / norm; 

    // 第二步：DCM矩阵旋转
    vx = 2*(q1q3 - q0q2); 
    vy = 2*(q0q1 + q2q3); 
    vz = q0q0 - q1q1 - q2q2 + q3q3 ;

    // 第三步：在机体坐标系下做向量叉积得到补偿数据
    ex = ay*vz - az*vy ;
    ey = az*vx - ax*vz ;
    ez = ax*vy - ay*vx ;

    // 第四步：对误差进行PI计算，补偿角速度
    exInt = exInt + ex * Ki;
    eyInt = eyInt + ey * Ki;
    ezInt = ezInt + ez * Ki;

    gx = gx + Kp*ex + exInt;
    gy = gy + Kp*ey + eyInt;
    gz = gz + Kp*ez + ezInt;

    // 第五步：按照四元数微分公式进行四元数更新
    q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
    q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
    q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

    norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 = q0/norm;
    q1 = q1/norm;
    q2 = q2/norm;
    q3 = q3/norm;

    roll =  atan2f(2*q2*q3 + 2*q0*q1, -2*q1*q1 - 2*q2*q2 + 1)*57.3;     
    pitch =  asinf(2*q1*q3 - 2*q0*q2)*57.3;                                                          
    yaw  =  -atan2f(2*q1*q2 + 2*q0*q3, -2*q2*q2 -2*q3*q3 + 1)*57.3; 

    //Serial.print("pitch: ");
    Serial.print(pitch);Serial.print("\t");
   // Serial.print("yaw: ");
    Serial.print(yaw);Serial.print("\t");
   // Serial.print("roll: ");
    Serial.println(roll);
}


void setup(){
  Serial.begin(115200);
  delay(100);
  
  //init the hardware bmin160  
  if (bmi160.softReset() != BMI160_OK){
    Serial.println("reset false");
    while(1);
  }
  
  //set and init the bmi160 i2c address
  if (bmi160.I2cInit(i2c_addr) != BMI160_OK){
    Serial.println("init false");
    while(1);
  }
}

// void setup() {
//     Serial.begin(115200);
//     delay(100);

//     // 初始化SPI
//     SPI.begin();
//     pinMode(spi_cs_pin, OUTPUT);
//     digitalWrite(spi_cs_pin, HIGH); // 设置片选引脚为高电平

//     // 初始化BMI160传感器
//     if (bmi160.SPIInit() != BMI160_OK) {
//         Serial.println("BMI160 SPI init failed");
//         while (1);
//     }
// }


// void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az) {
//     // 其他代码保持不变
// }

void loop(){  
  int i = 0;
  int rslt;
  int16_t accelGyro[6]={0}; 
  
  //get both accel and gyro data from bmi160
  //parameter accelGyro is the pointer to store the data
  rslt = bmi160.getAccelGyroData(accelGyro);
  if(rslt == 0){
    for(i=0;i<6;i++){
      if (i<3){
        //the first three are gyro data
        Serial.print(accelGyro[i]*3.14/180.0);Serial.print("\t");
      }else{
        //the following three data are accel data
        //Serial.print(accelGyro[i]/16384.0);Serial.print("\t");
      }
    }
    //Serial.println();
  }else{
    Serial.println("err");
  }
  IMUupdate(accelGyro[0]*3.14/180.,accelGyro[1]*3.14/180.,accelGyro[2]*3.14/180.,accelGyro[3]/16384.0,accelGyro[4]/16384.0,accelGyro[5]/16384.0);
  
  delay(100);
  /*
   * //only read accel data from bmi160
   * int16_t onlyAccel[3]={0};
   * bmi160.getAccelData(onlyAccel);
   */

  /*
   * ////only read gyro data from bmi160
   * int16_t onlyGyro[3]={0};
   * bmi160.getGyroData(onlyGyro);
   */
}
// void loop() {
//     int i = 0;
//     int rslt;
//     int16_t accelGyro[6] = {0};

//     // 从BMI160获取加速度和陀螺仪数据
//     rslt = bmi160.getAccelGyroData(accelGyro);
    
//     if (rslt == 0) {
//         // 处理数据的部分保持不变
//         IMUupdate(accelGyro[0] * 3.14 / 180., accelGyro[1] * 3.14 / 180., accelGyro[2] * 3.14 / 180., accelGyro[3] / 16384.0, accelGyro[4] / 16384.0, accelGyro[5] / 16384.0);
//     } else {
//         Serial.println("Error reading BMI160 data");
//     }
  
//     delay(100);
// }



