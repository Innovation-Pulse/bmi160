#include <BMI160Gen.h>
#include <SPI.h>

const int irq_pin = 30;
const int i2c_addr = 0x69;

float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // 初始四元数为 [1, 0, 0, 0]

void setup() {
  Serial.begin(115200); // 初始化串口通信
  BMI160.begin(BMI160GenClass::SPI_MODE, 10); // 使用 SPI 模式，SS pin = 10
  BMI160.setGyroRange(2000); // 设置陀螺仪量程为 2000
}

void loop() {
  float gx, gy, gz; // 陀螺仪数据
  float ax, ay, az; // 加速度计数据

  // 读取陀螺仪数据
  int gxRaw, gyRaw, gzRaw;
  BMI160.readGyro(gxRaw, gyRaw, gzRaw);
  gx = convertRawGyro(gxRaw);
  gy = convertRawGyro(gyRaw);
  gz = convertRawGyro(gzRaw);

  // 读取加速度计数据
  int axRaw, ayRaw, azRaw;
  BMI160.readAccelerometer(axRaw, ayRaw, azRaw);
  ax = convertRawAccel(axRaw);
  ay = convertRawAccel(ayRaw);
  az = convertRawAccel(azRaw);

  // 使用陀螺仪数据和加速度计数据进行姿态更新
  updateIMU(gx, gy, gz, ax, ay, az);

  // 输出姿态信息
  //Serial.print("Yaw: ");
  Serial.print(getYaw());
  Serial.print("\t");
  // Serial.print(",");
  Serial.print(getPitch());
  Serial.print("\t");
  // Serial.print(",");
  Serial.println(getRoll());

  delay(100);
}

// 以下函数用于将原始传感器数据转换为实际物理量
float convertRawGyro(int gRaw) {
  return (gRaw * 2000.0) / 32767.0;
}

float convertRawAccel(int aRaw) {
  return ((aRaw * 2.0) / 32768.0) * 9.8;
}

// 以下函数用于更新姿态信息
void updateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
  // 根据陀螺仪数据和加速度计数据进行姿态更新的算法
  // 这里省略了实际的姿态更新逻辑，可以使用卡尔曼滤波器、互补滤波器等方法进行姿态更新
  // 你需要根据你的实际需求和算法来实现这个函数
  // 具体的算法需要根据你的 IMU、姿态表示和工程需求进行选择和优化
  const float Kp = 3.5, Ki = 0.05;
  float exInt, eyInt, ezInt;


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
  
}

// 以下函数用于获取姿态信息（角度）
float getRoll() {
  return atan2(2 * (q0*q1 + q2*q3), 1 - 2 * (q1*q1 + q2*q2)) * 180 / M_PI;
}

float getPitch() {
  return asin(2 * (q0*q2 - q3*q1)) * 180 / M_PI;
}

float getYaw() {
  return atan2(2 * (q0*q3 + q1*q2), 1 - 2 * (q2*q2 + q3*q3)) * 180 / M_PI;
}
