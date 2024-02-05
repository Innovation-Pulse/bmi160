/*
 * Copyright (c) 2016 Intel Corporation.  All rights reserved.
 * See the bottom of this file for the license terms.
 */

/*
   This sketch example demonstrates how the BMI160 on the
   Intel(R) Curie(TM) module can be used to read gyroscope data
*/
      
#include <BMI160Gen.h>
#include <SPI.h>

void setup() {
  Serial.begin(9600); // initialize Serial communication
  while (!Serial);    // wait for the serial port to open
  SPI.begin();        // 启动 SPI 总线
  pinMode(10, OUTPUT); // 设置片选引脚为输出模式
  pinMode(9, OUTPUT); // 设置片选引脚为输出模式
  digitalWrite(10, HIGH); // 禁用第一个从机
  digitalWrite(9, HIGH); // 禁用第二个从机

  // initialize device
  Serial.println("Initializing IMU device...");
  BMI160.begin(BMI160GenClass::SPI_MODE, /* SS pin# = */10);
  BMI160.begin(BMI160GenClass::SPI_MODE, /* SS pin# = */9);

  //BMI160.begin(BMI160GenClass::I2C_MODE);
  uint8_t dev_id = BMI160.getDeviceID();
  Serial.print("DEVICE ID: ");
  Serial.println(dev_id, HEX);

   // Set the accelerometer range to 250 degrees/second
  BMI160.setGyroRange(250);
  Serial.println("Initializing IMU device...done.");
}

void loop() {

  int gx1Raw, gy1Raw, gz1Raw,gx2Raw, gy2Raw, gz2Raw;         // raw gyro values
  float gx1, gy1, gz1, gx2, gy2, gz2;

  // read raw gyro measurements from device

  // 选中第一个从机
  digitalWrite(10, LOW);
  delay(1); // 等待芯片响应时间，具体时间可能需要根据芯片手册来确定
  // 读取第一个传感器的陀螺仪数据
  BMI160.readGyro(gx1Raw, gy1Raw, gz1Raw);
  //解析数据
  gx1 = convertRawGyro(gx1Raw);
  gy1 = convertRawGyro(gy1Raw);
  gz1 = convertRawGyro(gz1Raw);
  //打印结果
  Serial.print(gx1);
  Serial.print("\t");
  Serial.print(gy1);
  Serial.print("\t");
  Serial.print(gz1);
  Serial.print("\t");
  // 取消选中第一个从机
  digitalWrite(10, HIGH);
  // 等待一小段时间以确保数据稳定
  delay(10);

  // 选中第二个从机
  digitalWrite(9, LOW);
  delay(1); // 等待芯片响应时间
  // 读取第二个传感器的陀螺仪数据
  BMI160.readGyro(gx2Raw, gy2Raw, gz2Raw);  
  //解析数据
  gx2 = convertRawGyro(gx2Raw);
  gy2 = convertRawGyro(gy2Raw);
  gz2 = convertRawGyro(gz2Raw);
  //打印结果  
  Serial.print(gx2);
  Serial.print("\t");
  Serial.print(gy2);
  Serial.print("\t");
  Serial.print(gz2);
  Serial.println();
  // 取消选中第二个从机
  digitalWrite(9, HIGH);
  // convert the raw gyro data to degrees/second
  // display tab-separated gyro x/y/z values
  delay(100);

  
}

float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767
  //float g = gRaw;
  float g = (gRaw * 250.0) / 32768.0;

  return g;
}

/*
   Copyright (c) 2016 Intel Corporation.  All rights reserved.
   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
  
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.
   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.
   You should have received a copy of the GNU Lesser General Public
   License along with this library; if not, write to the Free Software
   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/
