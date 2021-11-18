#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_broadcaster.h>

#include <nav_msgs/Odometry.h>
#include <my_msgs/Body.h>
#include <my_msgs/Command.h>
#include <my_msgs/Control.h>

#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <wiringPiI2C.h>

nav_msgs::Odometry odom;
my_msgs::Command msg_head_receive; // Полученное сообщение из топика
my_msgs::Body msg_body_send;       // Сообщение которое публикуем в Топик
my_msgs::Control msg_control_send; // Сообщение которое публикуем в Топик

#include "MyKalman.h"
#include "config.h"
#include "code.h"
#include "MyClass.h"

int main(int argc, char **argv)
{

    ROS_INFO("%s -------------------------------------------------", NN);
    ROS_WARN("%s START Data Module HighLevel  Raspberry Pi 4B !!! ", NN);
    ROS_ERROR("%s -------------------------------------------------", NN);

    //MyClass myClass; // Лбьявляем свою локальную перемнную класса и дальше работаем внутри этого класса

    ros::init(argc, argv, "data_node");
    ros::NodeHandle nh;
    ros::Publisher str_pub_body = nh.advertise<my_msgs::Body>("body_topic", 16);               //Это мы публикуем структуру
    ros::Publisher str_pub_control = nh.advertise<my_msgs::Control>("control_topic", 16);      //Это мы публикуем структуру
    ros::Subscriber command_sub = nh.subscribe("command_topic", 16, message_callback_Command); // Это мы подписываемся на то что публигует Главная нода для Body
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster odom_broadcaster;
    ros::Time current_time; // Время ROS

    ros::Rate r(RATE);

    wiringPiSetup();                    // Инициализация библиотеки
    set_PIN_Led();                      // Устанавливаем и обьявляем пины. для вывода анализатора светодиодов и всего прочего
    init_SPI(SPI_CHANNAL_0, SPI_SPEED); // Инициализация нужного канала SPI
    init_SPI(SPI_CHANNAL_1, SPI_SPEED); // Инициализация нужного канала SPI

        // Setup I2C communication
        int fd = wiringPiI2CSetup(0x68);
        if (fd == -1)
        {
            ROS_WARN("%s Failed to init I2C communication = %i. \n", NN, fd);
            return -1;
        }
        ROS_INFO("%s I2C communication successfully setup = %i. \n", NN, fd);
        
    // laser.setParametr1 (1, 0.1);
    laser.setParametr2(5, 0.1); // Установка параетров фильтрации в фидьтре Каламан. Можно подбирать как получится
    // uzi.setParametr1 (0.5, 0.1);
    uzi.setParametr2(5, 0.1);

    while (ros::ok())
    {
        Led_Blink(PIN_LED_GREEN, 1000); // Мигание светодиодом, что цикл работает
        ros::spinOnce();                // Обновление в данных в ядре ROS, по этой команде происходит вызов функции обратного вызова

        //----------------------------------------------------------------------------------------------------------
        //wiringPiI2CWriteReg8(fd, REG_POWER_CTL, 0b00001000);
        int a = wiringPiI2CWriteReg8(fd, 00, 111);
        ROS_INFO(" %s I2C = %i. ", NN, a);
        int b = wiringPiI2CWrite(fd, 33);
        ROS_INFO(" %s I2C = %i. ", NN, b);
        

        // //----------------------------------------------------------------------------------------------------------
        // Collect_Data2Control(); //Собираем рабочие данные в структуру для передачи считывая из топиков
        // //printData_To_Control();                                            // Выводим на печать то что отправляем в Control
        // //rez_data = sendData2Control(SPI_CHANNAL_0, Control, Data2Control); //
        // data_control_all++;
        // //printDataFrom_Control();
        // if (rez_data) // Если пришли хорошие данные то обрабатываем их и публикуем данные в ROS
        // {
        //     digitalWrite(PIN_LED_RED, 0); // Гасим светодиод пришли хорошие данные
        //     //ROS_INFO("Data ok! ");
        //     dataProcessing_Control();                  // Обрабатываем данные
        //     str_pub_control.publish(msg_control_send); //Публикация полученных данных
        // }
        // if (!rez_data) // Если пришли плохие данные то выводим ошибку
        // {
        //     digitalWrite(PIN_LED_RED, 1); // Включаем светодиод пришли плохие данныеc

        //     ROS_WARN("%s Flag_bedData chek_sum BED Control", NN);
        // }

        // ROS_INFO("%s 0 channal data_control_all= %i, data_control_bed= %i", NN, data_control_all, data_control_bed);
        // //----------------------------------------------------------------------------------------------------------

        // Collect_Data2Body(); //Собираем рабочие данные в структуру для передачи считывая данные из топика ноды Head
        // //printData_To_Body();
        // //rez_data = sendData2Body(SPI_CHANNAL_1, Body, Data2Body); ////  Отправляем данные на нижний уровень
        // data_body_all++;
        // //printDataFrom_Body();

        // if (rez_data) // Если пришли хорошие данные то обрабатываем их и публикуем данные в ROS
        // {
        //     digitalWrite(PIN_LED_BLUE, 0); // Гасим светодиод пришли хорошие данные
        //     //ROS_INFO("Data ok! ");
        //     dataProcessing_Body(); // Обрабатываем данные
        //                            //setOdomToTf(nh, odom_broadcaster, current_time); // Функция которая одометрию пищет куда нужно, передаем этой функции все переменные котороые создали в гласной функции main

        //     current_time = ros::Time::now(); // Получаем текущее время в ROS

        //     //since all odometry is 6DOF we'll need a quaternion created from yaw
        //     // получаем из моего направления куда смотрит робот кватернион
        //     geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(Body.odom_enc.th);
        //     //first, we'll publish the transform over tf
        //     geometry_msgs::TransformStamped odom_trans;
        //     odom_trans.header.stamp = current_time;
        //     odom_trans.header.frame_id = "odom";
        //     odom_trans.child_frame_id = "base_link";

        //     odom_trans.transform.translation.x = Body.odom_enc.x;
        //     odom_trans.transform.translation.y = Body.odom_enc.y;
        //     odom_trans.transform.translation.z = 0.0;
        //     odom_trans.transform.rotation = odom_quat;

        //     //send the transform
        //     odom_broadcaster.sendTransform(odom_trans);

        //     //next, we'll publish the odometry message over ROS

        //     odom.header.stamp = current_time;
        //     odom.header.frame_id = "odom";

        //     //set the position
        //     odom.pose.pose.position.x = Body.odom_enc.x;
        //     odom.pose.pose.position.y = Body.odom_enc.y;
        //     odom.pose.pose.position.z = 0.0;
        //     odom.pose.pose.orientation = odom_quat;

        //     //set the velocity
        //     odom.child_frame_id = "base_link";
        //     odom.twist.twist.linear.x = Body.odom_enc.vel_x;
        //     odom.twist.twist.linear.y = Body.odom_enc.vel_y;
        //     odom.twist.twist.angular.z = Body.odom_enc.vel_th;

        //     odom_pub.publish(odom); //publish the message

        //     str_pub_body.publish(msg_body_send); //Публикация полученных данных
        // }

        // if (!rez_data) // Если пришли плохие данные то выводим ошибку
        // {
        //     data_body_bed++;
        //     digitalWrite(PIN_LED_BLUE, 1); // Включаем светодиод пришли плохие данные
        //     ROS_WARN("%s Flag_bedData chek_sum BED Body", NN);
        // }
        // ROS_INFO("%s 1 channal data_body_all   = %i, data_body_bed   = %i", NN, data_body_all, data_body_bed);

        ros::spinOnce(); // Обновление в данных в ядре ROS
        r.sleep();       // Интеллектуальная задержка на указанную частоту
    }
    return 0;
}
