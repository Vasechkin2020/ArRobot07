#ifndef CODE_H
#define CODE_H

//Функция возращает максимальный размер из 2 структур
uint16_t getMax_size_Struct(uint16_t stru1_, uint16_t stru2_)
{
	uint16_t ret = 0;

	if (stru1_ > stru2_)
	{
		ret = stru1_;
	}
	else
	{
		ret = stru2_;
	}
	return ret += 1; // + 1 байт Для контрольной суммы
}

//Функция возвращает контрольную сумму структуры без последних 4 байтов
template <typename T>
uint32_t measureCheksum(const T &structura_)
{
	uint32_t ret = 0;
	unsigned char *adr_structura = (unsigned char *)(&structura_); // Запоминаем адрес начала структуры. Используем для побайтной передачи
	for (int i = 0; i < sizeof(structura_) - 4; i++)
	{
		ret += adr_structura[i]; // Побайтно складываем все байты структуры кроме последних 4 в которых переменная в которую запишем результат
	}
	return ret;
}

//Настройка светодиодов
void set_PIN_Led()
{
	pinMode(PIN_LED_RED, OUTPUT);	// Зеленый светодиод
	pinMode(PIN_LED_BLUE, OUTPUT);	// Красный светодиод
	pinMode(PIN_LED_GREEN, OUTPUT); // Красный светодиод
	digitalWrite(PIN_LED_RED, 1);
	digitalWrite(PIN_LED_BLUE, 1);
	digitalWrite(PIN_LED_GREEN, 1);
	delay(1000);
	digitalWrite(PIN_LED_RED, 0);
	digitalWrite(PIN_LED_BLUE, 0);
	digitalWrite(PIN_LED_GREEN, 0);
}

//Функция мигания светодиодом в осномном цикле что программа не зависла и работает
void Led_Blink(int led_, unsigned long time_)
{
	static unsigned long led_time = 0;
	static bool led_status = 0;
	if ((millis() - led_time) > time_)
	{
		led_status = 1 - led_status;
		digitalWrite(led_, led_status);
		led_time = millis();
	}
}

//Обработка полученных данных и копирование их куда надо
void dataProcessing_Body()
{
	//Копируем полученные по SPI данные в сообщение которое потом опубликуем
	msg_body_send.id = Body.id;

	msg_body_send.odom_enc.x = Body.odom_enc.x;
	msg_body_send.odom_enc.y = Body.odom_enc.y;
	msg_body_send.odom_enc.th = Body.odom_enc.th;
	msg_body_send.odom_enc.vel_x = Body.odom_enc.vel_x;
	msg_body_send.odom_enc.vel_y = Body.odom_enc.vel_y;
	msg_body_send.odom_enc.vel_th = Body.odom_enc.vel_th;

	msg_body_send.odom_imu.x = Body.odom_imu.x;
	msg_body_send.odom_imu.y = Body.odom_imu.y;
	msg_body_send.odom_imu.th = Body.odom_imu.th;
	msg_body_send.odom_imu.vel_x = Body.odom_imu.vel_x;
	msg_body_send.odom_imu.vel_y = Body.odom_imu.vel_y;
	msg_body_send.odom_imu.vel_th = Body.odom_imu.vel_th;

	// Данные по лазерам
	//Данные по одометрии принимаем как есть
	msg_body_send.odom_L = Body.odom_L;
	msg_body_send.odom_R = Body.odom_R;

	msg_body_send.speed_L = Body.speed_L;
	msg_body_send.speed_R = Body.speed_R;

	msg_body_send.distance_lazer_L = Body.distance_lazer_L;
	msg_body_send.distance_lazer_R = Body.distance_lazer_R;
	msg_body_send.distance_uzi = Body.distance_uzi;

	//Данные по датчику BME280 принимаем как есть
	msg_body_send.bme.humidity = Body.bme.humidity;
	msg_body_send.bme.pressure = Body.bme.pressure;
	msg_body_send.bme.temperature = Body.bme.temperature;
	msg_body_send.bme.loc = Body.bme.loc;
	//Данные по датчику BNO055 принимаем как есть
	msg_body_send.bno055.roll = Body.bno055.roll;
	msg_body_send.bno055.pitch = Body.bno055.pitch;
	msg_body_send.bno055.yaw = Body.bno055.yaw;
	//Данные по датчикам газа
	msg_body_send.gaz_data = Body.gaz_data;

	msg_body_send.obmen_all = data_body_all;
	msg_body_send.obmen_bed = data_body_bed;
}

//Функция записи в нужные места данных одометрии в tf и в odom
void setOdomToTf(ros::NodeHandle nh_, tf::TransformBroadcaster odom_broadcaster_,ros::Time current_time_ )
{
	//Пример кода взят отсюда
	//http://library.isr.ist.utl.pt/docs/roswiki/navigation(2f)Tutorials(2f)RobotSetup(2f)Odom.html

	// current_time_ = ros::Time::now(); // Получаем текущее время в ROS

	// //since all odometry is 6DOF we'll need a quaternion created from yaw
	// // получаем из моего направления куда смотрит робот кватернион
	// geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(Body.odom_enc.th);
	// //first, we'll publish the transform over tf
	// geometry_msgs::TransformStamped odom_trans;
	// odom_trans.header.stamp = current_time_;
	// odom_trans.header.frame_id = "odom";
	// odom_trans.child_frame_id = "base_link";

	// odom_trans.transform.translation.x = Body.odom_enc.x;
	// odom_trans.transform.translation.y = Body.odom_enc.x;
	// odom_trans.transform.translation.z = 0.0;
	// odom_trans.transform.rotation = odom_quat;

	// //send the transform
	// odom_broadcaster_.sendTransform(odom_trans);

	// //next, we'll publish the odometry message over ROS

	// odom.header.stamp = current_time_;
	// odom.header.frame_id = "odom";

	// //set the position
	// odom.pose.pose.position.x = Body.odom_enc.x;
	// odom.pose.pose.position.y = Body.odom_enc.y;
	// odom.pose.pose.position.z = 0.0;
	// odom.pose.pose.orientation = odom_quat;

	// //set the velocity
	// odom.child_frame_id = "base_link";
	// odom.twist.twist.linear.x = Body.odom_enc.vel_x;
	// odom.twist.twist.linear.y = Body.odom_enc.vel_y;
	// odom.twist.twist.angular.z = Body.odom_enc.vel_th;


}

//Обработка полученных данных и копирование их для публикации в топике
void dataProcessing_Control()
{
	msg_control_send.id = Control.id;

	msg_control_send.radius = Control.radius;
	msg_control_send.startStop = Control.startStop;
	msg_control_send.connect_flag = Control.connect_flag;

	msg_control_send.led_program = Control.led_program;
	msg_control_send.camera = Control.camera;
	msg_control_send.crwl0516 = Control.crwl0516;

	msg_control_send.ssid_wifi = Control.ssid_wifi;
	msg_control_send.password_wifi = Control.password_wifi;

	msg_control_send.obmen_all = data_control_all;
	msg_control_send.obmen_bed = data_control_bed;
}

//Копирование данных из сообщения в топике в структуру для передачи по SPI
void Collect_Data2Body() // Данные для передачи на низкий уровень
{
	Data2Body.id++; //= 0x1F1F1F1F;
	Data2Body.command_body = msg_head_receive.command_body;
	Data2Body.radius = msg_head_receive.radius;
	Data2Body.speed = msg_head_receive.speed;
	Data2Body.motor_video_angle = msg_head_receive.motor_video_angle;
	//тут нужнопосчитать контрольную смму структуры
	Data2Body.cheksum = measureCheksum(Data2Body); // Считаем контрольную сумму отправляемой структуры
												   //printf("Отправляем: Id %i, чек= %i  ", Data2Body.id, Data2Body.cheksum);
}
//Копирование рабочих данных в структуру для передачи
void Collect_Data2Control() // Данные для передачи на низкий уровень
{
	// uint8_t *adr_DataHL_Control = (uint8_t *)(&DataHL_Control);		// Запоминаем адрес начала структуры.
	// memset(adr_DataHL_Control, 0, sizeof(DataHL_Control));             // очистка блока памяти
	Data2Control.id++; //= 0x1F1F1F1F;
	Data2Control.odom_L = Body.odom_L;
	Data2Control.odom_R = Body.odom_R;
	Data2Control.speed_L = Body.speed_L;
	Data2Control.speed_R = Body.speed_R;
	Data2Control.distance_lazer_L = Body.distance_lazer_L;
	Data2Control.distance_lazer_R = Body.distance_lazer_R;
	Data2Control.distance_uzi = Body.distance_uzi;
	Data2Control.bno055.roll = Body.bno055.roll;
	Data2Control.bno055.pitch = Body.bno055.pitch;
	Data2Control.bno055.yaw = Body.bno055.yaw;
	Data2Control.bme.temperature = Body.bme.temperature;
	Data2Control.bme.pressure = Body.bme.pressure;
	Data2Control.bme.humidity = Body.bme.humidity;
	Data2Control.bme.loc = Body.bme.loc;
	Data2Control.gaz_data = Body.gaz_data;
	// То что передаем из топика головы
	Data2Control.command_body = msg_head_receive.command_body;
	Data2Control.radius = msg_head_receive.radius;
	Data2Control.speed = msg_head_receive.speed;
	Data2Control.motor_video_angle = msg_head_receive.motor_video_angle;
	Data2Control.program_led = msg_head_receive.program_led;

	//тут нужнопосчитать контрольную смму структуры
	Data2Control.cheksum = measureCheksum(Data2Control); // Считаем контрольную сумму отправляемой структуры
														 //printf("Отправляем: Id %i, чек= %i  ", Data2Control.id, Data2Control.cheksum);
}

// Выводим на экран данные которые отправляем в Control
void printData_To_Control()
{
	printf(" Data2Control id = %i", Data2Control.id);
	printf(" distance_uzi = %f", Data2Control.distance_uzi);
}
// Выводим на экран данные команды которую получили
void printData_To_Body()
{
	//printf(" SEND id = %i", stru_body_send.id);
	// printf(" command = %i", stru_body_send.command);
	//printf(" napravlenie = %i", msg_head_receive.napravlenie);
	//printf(" radius = %f", stru_body_send.radius);
	// printf(" speed = %f", stru_body_send.speed);
	// printf(" angle_cam = %f", stru_body_send.cam_angle);
	//printf(" ventil_speed = %i", stru_body_send.ventil_speed);
	//printf("  /  ");
}
// Выводим на экран данные которые получили
void printData_From_Body()
{
	printf(" RECEIVE id = %i", Body.id);
	printf(" gaz.x = %f", Body.gaz_data);
	//printf(" gaz2.x = %f", stru_body_receive.gaz2_data);
	// printf(" bno055.x = %f", stru_body_receive.bno055.x);
	// printf(" bno055.y = %f", stru_body_receive.bno055.y);
	// printf(" bno055.z = %f", stru_body_receive.bno055.z);
	//printf(" temperature = %f", stru_body_receive.bmp280.temperature);
	//printf(" pressure = %f", stru_body_receive.bmp280.pressure);
	//printf(" humidity = %f", stru_body_receive.bmp280.humidity);
	// printf(" distance_lazer = %.2f", stru_body_receive.distance_lazer);
	//	printf(" a1 = %.2f", a1);
	// printf(" a2 = %.2f", a2);
	// printf(" distance_uzi = %.2f", stru_body_receive.distance_uzi);
	//	printf(" u1 = %.2f", u1);
	// printf(" u2 = %.2f", u2);
	// printf(" X_comp = %.2f", msg_body_send.mpu9250.x);
	// printf(" Y_comp = %.2f", msg_body_send.mpu9250.y);
	// printf(" Z_comp = %.2f", msg_body_send.mpu9250.z);
	printf("  / data_body_all= %i data_body_bed= %i /", data_body_all, data_body_bed);
	printf("\n");
}

//Инициализация канала шины SPI
void init_SPI(int channel_, int speed_)
{
	uint8_t errSpi = 0; //Ошибка при инициализации шины SPI
	ROS_INFO("%s Init SPI start... ",NN);

	if ((errSpi = wiringPiSPISetup(channel_, speed_)) < 0) //Инициализация канало 0 это чип селект 0
	{
		ROS_ERROR ("%s Can't open the SPI bus 0: %s\n",NN, strerror(errno));
		ROS_ERROR ("%s errSpi: %s\n", NN, errSpi);
		exit(EXIT_FAILURE);
	}
	else
	{
		ROS_INFO("%s SPI ok! \n",NN);
	}
}

// Обратный вызов при опросе топика Control
void message_callback_Command(const my_msgs::Command &msg)
{
	msg_head_receive = msg; // Копируем структуру в глобальную переменную для дальнейшей работы с ней.
							// ROS_INFO("message_callback_Command.");
}

// Выводим на экран данные которые получили
void printDataFrom_Control()
{
	printf(" Получили id = %i", Control.id);
	printf(" radius = %.2f", Control.radius);
	printf(" cheksum = %i", Control.cheksum);
	printf("\n");
}


//Основная функция приема-передачи двух структур на slave контроллер по протоколу SPI
bool sendData2Control(int channel_, Struct_Control &structura_receive_, Struct_Data2Control &structura_send_) // Указываем на каком пине устройство и с какого регистра нужно прочитать данные
{
	uint8_t rez = false;												// Результат выполнения функции
	const uint16_t size_structura_receive = sizeof(structura_receive_); // Размер структуры с данными которые получаем
	const uint16_t size_structura_send = sizeof(structura_send_);		// Размер структуры с данными которые передаем
	uint8_t *adr_structura_send = (uint8_t *)(&structura_send_);		// Запоминаем адрес начала структуры. Используем для побайтной передачи

	//uint16_t max_size_stuct = getMax_size_Struct(size_structura_receive, size_structura_send); // Какая из структур больше
	const uint16_t max_size_stuct = (size_structura_receive < size_structura_send) ? size_structura_send : size_structura_receive; // Какая из структур больше
	memset(buffer, 0, sizeof(buffer));																							   // Очищаем буфер передачи

	//Заполняем буфер данными структуры для передачи
	Struct_Data2Control *buffer_send = (Struct_Data2Control *)buffer; // Создаем переменную в которую записываем адрес буфера в нужном формате
	*buffer_send = structura_send_;									  // Переписываем по этому адресу данные в буфер

	// for (int i = 0; i < 16; i++)
	// {
	// 	printf("%x ",adr_structura_send[i]);
	// }
	// 	printf("\n");

	// for (int i = 0; i < 16; i++)
	// {
	// 	printf("%x ",buffer[i]);
	// }
	// 	printf("\n");

	//data_control_all++;
	// int aa = micros();
	rez = wiringPiSPIDataRW(channel_, buffer, sizeof(buffer)); //Передаем и одновременно получаем данные
	// int time_transfer = micros() - aa;
	// float time_transfer_sec = time_transfer / 1000000.0;

	// Serial.print(String(micros()) + " Delta time= " + (ed - st) + " Size= " + Size_structura_send + " ");
	// float time_sec = (ed - st) / 1000000.0; // Перводим в секунды
	// Serial.println(" Speed= " + String((Size_structura_send / 1024.0) / time_sec) + "Kbyte/sec");

	// printf("---\n");
	// for (int i = 0; i < 40; i++)
	// {
	// 	printf("%x ",buffer[i]);
	// }
	// 	printf("\n");

	//Извлекаем из буфера данные в формате структуры и копирум данные
	Struct_Control *copy_buf_master_receive = (Struct_Control *)buffer; // Создаем переменную в которую пишем адрес буфера в нужном формате
	structura_receive_ = *copy_buf_master_receive;						// Копируем из этой перемнной данные в мою структуру
	uint32_t cheksum_receive = measureCheksum(structura_receive_);		// Считаем контрольную сумму пришедшей структуры

	//Struct_Control copy_Control_receive = *copy_buf_master_receive;		// Копируем из этой перемнной данные в мою структуру
	//uint8_t *adr_copy_Control_receive = (uint8_t *)(&copy_Control_receive); // Запоминаем адрес начала структуры. Используем для побайтной передачи
	// for (int i = 0; i < 20; i++)
	// {
	// 	printf("%x ",adr_copy_Control_receive[i]);
	// }
	// 	printf("\n");

	// uint32_t cheksum_buf = measureCheksum(buffer);				   // Считаем контрольную сумму пришедшей структуры

	// printf(" Получили: Id %i, cheksum %i", structura_receive_.id, structura_receive_.cheksum);
	// printf(" capacity_real %f, capacity_percent %f", structura_receive_.capacity_real, structura_receive_.capacity_percent);
	// printf(" inVoltage %f, current_mA %f", structura_receive_.INA219.inVoltage, structura_receive_.INA219.current_mA);
	// printf(" radius %f, startStop %i", structura_receive_.radius, structura_receive_.startStop);
	//printf(" чексумма %i, myChek= %i, buf_chek= %i", structura_receive_.cheksum, cheksum_receive, cheksum_buf);

	//printf(" ok= %i, bed= %i, time= %i, speed Kbyte/sec= %f \n", ok, bed, time_transfer, (sizeof(buffer)/1024.0) / time_transfer_sec);
	//printf(" /  ok= %i, bed= %i  / ", data_control_all, data_control_bed);

	//Serial.println(String(micros()) + " copy_DataHL_Control_receive id= " + copy_DataHL_Control_receive.id);
	//Serial.println(String(micros()) + " copy_DataHL_Control_receive bmp280.pressure= " + copy_DataHL_Control_receive.bmp280.pressure);
	// Serial.println(String(micros()) + " copy_DataHL_Control_receive.cheksum_receive= " + copy_DataHL_Control_receive.cheksum);
	//printf(" measureCheksum= %i \n", cheksum_receive);
	if (cheksum_receive != structura_receive_.cheksum || structura_receive_.cheksum == 0) // Если наша чек сумма совпадает с последним байтом где чексума переданных данных
	{
		data_control_bed++;
		return false;
	}
	else // Все хорошо возвращаем Ок
	{
		return true;
	}
}

//Основная функция приема-передачи двух структур на slave контроллер по протоколу SPI
bool sendData2Body(int channel_, Struct_Body &structura_receive_, Struct_Data2Body &structura_send_) // Указываем на каком пине устройство и с какого регистра нужно прочитать данные
{
	uint8_t rez = false;												// Результат выполнения функции
	const uint16_t size_structura_receive = sizeof(structura_receive_); // Размер структуры с данными которые получаем
	const uint16_t size_structura_send = sizeof(structura_send_);		// Размер структуры с данными которые передаем
	uint8_t *adr_structura_send = (uint8_t *)(&structura_send_);		// Запоминаем адрес начала структуры. Используем для побайтной передачи

	const uint16_t max_size_stuct = (size_structura_receive < size_structura_send) ? size_structura_send : size_structura_receive; // Какая из структур больше
	memset(buffer, 0, sizeof(buffer));																							   // Очищаем буфер передачи

	//Заполняем буфер данными структуры для передачи
	Struct_Data2Body *buffer_send = (Struct_Data2Body *)buffer; // Создаем переменную в которую записываем адрес буфера в нужном формате
	*buffer_send = structura_send_;								// Переписываем по этому адресу данные в буфер

	rez = wiringPiSPIDataRW(channel_, buffer, sizeof(buffer)); //Передаем и одновременно получаем данные

	//Извлекаем из буфера данные в формате структуры и копирум данные
	Struct_Body *copy_buf_master_receive = (Struct_Body *)buffer;  // Создаем переменную в которую пишем адрес буфера в нужном формате
	structura_receive_ = *copy_buf_master_receive;				   // Копируем из этой перемнной данные в мою структуру
	uint32_t cheksum_receive = measureCheksum(structura_receive_); // Считаем контрольную сумму пришедшей структуры

	if (cheksum_receive != structura_receive_.cheksum || structura_receive_.cheksum == 0) // Если наша чек сумма совпадает с последним байтом где чексума переданных данных
	{
		return false;
	}
	else // Все хорошо возвращаем Ок
	{
		return true;
	}
}
#endif