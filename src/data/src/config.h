#ifndef CONFIG_H
#define CONFIG_H

#define RATE 2 // Частота обмена с нижним уровнев в Герц
//---------------------------------------------------------------------------------------
#define PIN_LED_BLUE 28
#define PIN_LED_RED 29
#define PIN_LED_GREEN 25
//---------------------------------------------------------------------------------------
#define SPI_CHANNAL_0 0   //Какой из двух каналов инициализируем
#define SPI_CHANNAL_1 1   //Какой из двух каналов инициализируем
#define SPI_SPEED 4000000 // Скорость работы шины SPI
//#define PIN_SPI_LINE 2 // Пин обратной связи с ведомым
//#define PIN_SPI_LINE 3 // Пин обратной связи с ведомым
//---------------------------------------------------------------------------------------

#define SIZE_BUFF 160         // Размер буфера, стараться делать кратно 32
unsigned char buffer[SIZE_BUFF]; //Буфер в 1 байт в который пишем передаваемый байт и в котором оказывется принятый байт
//---------------------------------------------------------------------------------------
#define DEVICE_ID_STM 0x1A         // Адрес I2C платы STM как slave устройства


bool rez_data = false;

int data_body_all = 0;
int data_body_bed = 0;
int data_control_all = 0;
int data_control_bed = 0;

float a1, a2;
float u1, u2;
MyKalman laser;
MyKalman uzi;
MyKalman mpu9250_x;
MyKalman mpu9250_y;

//*********************************************************************
//Структура для углов наклонов
struct Struct_XYZ
{
  float roll = 0;
  float pitch = 0;
  float yaw = 0;
};
//Структура для температурного датчика BMP280
struct Struct_BME
{
  float temperature = 0;
  float pressure = 0;
  float humidity = 0;
  float loc = 0;
};
//Структура для датчика напряжения INA219
struct Struct_INA
{
  float voltage = 0;
  float current = 0;
  float capacity_percent = 0;
  float capacity_real = 0;
};

struct Struct_Data2Control // Структура передаваемых данных
{
  uint32_t id = 0;            // Id команды
  float odom_L = 0;           // Пройденный телом путь
  float odom_R = 0;           // Пройденный телом путь
  float speed_L = 0;          // Скорость левого колеса
  float speed_R = 0;          // Скорость правого колеса
  float distance_lazer_L = 0; // Данные лазерного датчика
  float distance_lazer_R = 0; // Данные лазерного датчика
  float distance_uzi = 0;     // Данные ультразвукового датчика
  Struct_XYZ bno055;          // Данные с датчика BNO055
  Struct_BME bme;             // Данные с датчика BME
  float gaz_data = 0;         // Данные с первого датчика газа
  Struct_INA ina;             // Данные с датчика INA219
  int32_t command_body = 0;    // Команда для выполнения
  float radius = 0;            // Радиус по которому нужно двигаться
  float speed = 0;             // Скорость которую нужно установить
  float motor_video_angle = 0; // Теоретическое положение шагового мотора который поворачивает камеру
  uint32_t program_led = 0;    // Номер программы которая выполняется на светодиодах

  uint32_t cheksum = 0; // Контрольная сумма данных в структуре
};

//Структура в которой все главные переменные которые передаюся на высокий уровень
struct Struct_Control
{
  uint32_t id = 0;            // Id пакета
  int32_t radius = 0;           // =0..100 положение слайдера
  uint32_t startStop = 0;     // =0 если переключатель в положении A, =1 если в положении B, =2 если в положении C, ...
  uint32_t connect_flag = 0;  // =1 if wire connected, else =0
  uint32_t led_program = 0;   // Номер программы которая выполняется на светодиодах
  uint32_t camera = 0;        // положение видеокамеры 
  uint32_t crwl0516 = 0;      // Данные с датчика движения
  char ssid_wifi[16]{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  char password_wifi[16]{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  uint32_t cheksum = 0;       // Контрольная сумма данных в структуре
};

//Структура для углов наклонов
struct Struct_RPY
{
  float roll = 0;  // Крен в право  влево
  float pitch = 0; // Тангаж вверх или вних
  float yaw = 0;   // Поворот по часовой мом против часовой

  Struct_RPY &operator=(const Struct_RPY &source) // Специальный оператор, как функция в структуре, позволяет копировать одинаковые структуры просто знаком равно
  {
    roll = source.roll;
    pitch = source.pitch;
    yaw = source.yaw;
    return *this;
  }
};

//Структура одометрии
struct Struct_Odom
{
  float x = 0;      // Координата по Х
  float y = 0;      // Координата по Y
  float th = 0;     // Направление носа
  float vel_x = 0;  // Линейная скорость движения робота по оси X
  float vel_y = 0;  // Линейная скорость движения робота по оси Y 
  float vel_th = 0; // Угловая скорость вращения робота
  
  Struct_Odom &operator=(const Struct_Odom &source) // Специальный оператор, как функция в структуре, позволяет копировать одинаковые структуры просто знаком равно
  {
    x = source.x;
    y = source.y;
    th = source.th;
    vel_x = source.vel_x;
    vel_y = source.vel_y;
    vel_th = source.vel_th;
    return *this;
  }
};



//Структура в которой все главные переменные которые передаюся на высокий уровень

struct Struct_Body
{
  uint32_t id = 0;            // id команды
  Struct_Odom odom_enc;       // Одометрия по энкодерам
  Struct_Odom odom_imu;       // Одометрия по гироскопу и аксельрометру
  float odom_L = 0;           // Пройденный путь левым колесом
  float odom_R = 0;           // Пройденный путь правым колесом
  float speed_L = 0;          // Скорость левого колеса
  float speed_R = 0;          // Скорость правого колеса
  float distance_lazer_L = 0; // Данные лазерного датчика
  float distance_lazer_R = 0; // Данные лазерного датчика
  float distance_uzi = 0;     // Данные ультразвукового датчика
  Struct_RPY bno055;          // Данные с датчика BNO055
  Struct_BME bme;             // Данные с датчика BME
  Struct_INA ina;             // Данные с датчика INA219
  float gaz_data = 0;         // Данные с первого датчика газа
  float lux = 0;              // Данные с дачика освещенности
  uint32_t cheksum = 0;       // Контрольная сумма данных в структуре
};

// Структура отправляемых данных
struct Struct_Data2Body
{
  uint32_t id = 0;         // Номер команды по порядку
  int32_t command_body = 0;     // Команда для выполнения
  float radius = 0;        // Радиус по которому нужно двигаться
  float speed = 0;         // Скорость которую нужно установить
  float motor_video_angle; // Угол для шагового мотора камеры
  uint32_t cheksum = 0;    // Контрольная сумма данных в структуре
};

//========================================================
Struct_Body Body;           //Тело робота. тут все переменные его характеризующие на низком уровне
Struct_Data2Body Data2Body; // Экземпляр структуры отправлемых данных

//========================================================
Struct_Control Control;           //
Struct_Data2Control Data2Control; // Экземпляр структуры отправлемых данных

const uint32_t size_stucturs = sizeof(Struct_Body);

//********************************** Вывод на печать отладочной информации

//#define ROSCONSOLE_MIN_SEVERITY ROSCONSOLE_SEVERITY_WARN  // Если поставить WARN и раскомментировать то не будут выводиться сооющения ROS уровня INFO
//#define LEVEL_SEVERITY YES              // Если раскомментировать то мои метки не будут выводиться

#define RED "\x1b[31;40m"
#define GREEN "\x1b[32;40m"
#define YELLOW "\x1b[33;40m"
#define BLUE "\x1b[34;40m"
#define MAGENTA "\x1b[35;40m"
#define CYAN "\x1b[36;40m"
#define NORM "\x1b[0m"
#define NN "\x1b[33;40m Data_node"

void my_printInfo()
{
#ifndef LEVEL_SEVERITY
    printf("%s %s [ INFO]", NORM, NN);
#endif
}
void my_printWarn()
{
#ifndef LEVEL_SEVERITY
    fprintf(stderr, "%s %s [ WARN]", YELLOW, NN);
#endif
}
void my_printErr()
{
#ifndef LEVEL_SEVERITY
    fprintf(stderr, "%s %s [ ERR ]", RED, NN);
#endif
}

#define INFO my_printInfo();
#define WARN my_printWarn();
#define ERROR my_printErr();

#endif