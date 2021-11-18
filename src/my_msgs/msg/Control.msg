
# Структура топика с моими данными получаемыми с нижнего уровня Control
uint32 id                   #  Id команды
float32 radius				#	=0..100 положение слайдера
uint32 startStop			#	=0 если переключатель в положении A, =1 если в положении B, =2 если в положении C, ...
uint32 connect_flag			#	=1 if wire connected, else =0
float32 camera              # положение видеокамеры угол в градусах
uint32 led_program          # Номер программы которая выполняется на светодиодах
uint32 crwl0516             # Данные с датчика движения
string ssid_wifi
string password_wifi
uint32 obmen_all            # Число обменов данными
uint32 obmen_bed            # Число обменов с ошибками