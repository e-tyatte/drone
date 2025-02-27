''' Программирование дрона для полёта по треугольной траектории. '''

# Из библиотеки dronekit импортируем connect для связи с дроном
# Из библиотеки dronekit импортируем VehicleMode для изменения режима полёта дрона
# Из библиотеки dronekit импортируем LocationGlobalRelative для изменения местоположения дрона
from dronekit import connect, VehicleMode, LocationGlobalRelative 

# Подключаем библиотеку math для проведения необходимых вычислений
# Подключаем библиотеку time
import time, math

# соединение

connection_string = '127.0.0.1:14550'

vehicle = connect(connection_string, wait_ready=True, heartbeat_timeout=60)

# запуск двигателей и набор высоты
def arm_and_takeoff(target_altitude):
	print("запуск двигателей")
	vehicle.mode = VehicleMode("GUIDED")
	vehicle.armed = True
	
	while not vehicle.armed:  # ожидание запуска двигателей
		print("Ожидание запуска")
		time.sleep(1)
		
	print("запуск двигателей")
	
	vehicle.simple_takeoff(target_altitude)  # набор высоты
	
	while True:
		print("Текущая высота:", vehicle.location.global_relative_frame.alt)  # вывод текущей высоты
		if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:  # если высота набрана
			print("Высота набрана")
			break  # прерываем набор высоты
			
		time.sleep(1)
          
def send_to(latitude, longitude, altitude):

    """

    Эта функция отправит беспилотник в нужное место, когда
летательный аппарат находится в управляемом режиме.

    Ввод:
        1. latitude  - Широта пункта назначения
        2. longitude - Долгота пункта назначения
        3. altitude - высота полёта летательного аппарата

    """
# если задан управляемый режим, можно вычислить координаты пункта назначения
    if vehicle.mode.name == "GUIDED":
        location = LocationGlobalRelative(latitude, longitude, float(altitude))
        vehicle.simple_goto(location)
        time.sleep(1)

def distance_calculation(homeLattitude, homeLongitude, destinationLattitude, destinationLongitude):

    """

    Эта функция возвращает расстояние между двумя географическими точками, используя
    формулу хаверсина.

    Входные данные:
        1.  homeLattitude          -   широта базы или текущего местоположения
        2.  homeLongitude          -    долгота базы или текущего местоположения
        3.  destinationLattitude   -   широта пункта назначения
        4.  destinationLongitude   -   долгота пункта назначения

    """

    # Радиус земли в метрах
    R = 6371e3

# Переводим широту и долготу из градусов в радианы
    rlat1, rlon1 = homeLattitude * (math.pi/180), homeLongitude * (math.pi/180)
    rlat2, rlon2 = destinationLattitude * (math.pi/180), destinationLongitude * (math.pi/180)

    # формула хаверсина
    dlat = (destinationLattitude - homeLattitude) * (math.pi/180)
    dlon = (destinationLongitude - homeLongitude) * (math.pi/180)

    # формула для определения расстояния
    a = (math.sin(dlat/2) * math.sin(dlat/2)) + (math.cos(rlat1) * math.cos(rlat2) * (math.sin(dlon/2) * math.sin(dlon/2)))
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

    # Расстояние (в метрах)
    distance = R * c

    return distance

def destination_location(homeLattitude, homeLongitude, distance, bearing):

    """

    Эта функция возвращает широту и долготу
конечного местоположения, если указаны расстояние и азимут.

    Входные данные:
        1.  homeLattitude       -  широта базы или текущего местоположения
        2.  homeLongitude       -   долгота базы или текущего местоположения
        3.  distance            -   расстояние от места расположения базы
        4.  bearing             -   угол пеленга от исходного местоположения

    """

    # Радиус земли в метрах
    R = 6371e3

# Переводим широту и долготу из градусов в радианы
    rlat1, rlon1 = homeLattitude * (math.pi/180), homeLongitude * (math.pi/180)

    d = distance

    # Преобразование азимута в радианы
    bearing = bearing * (math.pi/180)

# применяем формулу для вычисления расстояния
    rlat2 = math.asin((math.sin(rlat1) * math.cos(d/R)) + (math.cos(rlat1) * math.sin(d/R) * math.cos(bearing)))
    rlon2 = rlon1 + math.atan2((math.sin(bearing) * math.sin(d/R) * math.cos(rlat1)) , (math.cos(d/R) - (math.sin(rlat1) * math.sin(rlat2))))

    # Преобразование в градусы
    rlat2 = rlat2 * (180/math.pi) 
    rlon2 = rlon2 * (180/math.pi)

    # Широта и долгота в виде списка
    location = [rlat2, rlon2]

    return location

def triangle_calculation(side_length):

    """

   Эта функция сгенерирует географические координаты (широты и долготы)
    треугольной траектории (равностороннего треугольника) с заданной длиной стороны. Исходным или
базовым местоположением для построения треугольной траектории является текущее местоположение летательного аппарата.

    Входные данные:
        1.  side_length         -   Длина стороны равностороннего треугольника

    """

    # Направление движения и текущее местоположение летательного аппарата
    angle          =  int(vehicle.heading)
    loc            =  (vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, vehicle.location.global_relative_frame.alt)

    # Угол смещения
    offset_angle   =  90

    # Уменьшение угла смещения курсового угла летательного аппарата для формирования 
    # направления треугольника относительно угла поворота летательного аппарата.
    angle         -=  offset_angle

    # Объявление нового списка для хранения
    # географического расположения точек треугольника
    final_location =  []

    for count in range(3):
        # Увеличивающийся курсовой угол
        # Внешний угол равностороннего треугольника = 120 градусов
        angle  += 120

       # вычисляем конечные координаты
        new_loc =  destination_location(homeLattitude = loc[0], homeLongitude = loc[1], distance = side_length, bearing = angle)
        final_location.append((new_loc[0], new_loc[1], loc[2]))
        loc     =  (new_loc[0], new_loc[1], loc[2])

    return final_location

def triangular_mission(side_length):

    """

    Эта функция извлекает координаты треугольника из функции triangle_calculation()
    и направляет транспортное средство к найденным точкам.

    Входные данные:
        1.  side_length         -   Длина стороны равностороннего треугольника

    """

    # Получение списка местоположений треугольного контура
    locations  =  triangle_calculation(side_length = side_length)

    for location in locations:

        # Отправляем летательный аппарат в пункт назначения
        send_to(latitude = location[0], longitude = location[1], altitude = location[2])

        while True:

            # Расстояние между текущим местоположением летательного аппарата и пунктом назначения
            distance = distance_calculation(homeLattitude = vehicle.location.global_frame.lat,
                                            homeLongitude = vehicle.location.global_frame.lon,
                                            destinationLattitude  = location[0],
                                            destinationLongitude = location[1])

            if distance <= 1.8:
                break # прервать цикл, если расстояние меньше или равно 1,8 метров

            time.sleep(2)


# координаты базы		
home_lat = -35.36326155 
home_lon = 149.16527625 

print("Получение некоторых значений атрибутов транспортного средства:")
print("Прошивка автопилота", vehicle.version)  # версия автопилота
print("GPS: ", vehicle.gps_0)  # параметры gps
print("Батарея: ", vehicle.battery)  # состояние батареи
print("Последний отклик: ", vehicle.last_heartbeat)  # последний отклик
print('Двигатели запущены?: ', vehicle.is_armable)  # запущены ли двигатели
print("Статус системы: ", vehicle.system_status.state)  # статус системы
print("Режим: ", vehicle.mode.name)  # установленный режим

target_altitude = 20  # высота на которую нужно подняться
arm_and_takeoff(target_altitude)  # запускаем двигатели и взлетаем на заданную высоту

# длины сторон равносторонних треугольников в виде списка
side = [10, 20, 30, 40, 50]


for i in side:
   # разворачиваем список с длинами сторон треугольника
   side.reversed()
   # поочерёдно подставляем длины сторон равностороннего треугольника в функцию
    triangular_mission(side_length = i) 

print("RTL")
vehicle.mode = VehicleMode("RTL")  # установлен режим возвращения на базу
time.sleep(10)
vehicle.close()  # закрытие соединения
	
print("Готово")
