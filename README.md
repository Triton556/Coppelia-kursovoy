# Coppelia-kursovoy

Модель автоматизированного подводного аппарата, который сканирует поверхность дна при помощи 2D лидара в заданной области пространства, с задаваемой высотой над поверхностью дна.

<img src="https://sun5-4.userapi.com/s/v1/if2/_OaAi3z6sxR23Ebn5rpdLltUoZYuFa9nmx4QkXtN5pNEVTlLtbUx2zO3dAdAGEIvmhs8JNdvR-xcHOkmqllxWNua.jpg?size=593x465&quality=96&type=album" width="200" height="170">

**Для работы программы необходима виртуальная среда моделирования V-REP, в сцену которого загружен необходимый объект управления (подводный робот).**

## Задача
- объект управления - Подводный робот, управляемый по всем степеням свободы
- архитектура - Расперделённая
- функция - Картографирование

## Решение
Была написана программа на языке Python, которая управляла объектом в среде V-REP

### Общий вид меню во время выполнения

<img src="https://sun9-33.userapi.com/s/v1/if2/jJKBHeohEGQ8hx8o-Au6L5AqT_Uv9022vVJ4cYLR-GimK8OHXhl7nHxP93jfR7j6IULpiua2_dz6CXrsM8gJx9oZ.jpg?size=508x471&quality=96&type=album">

**Красный пунктир** - маршрут, построенный программой для робота в соответсвии с задаными координатами и высотой от дна.
**Зелёный крестик** - Положение робота на маршруте.

Программа аозволяет выбирать скорость дивжения робота, диагональные координаты области сканирования, высоту от дна и частоту сканирования. А так же варианты движения по прямой или по сплайну.

### Результаты работы программы
- было произведено сканирование тестового полигона

<img src="https://sun9-27.userapi.com/s/v1/if2/81TUJAUQn6GELNM3FO82HcWYip_QDaZS1KcHHeXWVX-DURMwSrycgOPOy-jgBtgVHn5ppA3pHTISv9v9SdUvq3O0.jpg?size=758x604&quality=96&type=album" width="450" height="450">

- В результате программа выдаёт облако точек в файле CSV, которое можно визуализировать при помощи Cloud Compare

<img src="https://sun9-42.userapi.com/s/v1/if2/1GbiVLjxDUj_NgSJInnjVDMYJNBekJ0nA3fINL8eEyqz1g1DmH2-x0RPZZVXPdRmd6PQmxhAE9uZwyyHsKGVMZoY.jpg?size=888x339&quality=96&type=album" width="500" height="450">
