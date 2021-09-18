import math
#скрипт для перевода координат из TLE в готовые координаты дрона
x = float(input('X - '))
x1 = float(input('X1 - '))
y = float(input('Y - '))
y1 = float(input('Y1 - '))

print(math.sqrt((x-x1)**2+(y-y1)**2))