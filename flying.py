from PID import pid_class
from Rocket import rocket
from numpy.linalg import norm
import math as m
import matplotlib.pyplot as plt
import numpy as np
import sqlite3
connection = sqlite3.connect("flight_data.db")
data = connection.cursor()
sql_command = """delete from data;"""
# SQL command to create a table in the database


pozitron = rocket()

peng = 395900
peng2 = 58800

w = 3300
dm = peng / w
s1 = 11
s2 = 9
mb1 = 27640
mb2 = 5000

#Плотности
po = 1140
pg = 440
kompsot = 3.5

#Основные габариты
D = 1.7
Lgo = 3.4+0.1
Loh1 = 2.8
Loh2 = 2.1
Lst1 = 17.4
Lst2 = 4.7
Lrocket = 25.6
#Массы компонентов топлива (константы)
mt1 = 25120
mt2 = 4440
#Массы компонентов топлива
mg1 = mt1 * 1 / (1 + kompsot)
mg2 = mt2 * 1 / (1 + kompsot)
mo1 = mt1 * kompsot / (1 + kompsot)
mo2 = mt2 * kompsot / (1 + kompsot)

#Объемы компонентов топлива
wg1 = mg1 / pg
wg2 = mg2 / pg
wo1 = mo1 / po
wo2 = mo2 / po

#Массы и объемы конструкции блоков
mk1 = mb1 - mt1
mk2 = mb2 - mt2
wk1 = mk1/2700
wk2 = mk2/2700

mass = mk1 + mk2 + mt1 + mt2 + 1000

#Высоты столба жидкости
Lg1 = wg1 * 4 / (np.pi*(D**2))
Lg2 = wg2 * 4 / (np.pi*(D**2))

Lo1 = wo1 * 4 / (np.pi*(D**2))
Lo2 = wo2 * 4 / (np.pi*(D**2))
#Высоты баков
Ug1 = Lg1*1/0.9
Ug2 = Lg2*1/0.9
Uo1 = Lo1*1/0.9
Uo2 = Lo2*1/0.9

#Уровни топлива
K21o = Lgo + Uo2 - Lo2
K22o = Lgo + Uo2
K2summo = K21o + K22o

K21g = K22o + Ug2 - Lg2
K22g = K22o + Ug2
K2summg = K21g + K22g

K11o = K22g + Loh2 + Uo1 - Lo1
K12o = K22g + Loh2 + Uo1
K1summo = K11o + K12o

K11g = K12o + Uo1 - Lg1
K12g = K12o + Uo1
K1summg = K11g + K12g

#Стартовые моменты инерции:

#_Конструкция
K2summ = Lgo + (K22g + Loh2)
K1summ = (K22g + Loh2) + (Loh1 + K12g)
Sk2 = 0.5 * K2summ * mk2
Sk1 = 0.5 * K1summ * mk1
Sk = Sk1 + Sk2

Ik2 = 0.25 * (K2summ**2) + (D/2)**2 + 0.333 * (Lst2**2) * mk2
Ik1 = 0.25 * (K1summ**2) + (D/2)**2 + 0.333 * (Lst1**2) * mk1

Ik = Ik1 + Ik2
#_Топливо
Stg1 = mg1 * K1summg * 0.5
Stg2 = mg2 * K2summg * 0.5

Sto1 = mo1 * K1summo * 0.5
Sto2 = mo2 * K2summo * 0.5

Sto = Sto1 + Sto2
Stg = Stg1 + Stg2

Itg1 = mg1 * (K1summg**2) + (D/2)**2 + 0.333 * (Ug1**2) * 0.25
Itg2 = mg2 * (K2summg**2) + (D/2)**2 + 0.333 * (Ug2**2) * 0.25

Ito1 = mo1 * (K1summo**2) + (D/2)**2 + 0.333 * (Uo1**2) * 0.25
Ito2 = mo2 * (K2summo**2) + (D/2)**2 + 0.333 * (Uo2**2) * 0.25


Ito = Ito1 + Ito2
Itg = Itg1 + Itg2
Isumm = Ito + Itg + Ik + (D * (Lgo**3) / 12)
Ssumm = Sto + Stg + Sk + (D * (Lgo**2) / 24)
Xcm = Ssumm / mass

print(Xcm)
print(Lst1+Lst2+Lgo)


def goal(time):
    res = (m.pi/2)*m.exp(-0.005*time)*(m.exp(0.005*time)-1)
    return res

def aero_force(V,a,h):
    CY = 0.6*25*3
    CX = 0.08*m.pi*(1.6/2)**2
    w = 0
    if 0 < h <= 10.5:
        w = 11.5 * m.exp(0.195 * h)
    if 10.5 < h <= 27:
        w = 21 * m.exp(4.93 * (1e-3) * (27 - h) ** 2)
    if h > 27:
        w = 21

    wind = np.array([-25, 0])

    if norm(V) < 0.5: a=a - m.pi/2
    else: a=a-m.atan(wind[0]/norm(V))

    rho = 1.29*m.exp(-h/8)
    V = V+wind
    mas = np.array([-CX * m.cos(a),
                     CY * m.sin(a)])

    mas = mas * rho * (norm(V) ** 2) / 2

    return a, mas


def thrust(P_, ang):
    mas = np.array([m.cos(ang),
                    -m.sin(ang)])
    return P_ * mas


def rot_1(vec, ang):
    rot = np.array([[np.sin(ang), np.cos(ang)],
                    [-np.cos(ang), np.sin(ang)]])

    return np.matmul(vec, rot)


def rot_2(vec, ang):
    rot = np.array([[np.cos(ang), np.sin(ang)],
                    [-np.sin(ang), np.cos(ang)]])
    return np.matmul(vec, rot)

def atan(vec):
    if vec[1] == 0:
        res = 0

    else:
        res = m.atan(vec[0] / vec[1])

    return res

def engine_move(dang, beta):
    u_max = 0.3;
    u = u_max * np.sign(dang - beta*abs(beta)/(2*u_max))
    return u

x_list = list()
y_list = list()
t_list = list()
test_list = list()
test_list2 = list()
test_list3 = list()
test_list4 = list()
test_list5 = list()
test_list6 = list()
test_list7 = list()
xc_list = list()
inert_list = list()
Vel = np.array([0, 0])
Pos = np.array([0, 0])
omega = 0
pitch = 0
phi = 0
beta = 0
g = np.array([0, -9.81])


t = 0
h = 0.01




pid = pid_class(h,0,2,0.1,1)

while mt1 > 0:
    if Pos[1] < 0:
        print('Falling')
        break
    t_list.append(t)
    x_list.append(Pos[0])
    y_list.append(Pos[1])
    vel_ang = atan(Vel)
    attack_angle = vel_ang - pitch



    attack_angle, R = aero_force(Vel, attack_angle,Pos[1]/1000)

    test_list.append(180 * vel_ang / m.pi)
    test_list2.append(180 * pitch / m.pi)
    test_list3.append(180 * attack_angle / m.pi)


    NQ = rot_2(R, attack_angle)

    goal_pitch = goal(t)




    pid.update_goal(goal_pitch)

    goal_phi = pid.gen_signal(pitch)
    if abs(goal_phi) > 10*m.pi/180:
        goal_phi = 10*m.pi/180*np.sign(goal_phi)


    beta = beta + engine_move(goal_phi-phi, beta)*h
    phi = phi + beta *h


    P = thrust(peng, phi)

    mt1 = mt1 - dm * h

    # Массы компонентов топлива
    mg1 = mt1 * 1 / (1 + kompsot)
    mg2 = mt2 * 1 / (1 + kompsot)
    mo1 = mt1 * kompsot / (1 + kompsot)
    mo2 = mt2 * kompsot / (1 + kompsot)

    # Объемы компонентов топлива
    wg1 = mg1 / pg
    wg2 = mg2 / pg
    wo1 = mo1 / po
    wo2 = mo2 / po

    # Массы и объемы конструкции блоков
    mk1 = mb1 - mt1
    mk2 = mb2 - mt2
    wk1 = mk1 / 2700
    wk2 = mk2 / 2700

    mass = mk1 + mk2 + mt1 + mt2 + 1000

    # Длины баков
    Lg1 = wg1 * 4 / (np.pi * (D ** 2))
    Lg2 = wg2 * 4 / (np.pi * (D ** 2))

    Lo1 = wo1 * 4 / (np.pi * (D ** 2))
    Lo2 = wo2 * 4 / (np.pi * (D ** 2))

    Ug1 = Lg1
    Ug2 = Lg2
    Uo1 = Lo1
    Uo2 = Lo2

    # Уровни топлива
    K21o = Lgo + Uo2 - Lo2
    K22o = Lgo + Uo2
    K2summo = K21o + K22o

    K21g = K22o + Ug2 - Lg2
    K22g = K22o + Ug2
    K2summg = K21g + K22g

    K11o = K22g + Loh2 + Uo1 - Lo1
    K12o = K22g + Loh2 + Uo1
    K1summo = K11o + K12o

    K11g = K12o + Uo1 - Lg1
    K12g = K12o + Uo1
    K1summg = K11g + K12g

    # Стартовые моменты инерции:

    # _Конструкция
    K2summ = K21o + K22g;
    K1summ = K11o + K12g
    Sk2 = 0.5 * K2summ * mk2
    Sk1 = 0.5 * K1summ * mk1
    Sk = Sk1 + Sk2

    Ik2 = 0.25 * (K2summ ** 2) + (D / 2) ** 2 + 0.333 * (Lst2 ** 2) * mk2
    Ik1 = 0.25 * (K1summ ** 2) + (D / 2) ** 2 + 0.333 * (Lst1 ** 2) * mk1

    Ik = Ik1 + Ik2
    # _Топливо
    Stg1 = mg1 * K1summg * 0.5
    Stg2 = mg2 * K2summg * 0.5

    Sto1 = mo1 * K1summo * 0.5
    Sto2 = mo2 * K2summo * 0.5

    Sto = Sto1 + Sto2
    Stg = Stg1 + Stg2

    Itg1 = mg1 * (K1summg ** 2) + (D / 2) ** 2 + 0.333 * (Ug1 ** 2) * 0.25
    Itg2 = mg2 * (K2summg ** 2) + (D / 2) ** 2 + 0.333 * (Ug2 ** 2) * 0.25

    Ito1 = mo1 * (K1summo ** 2) + (D / 2) ** 2 + 0.333 * (Uo1 ** 2) * 0.25
    Ito2 = mo2 * (K2summo ** 2) + (D / 2) ** 2 + 0.333 * (Uo2 ** 2) * 0.25

    Ito = Ito1 + Ito2
    Itg = Itg1 + Itg2
    Isumm = Ito + Itg + Ik + (D * (Lgo ** 3) / 12)
    Ssumm = Sto + Stg + Sk + (D * (Lgo ** 2) / 24)
    Xcm = Ssumm / mass

    # минус-стабильно плюс-нестабильно
    omega = omega + (P[1] * (5) / Isumm - NQ[1] * (3) / Isumm) * h

    pitch = pitch + omega * h

    magic = mass * np.array([0, 1])*(Vel[0]**2)/(6371000+Pos[1])

    Vel = Vel + (rot_1(R, vel_ang) + rot_1(P, pitch) + mass * g+magic) * h / mass

    Pos = Pos + Vel * h

    inert_list.append(Isumm)
    xc_list.append(Xcm)
    test_list4.append(NQ[1])
    test_list5.append(NQ[0])
    test_list6.append(omega)

    data.execute("INSERT INTO DATA VALUES (?,?,?,?,?, ?, ?, ?, ?, ?, ?, ?)", (t, mass, norm(Vel) * 3.6, Pos[1] /1000, peng, Xcm, 0, 180 * pitch / m.pi, 180 * vel_ang / m.pi, 180 * attack_angle / m.pi, NQ[0], NQ[1]))
    t = t + h


print('Cкорости(км/ч): ',norm(Vel) * 3.6)
print('Высота(км): ',Pos[1] /1000)


plt.plot(t_list, inert_list, label='Момент  инерции ')
plt.legend()
plt.xlabel('Секунда полёта')
plt.ylabel('кг*м^2')
plt.grid()
plt.show()


plt.plot(t_list, xc_list, label='Координта ц.т. ')
plt.legend()
plt.xlabel('Секунда полёта')
plt.ylabel('м')
plt.grid()
plt.show()


plt.plot(x_list, y_list)
plt.show()

plt.plot(t_list, test_list, label = 'Наклон траектории')
plt.plot(t_list, test_list2,  label = 'Тангаж')
plt.plot(t_list, test_list3, label = 'Угол Атаки')
plt.legend(loc=4)
plt.xlabel('Секунда полёта')
plt.ylabel('Градус')
plt.grid()
plt.show()

plt.plot(test_list4, t_list,label='Перерезывающая сила')
plt.plot(test_list5, t_list,label='Продольная сила')
plt.legend(loc=4)
plt.ylabel('Секунда полёта')
plt.xlabel('Ньютон')
plt.grid()
plt.show()

plt.plot(t_list, test_list6, label='вращение по тангажу ')
plt.legend(loc=4)
plt.xlabel('Секунда полёта')
plt.ylabel('рад/c')
plt.grid()
plt.show()

connection.commit()
connection.close()