import math
import numpy as np
import matplotlib.pyplot as plt

def forward_kinematics2DoF():
    alpha = float(input("Masukkan sudut alpha : "))
    a = math.radians(alpha)
    L1 = float(input("Masukkan panjang lengan 1 : "))
    beta = float(input("Masukkan sudut beta : "))
    b = math.radians(beta)
    L2 = float(input("Masukkan panjang lengan 2 : "))
    print("\n")

    h01 = np.matrix([[math.cos(a), -math.sin(a), 0],
                     [math.sin(a), math.cos(a), 0],
                     [0, 0, 1]])

    h12 = np.matrix([[1,0, L1],
                     [0,1,0],
                     [0,0,1]])
    h23 = np.matrix([[math.cos(b), -math.sin(b), 0],
                     [math.sin(b),  math.cos(b), 0],
                     [0, 0, 1]])
    h34 = np.matrix([[1,0, L2],
                     [0,1,0],
                     [0,0,1]])

    h04 = h01 @ h12 @ h23 @ h34
    
    return h04


def forward_kinematicsN():
    n = int(input("Masukkan jumlah DOF : "))


    teta = {}
    LENGTH = {}
    SUDUT = 0
    x = 0
    y = 0

    for i in range(n):
        ALPHA = float(input(f"Masukkan sudut alpha {i+1} : "))
        a= math.radians(ALPHA)
        teta[i] = a
        L = float(input(f"Masukkan panjang lengan {i+1} : "))
        LENGTH[i] = L
        print("\n")
        if i == n-1:
            break
    
    temp = np.matrix([[1,0,0],
                      [0,1,0],
                      [0,0,1]])

    for i in range(n):
        h01 = np.matrix([[math.cos(teta[i]), -math.sin(teta[i]), 0],
                         [math.sin(teta[i]), math.cos(teta[i]), 0],
                         [0, 0, 1]])

        h12 = np.matrix([[1,0, LENGTH[i]],
                         [0,1,0],
                         [0,0,1]])
        
        htotal = h01 @ h12
        temp = htotal @ temp
        if i == n-1:
            break

    return temp

def forward_kinematicsgambar():
    n = int(input("Masukkan jumlah DOF : "))


    teta = {}
    LENGTH = {}
    SUDUT = 0
    x = 0
    y = 0

    for i in range(n):
        ALPHA = float(input(f"Masukkan sudut alpha {i+1} : "))
        a= math.radians(ALPHA)
        teta[i] = a
        L = float(input(f"Masukkan panjang lengan {i+1} : "))
        LENGTH[i] = L
        print("\n")
        if i == n-1:
            break
    
    temp = np.matrix([[1,0,0],
                      [0,1,0],
                      [0,0,1]])
    x0, y0= 0, 0
    for i in range(n):
        h01 = np.matrix([[math.cos(teta[i]), -math.sin(teta[i]), 0],
                         [math.sin(teta[i]), math.cos(teta[i]), 0],
                         [0, 0, 1]])

        h12 = np.matrix([[1,0, LENGTH[i]],
                         [0,1,0],
                         [0,0,1]])
        
        htotal = h01 @ h12
        temp = htotal @ temp
        
        plt.plot([temp[0,2]], [temp[1,2]], marker='o')
        x1= temp[0,2]
        y1= temp[1,2]
        plt.plot([x0, x1], [y0, y1], "b-")
        x0, y0 = temp[0,2], temp[1,2]
        if i == n-1:
            break
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.axis('equal')
    plt.title('Forward Kinematics N-DOF Robot Arm')
    plt.grid(True)
    plt.legend()
    plt.show()

    return temp

def inverse_kinematics():
    x = float(input("Masukkan posisi x target: "))
    y = float(input("Masukkan posisi y target: "))
    l1 = float(input("Masukkan panjang lengan 1: "))
    l2 = float(input("Masukkan panjang lengan 2: "))
    print("\n")

    D = (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2)

    if D < -1 or D > 1:
        print("posisi target tidak dapat dijangkau dengan panjang lengan yang diberikan.")
    
    teta2 = math.acos(D)
    teta1 = math.atan2(y, x) - math.atan2(l2 * math.sin(teta2), l1 + l2 * math.cos(teta2))

    o1 = math.degrees(teta1)
    o2 = math.degrees(teta2)

    return o1, o2


switch = int(input("Pilih fungsi forward kinematics:\n1. 2-DOF\n2. N-DOF\n3. N-DOF dengan gambar\n4. Invers Kinematik\nMasukkan pilihan (1/2/3/4): "))  

if switch == 1:
    result = forward_kinematics2DoF()
    print("Hasil Kinematika Maju 2-DOF:")
    print(result)
elif switch == 2:
    result = forward_kinematicsN()
    print("Hasil Kinematika Maju N-DOF:")
    print(result)
elif switch == 3:
    forward_kinematicsgambar()
elif switch == 4:
    o1, o2 = inverse_kinematics()
    print("Invers Kinematik untuk 2-DOF Robot Arm")
    print(f"Sudut teta1: {o1} derajat")
    print(f"Sudut teta2: {o2} derajat")
else:
    print("Pilihan tidak valid.")
