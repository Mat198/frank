import numpy as np 
from scipy.linalg import expm

def to_skew_symmetric(omega):
    # Cria a matriz anti-simétrica 3x3 de um vetor 3x1
    return np.array([
        [0, -omega[2], omega[1]],
        [omega[2], 0, -omega[0]],
        [-omega[1], omega[0], 0]
    ])

def twist_to_matrix(S):
    # Converte o vetor de Twist S (6x1) para a matriz [S] (4x4)
    S_flat = np.ravel(S)

    omega = S_flat[:3]  # Primeiros 3 elementos (vetor de rotação)
    v = S_flat[3:]      # Últimos 3 elementos (vetor de translação)
    
    omega_skew = to_skew_symmetric(omega)
    
    # Cria a matriz 4x4
    S_matrix = np.zeros((4, 4))
    S_matrix[:3, :3] = omega_skew
    S_matrix[:3, 3] = v
    
    return S_matrix

# Matriz da posição do efetuador com as juntas iguais a 0
M = np.matrix([[-1, 0, 0, 0.287], [0, -1, 0, 0], [0, 0, 1, 2.072], [0, 0, 0, 1]])

# Ponto arbitrário em cada junta. Escolhido na origem do DH com ângulo de juntas igual a 0
a1 = np.matrix([0, 0, 0])
a2 = np.matrix([0.287, 0, 0.155])
a3 = np.matrix([0.287, 0, 0.955])
a4 = np.matrix([0.287, 0, 1.755])
a5 = np.matrix([0.287, 0, 2.072])

# Screw de rotação - segue os mesmos eixos definidos no DH
Sw1 = np.matrix([0, 0, 1])
Sw2 = np.matrix([0, -1, 0])
Sw3 = np.matrix([0, -1, 0])
Sw4 = np.matrix([0, -1, 0])
Sw5 = np.matrix([0, 0, 1])
print("Screws de translação: ")
print(Sw1)
print(Sw2)
print(Sw3)
print(Sw4)
print(Sw5)
print("")

# Screw de translação
Sv1 = -np.cross(Sw1, a1)
Sv2 = -np.cross(Sw2, a2)
Sv3 = -np.cross(Sw3, a3)
Sv4 = -np.cross(Sw4, a4)
Sv5 = -np.cross(Sw5, a5)
print("Screws de translação: ")
print(Sv1)
print(Sv2)
print(Sv3)
print(Sv4)
print(Sv5)
print("")

# Screw completo 6D
# S1 = np.append(Sw1, Sv1)
S1 = np.concatenate((Sw1, Sv1), axis=1)
S2 = np.concatenate((Sw2, Sv2), axis=1)
S3 = np.concatenate((Sw3, Sv3), axis=1)
S4 = np.concatenate((Sw4, Sv4), axis=1)
S5 = np.concatenate((Sw5, Sv5), axis=1)
print("Screws: ")
print(S1)
print(S2)
print(S3)
print(S4)
print(S5)
print("")


# Matriz de transformação
theta1 = 0
theta2 = -np.pi/4
theta3 = -np.pi/4
theta4 = 0
theta5 = 0
# Conversão e exponenciação de cada termo
T1 = expm(twist_to_matrix(S1) * theta1)
T2 = expm(twist_to_matrix(S2) * theta2)
T3 = expm(twist_to_matrix(S3) * theta3)
T4 = expm(twist_to_matrix(S4) * theta4)
T5 = expm(twist_to_matrix(S5) * theta5)
print("Exponenciais: ")
print(T1)
print(T2)
print(T3)
print(T4)
print(T5)
print("")

print("Matriz de Transformação T:")
T = T1 @ T2 @ T3 @ T4 @ T5 @ M
# Bate com o do Peter Corke!

float_formatter = "{:.3f}".format
np.set_printoptions(formatter={'float_kind': float_formatter})
print(T)