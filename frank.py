import roboticstoolbox as rtb
import numpy as np
from ik import differential_ik

# Opções de print do numpy
float_formatter = "{:.4f}".format
np.set_printoptions(formatter={'float_kind': float_formatter})

# Define DH parameters for a simple 2-link manipulator
# [a, alpha, d, theta]
L1 = rtb.RevoluteDH(a=0.287, alpha=np.pi/2, d=0.155, offset=0)
L2 = rtb.RevoluteDH(a=0.8, alpha=0, d=0, offset=np.pi/2)
L3 = rtb.RevoluteDH(a=0.8, alpha=0, d=0.0, offset=0)
L4 = rtb.RevoluteDH(a=0.0, alpha=np.pi/2, d=0.0, offset=np.pi/2)
L5 = rtb.RevoluteDH(a=0.0, alpha=0, d=0.317, offset=0)
# Create the robot model
robot = rtb.DHRobot([L1, L2, L3, L4, L5], name="Frank")
print(robot)
q = [0, -np.pi/4, -np.pi/4, 0, 0]
robot.plot(q, backend='pyplot', block=True)

print("T1")
print(L1.A(np.pi/6))
print("T2")
print(L2.A(np.pi/6))
print("T3")
print(L3.A(np.pi/6))
print("T4")
print(L4.A(np.pi/6))
print("T5")
print(L5.A(np.pi/6))
# You can now perform operations like forward kinematics:
angle = 0
print("O1")
print(L1.A(angle))
print("O2")
print(L1.A(angle) @ L2.A(angle))
print("O3")
print(L1.A(angle) @ L2.A(angle) @ L3.A(angle))
print("O4")
print(L1.A(angle) @ L2.A(angle) @ L3.A(angle) @ L4.A(angle))
print("O5")
print(L1.A(angle) @ L2.A(angle) @ L3.A(angle) @ L4.A(angle) @ L5.A(angle))

print("O5 - Origem")
print(L1.A(0) @ L2.A(-np.pi/4) @ L3.A(-np.pi/4) @ L4.A(-np.pi/2) @ L5.A(0))
q = [0, -np.pi/4, -np.pi/4, 0, 0]
q = [0, 0, 0, 0, 0]
# q = [np.pi/6, np.pi/6 , np.pi/6, np.pi/6, np.pi/6]
robot.plot(q, backend='pyplot', block=True)
jacob = robot.jacob0(q)
print("Size of the Jacobian: ", len(jacob), "x", len(jacob[0]))
print([format(i, '>7.4f') for i in jacob[0]])
print([format(i, '>7.4f') for i in jacob[1]])
print([format(i, '>7.4f') for i in jacob[2]])
print([format(i, '>7.4f') for i in jacob[3]])
print([format(i, '>7.4f') for i in jacob[4]])
print([format(i, '>7.4f') for i in jacob[5]])

# Simula uma aplicação de pick and place
qi = [0, -np.pi/4, -np.pi/4, -np.pi/2, 0]
initial_fk = robot.fkine(qi)

task = [
    {"pos": initial_fk.t, "ori": [np.pi, 0, 0]}, # Apenas rodando
    {"pos": [1.2, 0.4, 0.5], "ori": [np.pi, 0, 0]}, # Indo até a acima do objeto
    {"pos": [1.2, 0.4, 0.2], "ori": [np.pi, 0, 0]}, # Descendo para pegar
    {"pos": [1.2, 0.4, 0.5], "ori": [np.pi, 0, 0]}, # Retornando
    {"pos": [1.5, -0.4, 0.5], "ori": [np.pi, 0, 0]}, # Indo até a posição de deposito
    {"pos": [1.5, -0.4, 0.2], "ori": [np.pi, 0, 0]}, # Descendo para colocar
    {"pos": [1.5, -0.4, 0.5], "ori": [np.pi, 0, 0]}, # Subindo depois de colocar
    {"pos": initial_fk.t, "ori": initial_fk.rpy()}, # Retornando para posição inicial
]

task_counter = 0
for point in task:
    print(f"\nExecuting task {task_counter}")

    final_pos = point["pos"]
    final_ori = point["ori"]
    qn = differential_ik(robot, qi, final_pos, final_ori)

    print("Final IK position is: ", qn)
    traj = rtb.jtraj(qi, qn, 50)
    robot.plot(traj.q)
    qi = qn
    task_counter += 1
