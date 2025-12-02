import numpy as np
import roboticstoolbox as rtb

def differential_ik(robot: rtb.DHRobot, qi, final_pos, final_ori):
    initial_fk = robot.fkine(qi)
    initial_pos = initial_fk.t 
    initial_ori = initial_fk.rpy()
    # Cinemática inversa diferencial
    max_iterations = 100
    # Calcular velocidade inicial com base na distância da posição atual até o robô
    lin_gain = 2.0
    ang_gain = 0.2
    lin_vel = [lin_gain * (x[0] - x[1]) for x in zip(final_pos, initial_pos)]
    # ang_vel = [ang_gain * (x[0] - x[1]) for x in zip(final_ori, initial_ori)]
    ang_vel = [0,0,0]
    print(f"Initial position: {initial_pos}")
    print(f"Initial orientation: {initial_ori}")
    print(f"Final position: {final_pos}")
    print(f"Final orientation: {final_ori}")
    print(f"Linear Velocity: {lin_vel}")
    print(f"Angular Velocity: {ang_vel}")
    for i in range(0, max_iterations): 
        # TODO: Ajustar velocidade de acordo com a distância para a posição alvo
        qn = jacobian_ik(robot, qi, lin_vel=lin_vel, ang_vel=ang_vel, dt=0.1)
        curr_pos = robot.fkine(qn)
        print(f"Position on iteration {i}: {[format(i, '.3f') for i in curr_pos.t]}")
        print(f"Orientation on iteration {i}: {[format(i, '.3f') for i in curr_pos.rpy()]}")

        # Verifica se chegamos na posição alvo
        reached_pos = np.allclose(final_pos, curr_pos.t,atol=1e-3)
        reached_angle = np.allclose(final_ori, curr_pos.rpy(),atol=1e-3)
        if reached_pos and reached_angle: break
        
        # Atualiza os valores para o próximo loop
        qi = qn
        lin_vel = [lin_gain * (x[0] - x[1]) for x in zip(final_pos, curr_pos.t)]
        # ang_vel = [ang_gain * (x[0] - x[1]) for x in zip(final_ori, curr_pos.rpy())]
    return qn

def jacobian_ik(robot: rtb.DHRobot, q, lin_vel, ang_vel, dt):
    J = robot.jacob0(q)
    v = np.zeros(6)
    v[:3] = lin_vel
    v[3:] = ang_vel
    dq = np.linalg.pinv(J) @ v
    nq = q + dq * dt
    return nq