import time
import numpy as np
import math
import random
import csv

# ================= DIMENSÕES DO ROARM =================
NUMBER_OF_JOINTS = 4
l1 = 123.06 / 1000.0
l2 = 238.71 / 1000.0
l3 = 280.15 / 1000.0

# ================= PARÂMETROS DO RRT =================
MAX_ITER = 5000 
STEP_SIZE = 0.1
GOAL_THRESHOLD = 0.05 
TRIALS = 100 # Número de testes a serem executados

JOINT_LIMITS = [
    (-np.pi, np.pi),
    (-np.pi/2, np.pi/2),
    (-np.pi/2, np.pi/2),
    (0, 0)
]

# ================= AMBIENTE DENSO =================
OBSTACLES_LIST = [
    # 1. Pilar original (Frente-Esquerda)
    {'type': 'cylinder', 'name': 'Pilar1', 'x': -0.07, 'y': 0.17, 'z': 0.0, 'r': 0.025, 'h': 0.245},
    # 2. Caixa original (Chão)
    {'type': 'box', 'name': 'Base1', 'x': -0.037, 'y': 0.15, 'z': 0.0, 'size_x': 0.23, 'size_y': 0.06, 'size_z': 0.1},
    # 3. NOVO: Muro Lateral (Direita)
    {'type': 'box', 'name': 'MuroLateral', 'x': 0.1, 'y': 0.10, 'z': 0.0, 'size_x': 0.05, 'size_y': 0.30, 'size_z': 0.20},
    # 4. NOVO: Pilar Secundário (Atrás)
    {'type': 'cylinder', 'name': 'Pilar2', 'x': -0.10, 'y': -0.15, 'z': 0.0, 'r': 0.030, 'h': 0.300}
]

# ================= CINEMÁTICA E COLISÃO =================
def fkine_all_joints(angulos):
    """ Retorna as posições 3D [Base, Ombro, Cotovelo, Garra] """
    th1 = angulos[0]
    th2 = -angulos[1] + (np.pi / 2.0) - 0.12601
    th3 = -angulos[2] + 0.12601
    
    p0 = np.array([0.0, 0.0, 0.0])
    p1 = np.array([0.0, 0.0, l1])
    
    r_elbow = l2 * np.cos(th2)
    z_elbow = l1 + l2 * np.sin(th2)
    p2 = np.array([r_elbow * np.cos(th1), r_elbow * np.sin(th1), z_elbow])
    
    r_tip = r_elbow + l3 * np.cos(th2 + th3)
    z_tip = z_elbow + l3 * np.sin(th2 + th3)
    p3 = np.array([r_tip * np.cos(th1), r_tip * np.sin(th1), z_tip])
    
    return [p0, p1, p2, p3]

def is_point_colliding(pos):
    """ Verifica se um ponto XYZ (tx, ty, tz) está dentro de algum obstáculo """
    tx, ty, tz = pos[0], pos[1], pos[2]
    if tz < 0.0: return True # Colisão com o chão
    
    for obs in OBSTACLES_LIST:
        ox, oy = obs['x'], obs['y']
        if obs['type'] == 'cylinder':
            dist_xy = math.sqrt((tx - ox)**2 + (ty - oy)**2)
            if dist_xy < obs['r'] and tz < obs['h']:
                return True
        elif obs['type'] == 'box':
            hx, hy = obs['size_x']/2.0, obs['size_y']/2.0
            if (ox - hx < tx < ox + hx) and (oy - hy < ty < oy + hy) and (tz < obs['size_z']):
                return True
    return False

# ================= RRT ENGINE =================
class Node:
    def __init__(self, q):
        self.q = np.array(q)
        self.parent = None

class RRT:
    def __init__(self, start_q, goal_xyz, mode="sca"):
        self.start = Node(start_q)
        self.goal_xyz = np.array(goal_xyz)
        self.nodes = [self.start]
        self.mode = mode # "baseline" ou "sca"

    def get_random_config(self):
        q = []
        for i in range(NUMBER_OF_JOINTS):
            if JOINT_LIMITS[i][0] == JOINT_LIMITS[i][1]: q.append(JOINT_LIMITS[i][0])
            else: q.append(random.uniform(JOINT_LIMITS[i][0], JOINT_LIMITS[i][1]))
        return np.array(q)

    def steer(self, from_node, q_rand):
        dir_vec = q_rand - from_node.q
        dist = np.linalg.norm(dir_vec)
        if dist > STEP_SIZE:
            new_q = from_node.q + (dir_vec / dist) * STEP_SIZE
        else:
            new_q = q_rand
        return Node(new_q)

    def check_collision(self, q):
        joint_positions = fkine_all_joints(q)
        
        if self.mode == "baseline":
            # BASELINE: Checa APENAS a ponta da garra
            return is_point_colliding(joint_positions[-1])
            
        else:
            # SCA: Interpola ao longo dos elos do corpo inteiro
            steps = 8
            for i in range(len(joint_positions) - 1):
                p_start, p_end = joint_positions[i], joint_positions[i+1]
                for t in range(steps + 1):
                    ratio = t / float(steps)
                    p_inter = (1 - ratio)*p_start + ratio*p_end
                    if is_point_colliding(p_inter):
                        return True
            return False

    def has_edge(self, q1, q2):
        steps = 10 
        for t in range(1, steps):
            ratio = t / float(steps)
            q_new = (1 - ratio)*q1 + ratio*q2
            if self.check_collision(q_new):
                return False
        return True
        
    def planning(self):
        for i in range(MAX_ITER):
            q_rand = self.get_random_config()
            nearest = min(self.nodes, key=lambda node: np.linalg.norm(node.q - q_rand))
            new_node = self.steer(nearest, q_rand)
            
            if not self.check_collision(new_node.q):
                if self.has_edge(nearest.q, new_node.q):
                    new_node.parent = nearest
                    self.nodes.append(new_node)
                    
                    dist = np.linalg.norm(fkine_all_joints(new_node.q)[-1] - self.goal_xyz)
                    if dist < GOAL_THRESHOLD:
                        # Extrai caminho
                        path = []
                        curr = new_node
                        while curr is not None:
                            path.append(curr.q)
                            curr = curr.parent
                        return path[::-1]
        return None

# ================= FUNÇÕES AUXILIARES =================
def is_path_tunneling(path):
    """ Escaneia um caminho usando as regras estritas do SCA para ver se bateu """
    checker = RRT(path[0], [0,0,0], mode="sca")
    for i in range(len(path) - 1):
        if not checker.has_edge(path[i], path[i+1]):
            return True # O braço atravessou um objeto!
    return False

def get_valid_random_goal():
    """ Gera um XYZ aleatório que seja garantidamente alcançável e livre de colisão corpo inteiro """
    checker = RRT(np.zeros(NUMBER_OF_JOINTS), [0,0,0], mode="sca")
    while True:
        q_rand = checker.get_random_config()
        if not checker.check_collision(q_rand): # Se o corpo todo está seguro na pose
            return fkine_all_joints(q_rand)[-1] # Retorna o XYZ da garra

# ================= EXECUÇÃO PRINCIPAL =================
def main():
    start_q = np.zeros(NUMBER_OF_JOINTS)
    csv_filename = "rrt_comparativo.csv"
    
    print(f"Iniciando {TRIALS} testes pareados (Baseline vs SCA-RRT)...")
    
    with open(csv_filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['Trial', 'Goal_X', 'Goal_Y', 'Goal_Z', 
                         'Base_Success', 'Base_Time', 'Base_Nodes', 'Base_Tunneling',
                         'SCA_Success', 'SCA_Time', 'SCA_Nodes'])
        
        for trial in range(1, TRIALS + 1):
            goal_xyz = get_valid_random_goal()
            print(f"\n--- Trial {trial}/{TRIALS} | Meta XYZ: {goal_xyz[0]:.2f}, {goal_xyz[1]:.2f}, {goal_xyz[2]:.2f} ---")
            
            # 1. RODA O BASELINE
            planner_base = RRT(start_q, goal_xyz, mode="baseline")
            t0 = time.time()
            path_base = planner_base.planning()
            time_base = time.time() - t0
            
            base_success = path_base is not None
            base_nodes = len(planner_base.nodes)
            base_tunneling = False
            if base_success:
                base_tunneling = is_path_tunneling(path_base)
            
            print(f"[Baseline] Sucesso: {base_success} | Tempo: {time_base:.2f}s | Nós: {base_nodes} | Túnel: {base_tunneling}")
            
            # 2. RODA O SCA-RRT
            planner_sca = RRT(start_q, goal_xyz, mode="sca")
            t0 = time.time()
            path_sca = planner_sca.planning()
            time_sca = time.time() - t0
            
            sca_success = path_sca is not None
            sca_nodes = len(planner_sca.nodes)
            
            print(f"[SCA-RRT]  Sucesso: {sca_success} | Tempo: {time_sca:.2f}s | Nós: {sca_nodes}")
            
            # SALVA NO CSV
            writer.writerow([trial, goal_xyz[0], goal_xyz[1], goal_xyz[2],
                             base_success, f"{time_base:.3f}", base_nodes, base_tunneling,
                             sca_success, f"{time_sca:.3f}", sca_nodes])

    print(f"\n[FIM] Dados salvos em '{csv_filename}'.")

if __name__ == "__main__":
    main()