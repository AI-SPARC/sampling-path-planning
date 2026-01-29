import serial
import argparse
import threading
import time
import numpy as np
import math
import random
import json
import sys

NUMBER_OF_JOINTS = 4
l1 = 123.06 / 1000.0
l2 = 238.71 / 1000.0
l3 = 280.15 / 1000.0

MAX_ITER = 10000 
STEP_SIZE = 0.1 
GOAL_THRESHOLD = 0.05 
JOINT_LIMITS = [
    (-np.pi, np.pi),
    (-np.pi/2, np.pi/2),
    (-np.pi/2, np.pi/2),
    (0, 0)
]

OBSTACLES_LIST = [
    {
        'type': 'cylinder',
        'name': 'Cilindro',
        'x': -0.07, 'y': 0.17, 'z': 0.0, 
        'r': 0.025,
        'h': 0.245
    },
    {
        'type': 'box',
        'name': 'Paralelepipedo',
        'x': -0.037, 'y': 0.23, 'z': 0.0,
        'size_x': 0.23, 
        'size_y': 0.06, 
        'size_z': 0.045
    }
]

ser = None

def fkine(angulos):
    th1 = angulos[0]
    th2 = -angulos[1] + (np.pi / 2.0) - 0.12601
    th3 = -angulos[2] + 0.12601
    
    r = l2*np.cos(th2) + l3*np.cos(th2 + th3)
    z = l1 + l2*np.sin(th2) + l3*np.sin(th2 + th3)
    x = r*np.cos(th1)
    y = r*np.sin(th1)
    
    return np.array([x, y, z])

class Node:
    def __init__(self, q):
        self.q = np.array(q)
        self.parent = None

class RRT:
    def __init__(self, start_q, goal_xyz):
        self.start = Node(start_q)
        self.goal_xyz = np.array(goal_xyz)
        self.nodes = [self.start]

    def get_random_config(self):
        q = []
        for i in range(NUMBER_OF_JOINTS):
            if JOINT_LIMITS[i][0] == JOINT_LIMITS[i][1]:
                q.append(JOINT_LIMITS[i][0])
            else:
                q.append(random.uniform(JOINT_LIMITS[i][0], JOINT_LIMITS[i][1]))
        return np.array(q)

    def get_nearest_node(self, q_rand):
        dists = [np.linalg.norm(node.q - q_rand) for node in self.nodes]
        return self.nodes[np.argmin(dists)]
        
    def extract_path(self, end_node):
        path = []
        curr = end_node
        while curr is not None:
            path.append(curr.q)
            curr = curr.parent
        return path[::-1] # Inverte para ser inicio -> fim

    def steer(self, from_node, q_rand):
        dir_vec = q_rand - from_node.q
        dist = np.linalg.norm(dir_vec)
        if dist > STEP_SIZE:
            dir_vec = dir_vec / dist
            new_q = from_node.q + dir_vec * STEP_SIZE
        else:
            new_q = q_rand
        return Node(new_q)

    def check_collision(self, q):
        tip_pos = fkine(q)
        tx, ty, tz = tip_pos[0], tip_pos[1], tip_pos[2]
     
        if tz < 0.0: 
            return True

        for obs in OBSTACLES_LIST:
            ox, oy = obs['x'], obs['y']
            
            if obs['type'] == 'cylinder':
                dist_xy = math.sqrt((tx - ox)**2 + (ty - oy)**2)
                if dist_xy < obs['r'] and tz < obs['h']:
                    return True

            elif obs['type'] == 'box':
                half_x = obs['size_x'] / 2.0
                half_y = obs['size_y'] / 2.0
                max_z = obs['size_z']

                x_min, x_max = ox - half_x, ox + half_x
                y_min, y_max = oy - half_y, oy + half_y
                
                if (x_min < tx < x_max) and (y_min < ty < y_max) and (tz < max_z):
                    return True

        return False
        
    def has_edge(self, q1, q2):
        steps = 15 
        for t in range(1, steps):
            ratio = t / float(steps)
            q_new = (1 - ratio)*q1 + ratio*q2
            if self.check_collision(q_new):
                return False
        return True
        
    def planning(self):
        print(f"--- RRT Iniciado ---")
        print(f"Meta XYZ: {self.goal_xyz}")
        
        # Checagem inicial
        if self.check_collision(self.start.q):
            print("ERRO CRÍTICO: Robô nasce dentro de um obstáculo ou no chão!")
            return None

        for i in range(MAX_ITER):
            if i % 1000 == 0:
                print(f"Iteração {i} | Nós: {len(self.nodes)}")

            q_rand = self.get_random_config()
            nearest = self.get_nearest_node(q_rand)
            new_node = self.steer(nearest, q_rand)
            
            if not self.check_collision(new_node.q):
                if self.has_edge(nearest.q, new_node.q):
                    new_node.parent = nearest
                    self.nodes.append(new_node)
                    current_xyz = fkine(new_node.q)
                    dist = np.linalg.norm(current_xyz - self.goal_xyz)
                    
                    if dist < GOAL_THRESHOLD:
                        print(f"SUCESSO! Caminho encontrado na iteração {i}.")
                        print(f"Posição Final Atingida: {current_xyz}")
                        return self.extract_path(new_node)
        
        print("FALHA: Não foi possível encontrar um caminho livre.")
        return None

def read_serial():
    while True:
        try:
            if ser and ser.in_waiting > 0:
                data = ser.readline().decode('utf-8', errors='ignore').strip()
                if data: print(f"[Robô]: {data}")
        except:
            break

def send_to_robot(q_rads):
    # Converte radianos -> graus
    b = math.degrees(q_rads[0])
    s = math.degrees(q_rads[1])
    e = math.degrees(q_rads[2])
    
    # JSON comando
    cmd = {
        "T": 102,
        "base": round(b, 2),
        "shoulder": round(s, 2),
        "elbow": round(e, 2),
        "hand": 90, 
        "spd": 0,
        "acc": 0
    }
    msg = json.dumps(cmd) + "\n"
    ser.write(msg.encode())


def main():
    global ser
    parser = argparse.ArgumentParser(description='RRT RoArm Control')
    parser.add_argument('port', type=str, help='Porta COM')
    parser.add_argument('--x', type=float, default=-0.20, help='Alvo X (m)')
    parser.add_argument('--y', type=float, default=0.23, help='Alvo Y (m)')
    parser.add_argument('--z', type=float, default=0.03, help='Alvo Z (m)')
    
    args = parser.parse_args()
    
    print("=== Configuração do Cenário ===")
    print(f"Alvo: X={args.x:.3f}, Y={args.y:.3f}, Z={args.z:.3f}")

    # Define inicio (zeros) e fim
    start_q = np.zeros(NUMBER_OF_JOINTS)
    goal_xyz = [args.x, args.y, args.z]
    
    # Planeja
    planner = RRT(start_q, goal_xyz)
    path = planner.planning()
    
    # --- USO DO SYS PARA ERRO DE PLANEJAMENTO ---
    if path is None:
        print("\n[ERRO CRÍTICO] O RRT não conseguiu encontrar um caminho válido.")
        print("Verifique se o alvo está dentro de um obstáculo ou fora de alcance.")
        sys.exit(1) # Sai do programa indicando erro

    # Executa
    try:
        print(f"Conectando serial na porta {args.port}...")
        ser = serial.Serial(args.port, 115200, timeout=1)
        
        t = threading.Thread(target=read_serial)
        t.daemon = True
        t.start()
        time.sleep(2)
        for i, q in enumerate(path):
            send_to_robot(q)
            time.sleep(0.2) 
        time.sleep(2)
        sys.exit(0)

    except serial.SerialException as e:
        print(f"\n[ERRO DE CONEXÃO] Não foi possível abrir a porta {args.port}.")
        print(f"Detalhes: {e}")
        sys.exit(1)
        
    except KeyboardInterrupt:
        print("\nInterrompido pelo usuário.")
        sys.exit(0)
        
    except Exception as e:
        print(f"\n[ERRO DESCONHECIDO] {e}")
        sys.exit(1)
        
    finally:
        if ser and ser.is_open:
            ser.close()

if __name__ == "__main__":
    main()