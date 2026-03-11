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

MAX_ITER = 100000 
STEP_SIZE = 0.04
GOAL_THRESHOLD = 0.06
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
        'x': 0.17, 'y': 0.07, 'z': 0.0, 
        'r': 0.025,
        'h': 0.245
    },
    {
        'type': 'box',
        'name': 'Paralelepipedo',
        'x': 0.23, 'y': 0.037, 'z': 0.0,
        'size_x': 0.06, 
        'size_y': 0.23, 
        'size_z': 0.045
    }
]

ser = None

def return_to_home():
    print("Retornando para a posição inicial (Home)...")
    # Define a configuração zero (radianos)
    home_q = [0, 0, 0, math.degrees(np.pi)]
    send_to_robot(home_q)
    time.sleep(3) # Tempo suficiente para o movimento físico terminar

def fkine_hand(angulos):
    th1 = angulos[0]
    th2 = -angulos[1] + (np.pi / 2.0) - 0.12601
    th3 = -angulos[2] + 0.12601
    
    r = l2*np.cos(th2) + l3*np.cos(th2 + th3)
    z = l1 + l2*np.sin(th2) + l3*np.sin(th2 + th3)
    x = r*np.cos(th1)
    y = r*np.sin(th1)
    
    return np.array([x, y, z])

def fkine_elbow(angulos):
    th1 = angulos[0]
    th2 = angulos[1]

    r = l2*np.cos(th2)
    z = l1 + l2*np.sin(th2)
    x = r*np.sin(th1)
    y = r*np.cos(th1)
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

        #return Node(q_rand)

    def is_point_colliding(self, pos):
        px, py, pz = pos[0], pos[1], pos[2]
        if pz < 0.0: 
            return True

        for obs in OBSTACLES_LIST:
            ox, oy = obs['x'], obs['y']
            
            if obs['type'] == 'cylinder':
                dist_xy = math.sqrt((px - ox)**2 + (py - oy)**2)
                if dist_xy < obs['r'] and pz < obs['h']:
                    return True

            elif obs['type'] == 'box':
                half_x, half_y = obs['size_x'] / 2.0, obs['size_y'] / 2.0
                if (ox - half_x < px < ox + half_x) and \
                (oy - half_y < py < oy + half_y) and \
                (pz < obs['size_z']):
                    return True
        return False

    def check_collision(self, q):
        base_pos = np.array([0, 0, 0])
        elbow_pos = fkine_elbow(q)
        hand_pos = fkine_hand(q)
        steps = 10 

        for t in range(steps + 1):
            ratio = t / float(steps)
            p_inter = (1 - ratio) * base_pos + ratio * elbow_pos
            if self.is_point_colliding(p_inter):
                return True
        for t in range(steps + 1):
            ratio = t / float(steps)
            p_inter = (1 - ratio) * elbow_pos + ratio * hand_pos
            if self.is_point_colliding(p_inter):
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
        print(f"Meta XYZ: {self.goal_xyz}")
        
        # Checagem inicial
        if self.check_collision(self.start.q):
            print("ERRO CRÍTICO: Robô nasce dentro de um obstáculo ou no chão!")
            return None

        curr_closest_node = None
        curr_closest_node_dist = 9999999999999999999

        for i in range(MAX_ITER):
            flag = 0
            if i % 1000 == 0:
                print(f"Iteração {i} | Nós: {len(self.nodes)}")

            q_rand = self.get_random_config()
            nearest = self.get_nearest_node(q_rand)
            prob = random.randint(0,100)
            if prob < 50:
                flag = 1
                if curr_closest_node:
                    nearest = curr_closest_node
            new_node = self.steer(nearest, q_rand)
            
            if not self.check_collision(new_node.q):
                if self.has_edge(nearest.q, new_node.q):
                    new_node.parent = nearest
                    self.nodes.append(new_node)
                    current_xyz = fkine_hand(new_node.q)
                    dist = np.linalg.norm(current_xyz - self.goal_xyz)
                    
                    # ---------
                    # DEBUG
                    if dist < curr_closest_node_dist:
                        curr_closest_node = new_node
                        curr_closest_node_dist = dist
                        # DEBUG
                        if flag == 1:
                            print("foi por troca")
                        print(new_node.q)
                        print(current_xyz)
                        print(dist)
                        print('-------')
                    # -------

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
    b = math.degrees(q_rads[0])
    s = math.degrees(q_rads[1])
    e = math.degrees(q_rads[2])
    h = math.degrees(np.pi)
    cmd = {
        "T": 122,
        "b": round(b, 2),
        "s": round(s, 2),
        "e": round(e, 2),
        "h": round(h, 2), 
        "spd": 50,
        "acc": 10
    }
    
    msg = json.dumps(cmd) + "\n"
    print(f"Enviando: {msg.strip()}")
    ser.write(msg.encode('utf-8'))


def main():
    global ser
    parser = argparse.ArgumentParser(description='RRT RoArm Control')
    parser.add_argument('port', type=str, help='Porta COM')
    parser.add_argument('--x', type=float, default=0.30, help='Alvo X (m)')
    parser.add_argument('--y', type=float, default=0.25, help='Alvo Y (m)')
    parser.add_argument('--z', type=float, default=0.15, help='Alvo Z (m)')
    
    args = parser.parse_args()
    print(f"Alvo: X={args.x:.3f}, Y={args.y:.3f}, Z={args.z:.3f}")

    # Define inicio (zeros) e fim
    start_q = np.zeros(NUMBER_OF_JOINTS)
    goal_xyz = [args.x, args.y, args.z]
    
    # Planeja
    planner = RRT(start_q, goal_xyz)
    path = planner.planning()
    
    if path is None:
        print("\n[ERRO CRÍTICO] O RRT não conseguiu encontrar um caminho válido.")
        print("Verifique se o alvo está dentro de um obstáculo ou fora de alcance.")
        sys.exit(1) 

    # Executa
    try:
        print(f"Conectando serial na porta {args.port}...")
        ser = serial.Serial(args.port, 115200, timeout=1)
        
        t = threading.Thread(target=read_serial)
        t.daemon = True
        t.start()
        time.sleep(2)
        return_to_home()
        for i, q in enumerate(path):
            send_to_robot(q)
            print(q)
            print(fkine_hand(q))
            print('-----------')
            #time.sleep(5) 

        print("Alvo atingido. Aguardando no destino...")
        time.sleep(2)

        return_to_home()

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