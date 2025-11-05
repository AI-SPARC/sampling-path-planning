import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D  # importa suporte 3D
import os

# Caminhos dos arquivos (ajuste se necessário)
nodes_csv = "../data/rrt_nodes_3d.csv"
path_csv = "../data/rrt_path_3d.csv"
output_video = "../output/rrt3d_animation.mp4"

# Leitura dos dados
nodes = pd.read_csv(nodes_csv)
path = pd.read_csv(path_csv)

# Configura a figura 3D
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection="3d")
ax.set_title("Expansão da Árvore RRT 3D")
ax.set_xlabel("X (pixels)")
ax.set_ylabel("Y (pixels)")
ax.set_zlabel("Z (camada)")
ax.view_init(elev=25, azim=75)  # ângulo da câmera (pode ajustar)
ax.grid(True)

# Pega os pontos de início e objetivo
start_x, start_y, start_z = path.iloc[0]
goal_x, goal_y, goal_z = path.iloc[-1]

# adiciona os dois pontos fixos
start_dot, = ax.plot([start_x], [start_y], [start_z], "go", markersize=6, label="Início")
goal_dot, = ax.plot([goal_x], [goal_y], [goal_z], "ro", markersize=6, label="Objetivo")

# listas para guardar objetos de plot
lines = []   # conexões (arestas)
points = []  # nós (pontos azuis)

# Configura limites automáticos do gráfico
ax.set_xlim(nodes["x"].min(), nodes["x"].max())
ax.set_ylim(nodes["y"].min(), nodes["y"].max())
ax.set_zlim(nodes["z"].min(), nodes["z"].max())

# Função de atualização da animação
def update(frame):
    i = frame
    if i < len(nodes):
        node = nodes.iloc[i]
        parent_idx = node.get("parent_idx", -1)
        if parent_idx != -1 and parent_idx < len(nodes):
            parent = nodes.iloc[int(parent_idx)]
            # desenha linha entre nó e pai
            line, = ax.plot(
                [node["x"], parent["x"]],
                [node["y"], parent["y"]],
                [node["z"], parent["z"]],
                "b-", linewidth=0.5
            )
            lines.append(line)
        # desenha o nó atual
        point, = ax.plot(
            [node["x"]], [node["y"]], [node["z"]],
            "b.", markersize=3
        )
        points.append(point)
    elif i == len(nodes):
        # Desenha o caminho final em vermelho
        ax.plot(path["x"], path["y"], path["z"], "r-", linewidth=2)
    return lines + points + [start_dot, goal_dot]

# Cria a animação
ani = animation.FuncAnimation(
    fig,
    update,
    frames=len(nodes) + 30,
    interval=20,
    blit=False,
    repeat=False
)

# Salva vídeo
os.makedirs("../output", exist_ok=True)
try:
    FFMpegWriter = animation.writers["ffmpeg"]
    writer = FFMpegWriter(fps=30, codec="libx264")
    ani.save(output_video, writer=writer)
    print(f"✅ Vídeo MP4 salvo em: {output_video}")
except Exception as e:
    print(f"⚠️ FFmpeg não disponível ({e}), salvando como GIF...")
    gif_output = output_video.replace(".mp4", ".gif")
    ani.save(gif_output, writer=animation.PillowWriter(fps=30))
    print(f"✅ Animação GIF salva em: {gif_output}")

plt.close(fig)
