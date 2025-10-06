import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.image as mpimg
import os

nodes_csv = "../data/rrt_nodes.csv"
path_csv = "../data/rrt_path.csv"
map_path = "../data/map.jpg"
output_video = "../output/rrt_animation.mp4"

nodes = pd.read_csv(nodes_csv)
path = pd.read_csv(path_csv)
map_img = mpimg.imread(map_path)

fig, ax = plt.subplots(figsize=(8, 6))
ax.imshow(map_img)
ax.set_title("Expansão da Árvore RRT")
ax.set_xlabel("Coordenada X (pixels)")
ax.set_ylabel("Coordenada Y (pixels)")
ax.axis("equal")

start_x, start_y = path["x"].iloc[0], path["y"].iloc[0]
goal_x, goal_y = path["x"].iloc[-1], path["y"].iloc[-1]

# adiciona os dois pontos fixos no início da animação
start_dot, = ax.plot(start_x, start_y, "go", markersize=8, label="Início")
goal_dot, = ax.plot(goal_x, goal_y, "ro", markersize=8, label="Objetivo")

lines = []  # para arestas
points = []  # para nós

def update(frame):
    i = frame
    if i < len(nodes):
        node = nodes.iloc[i]
        if not pd.isna(node["parent_x"]):
            line, = ax.plot([node["x"], node["parent_x"]],
                            [node["y"], node["parent_y"]],
                            "b-", linewidth=0.5)
            lines.append(line)
        point, = ax.plot(node["x"], node["y"], "b.", markersize=4)
        points.append(point)
    elif i == len(nodes):
        ax.plot(path["x"], path["y"], "r-", linewidth=2)
    return lines + points + [start_dot, goal_dot]

ani = animation.FuncAnimation(
    fig,
    update,
    frames=len(nodes) + 30,
    interval=20,
    blit=False,
    repeat=False
)

os.makedirs("../output", exist_ok=True)
try:
    FFMpegWriter = animation.writers["ffmpeg"]
    writer = FFMpegWriter(fps=30, codec="libx264")
    ani.save(output_video, writer=writer)
    print(f"✅ Vídeo MP4 salvo em: {output_video}")
except Exception:
    print("⚠️ FFmpeg não disponível, salvando como GIF...")
    gif_output = output_video.replace(".mp4", ".gif")
    ani.save(gif_output, writer=animation.PillowWriter(fps=30))
    print(f"✅ Animação GIF salva em: {gif_output}")

plt.close(fig)
