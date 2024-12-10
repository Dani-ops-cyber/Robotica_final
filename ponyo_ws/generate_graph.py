import subprocess

try:
    # Captura la lista de nodos
    nodes_output = subprocess.check_output(["ros2", "node", "list"]).decode().strip()
    nodes = nodes_output.split("\n") if nodes_output else []

    if not nodes:
        print("No se encontraron nodos activos en el sistema ROS 2.")
        exit(1)

    edges = []  # Lista para almacenar las conexiones

    # Captura las conexiones entre nodos y tópicos
    for node in nodes:
        if node:  # Ignorar nodos vacíos
            try:
                node_info = subprocess.check_output(["ros2", "node", "info", node]).decode()
                lines = node_info.split("\n")
                for line in lines:
                    if "Subscription" in line or "Publisher" in line:
                        topic = line.split(":")[-1].strip()
                        edges.append(f'"{node}" -> "{topic}";')
            except subprocess.CalledProcessError as e:
                print(f"Error al procesar el nodo {node}: {e}")

    # Genera el archivo .dot
    dot_file_path = "/home/jetson/graph.dot"  # Guardar el archivo en /home/jetson
    with open(dot_file_path, "w") as f:
        f.write("digraph rosgraph {\n")
        for edge in edges:
            f.write(f"    {edge}\n")
        f.write("}\n")

    print(f"Grafo guardado en: {dot_file_path}")

except Exception as e:
    print(f"Error general: {e}")

