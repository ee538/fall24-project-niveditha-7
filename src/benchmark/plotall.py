import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV
df = pd.read_csv("outputs/all_benchmark4.csv")

# Extract the input size 'n' from the benchmark names
df[['name', 'n']] = df['name'].str.split("/", expand=True)
df['n'] = df['n'].astype(int)  # Convert 'n' to numeric
df = df[['name', 'n', 'real_time']]  # Keep only necessary columns
# Separate Dijkstra and Bellman-Ford
dijkstra_df = df[df['name'] == "BM_Dijkstra"]
bellman_df = df[df['name'] == "BM_BellmanFord"]

# Plot Dijkstra and Bellman-Ford together
plt.figure(figsize=(10, 6))
plt.plot(dijkstra_df['n'], dijkstra_df['real_time'], label="Dijkstra", marker='o', color='blue')
plt.plot(bellman_df['n'], bellman_df['real_time'], label="Bellman-Ford", marker='s', color='orange')
plt.xlabel("Input Size (n)", fontsize=14)
plt.ylabel("Execution Time (ns)", fontsize=14)
plt.title("Benchmark Results: Dijkstra vs Bellman-Ford", fontsize=16)
plt.legend(fontsize=12)
plt.grid(True)
plt.tight_layout()
plt.savefig("dijkstra_bellman_benchmark.png")
plt.show()
