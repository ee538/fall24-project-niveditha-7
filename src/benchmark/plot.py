import pandas as pd
import matplotlib.pyplot as plt
import re

# Load the CSV file
csv_file = 'outputs/tsp_benchmark.csv'  
df = pd.read_csv(csv_file)

# Extract input size from the 'name' column
df['Input Size'] = df['name'].apply(lambda x: int(re.findall(r'/(\d+)', x)[0]))

# Convert runtime to milliseconds for better readability
df['Runtime (ms)'] = df['real_time'] / 1e6

# Plot for Brute Force and Backtracking
brute_back = df[df['name'].str.contains('BM_TSP_BruteForce|BM_TSP_Backtracking')]
plt.figure(figsize=(10, 6))
for algo in brute_back['name'].str.split('/').str[0].unique():
    subset = brute_back[brute_back['name'].str.contains(algo)]
    plt.plot(subset['Input Size'], subset['Runtime (ms)'], marker='o', label=algo)
plt.xlabel('Input Size')
plt.ylabel('Runtime (ms)')
plt.title('Benchmark: Brute Force vs Backtracking')
plt.legend()
plt.grid(True)
plt.savefig('outputs/brute_force_vs_backtracking.png')
plt.show()

# Plot for 2-opt and 3-opt
opt_algorithms = df[df['name'].str.contains('BM_TSP_2opt|BM_TSP_3opt')]
plt.figure(figsize=(10, 6))
for algo in opt_algorithms['name'].str.split('/').str[0].unique():
    subset = opt_algorithms[opt_algorithms['name'].str.contains(algo)]
    plt.plot(subset['Input Size'], subset['Runtime (ms)'], marker='o', label=algo)
plt.xlabel('Input Size')
plt.ylabel('Runtime (ms)')
plt.title('Benchmark: 2-opt vs 3-opt')
plt.legend()
plt.grid(True)
plt.savefig('outputs/2opt_vs_3opt.png')

plt.show()
