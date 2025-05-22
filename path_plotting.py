import pandas as pd
import matplotlib.pyplot as plt
import glob

def plot_all_paths(path_folder):
    files = glob.glob(f"{path_folder}/path_*.csv")
    plt.figure()
    for f in files:
        df = pd.read_csv(f)
        label = f.split("/")[-1].replace(".csv", "")
        plt.plot(df["x"], df["y"], label=label)
    plt.title("Trajektorie plannerów")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.grid(True)
    plt.axis("equal")
    plt.legend()
    plt.show()

if __name__ == "__main__":
    plot_all_paths("data/navfn/Dijkstra")
