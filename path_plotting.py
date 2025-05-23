import pandas as pd
import matplotlib.pyplot as plt
import glob
import os
import re

def extract_goal_from_folder(path_folder):
    folder_name = os.path.basename(path_folder)
    match = re.search(r"plan_([-+]?[0-9]*\.?[0-9]+)_([-+]?[0-9]*\.?[0-9]+)", folder_name)
    if match:
        goal_x = float(match.group(1))
        goal_y = float(match.group(2))
        return goal_x, goal_y
    else:
        raise ValueError("Nie można odczytać celu z nazwy folderu.")

def plot_all_paths(path_folder):
    goal_x, goal_y = extract_goal_from_folder(path_folder)

    files = glob.glob(f"{path_folder}/path_*.csv")
    plt.figure()
    for f in files:
        df = pd.read_csv(f)
        filename = os.path.basename(f).replace(".csv", "")
        match = re.match(r"path_(.*)_[-+]?[0-9]*\.?[0-9]+_[-+]?[0-9]*\.?[0-9]+", filename)
        label = match.group(1) if match else filename
        plt.plot(df["x"], df["y"], label=label.title())

    # Dodaj punkt docelowy jako kropkę
    plt.plot(goal_x, goal_y, 'ko', label="Cel (%.2f, %.2f)" % (goal_x, goal_y))

    plt.plot(0, 0, marker='o',linestyle='None', color='mediumblue', label="Start (0.0, 0.0)")

    plt.title(f"Trajektorie planerów")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.grid(True)
    plt.axis("equal")
    plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
    plt.tight_layout()


    # Zapisz wykres do pliku PNG
    out_file = os.path.join(path_folder, f"plot_{goal_x}_{goal_y}.png")
    plt.savefig(out_file, dpi=300)
    print(f"[✔] Zapisano wykres do: {out_file}")

    plt.show()

if __name__ == "__main__":
    points_dir_names = ['plan_2.22_6.36', 
                        'plan_5.48_11.06', 
                        'plan_-11.46_2.43'
                        ]
    
    
    plot_all_paths("data/planners_compare/" + points_dir_names[2])
