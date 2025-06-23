#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt



#--------------------------------------------------------------------------------------------------

def graph(filepath1, filepath2, name1, name2):

    colors = [
        '#FF4C4C',  # bright red
        '#CC0000',  # darker red
        '#4CFF88',  # bright green
        '#00CC44',  # darker green
        '#4C9AFF',  # bright blue
        '#0047CC',  # darker blue

    ]
    plottitle = (f"{name1} vs {name2}").upper()

    df1 = pd.read_csv(filepath1)
    df2 = pd.read_csv(filepath2)

    plt.plot(df1['angle_dec'], df1[f'avg_x'], label=f'avg_x {name1}', color = colors[0], linewidth=1.8)
    plt.plot(df2['angle_dec'], df2[f'avg_x'], label=f'avg_x {name2}', color = colors[1], linewidth=1.8)

    plt.plot(df1['angle_dec'], df1[f'avg_y'], label=f'avg_y {name1}', color = colors[2], linewidth=1.8)
    plt.plot(df2['angle_dec'], df2[f'avg_y'], label=f'avg_y {name2}', color = colors[3], linewidth=1.8)

    plt.plot(df1['angle_dec'], df1[f'avg_z'], label=f'avg_z {name1}', color = colors[4], linewidth=1.8)
    plt.plot(df2['angle_dec'], df2[f'avg_z'], label=f'avg_z {name2}', color = colors[5], linewidth=1.8)

    plt.xlabel('Angle (dec)', fontsize=14)
    plt.ylabel('Magnetic Field (uT)', fontsize=14)
    plt.xticks(fontsize=12)
    plt.yticks(fontsize=12)

    plt.title(plottitle, fontsize=22)

    plt.legend(fontsize=14)
    
    plt.grid(True)
    plt.axhline(0, color='black', linewidth=1.5)

    plt.show(block=False)
    input("Press Enter to close all plots and exit...")
    plt.close('all')

#--------------------------------------------------------------------------------------------------

if __name__ == "__main__":

    name_1 = "empty"
    name_2 = "triangle"
    file_path_1 = f"/home/luka/catkin_ws/src/ZavrsniRad/tests/claw_tests_v2/test_18_empty/empty.csv"
    file_path_2 = f"/home/luka/catkin_ws/src/ZavrsniRad/tests/claw_tests_v2/test_21_triangle/triangle.csv"

    graph(file_path_1, file_path_2, name_1, name_2)