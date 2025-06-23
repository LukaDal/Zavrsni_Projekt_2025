#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt

#--------------------------------------------------------------------------------------------------

def graph(filepath, plottitle):

    colors = [
        '#FF4C4C',  # bright red
        '#CC0000',  # darker red
        '#4CFF88',  # bright green
        '#00CC44',  # darker green
        '#4C9AFF',  # bright blue
        '#0047CC',  # darker blue
        '#FFFF00'   # yellow
    ]

    df = pd.read_csv(filepath)

    plt.plot(df['angle_dec'], df['avg_r'], label='avg_r', color = colors[6], linewidth=1.8)
    plt.plot(df['angle_dec'], df['avg_x'], label='avg_x', color = colors[1], linewidth=1.8)
    plt.plot(df['angle_dec'], df['avg_y'], label='avg_y', color = colors[3], linewidth=1.8)
    plt.plot(df['angle_dec'], df['avg_z'], label='avg_z', color = colors[5], linewidth=1.8)

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

    file_path = "/home/luka/catkin_ws/src/ZavrsniRad/tests/claw_tests_v2/test_21_triangle/triangle.csv"
    plot_title = "TEST 21 - TRIANGLE"

    graph(file_path, plot_title)