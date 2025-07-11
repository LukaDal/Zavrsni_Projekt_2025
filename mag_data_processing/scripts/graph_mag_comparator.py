#!/usr/bin/env python3

# This code creates plots that compare data from two .csv files produced by claw_sweep_logger.py

import pandas as pd
import matplotlib.pyplot as plt

#--------------------------------------------------------------------------------------------------

def plot_data(filepath1, filepath2, name1, name2):

    colors = [
        '#FF4C4C',  # bright red
        '#CC0000',  # darker red
        '#4CFF88',  # bright green
        '#00CC44',  # darker green
        '#4C9AFF',  # bright blue
        '#0047CC',  # darker blue
        "#FFA600"   # orange/yellow
    ]

    df1 = pd.read_csv(filepath1)
    df2 = pd.read_csv(filepath2)

    plt.plot(df1['angle_dec'], df1[f'Bx'], label=f'Bx - {name1}', color = colors[0], linewidth = 2.4)
    plt.plot(df2['angle_dec'], df2[f'Bx'], label=f'Bx - {name2}', color = colors[1], linewidth = 2.4)

    plt.plot(df1['angle_dec'], df1[f'By'], label=f'By - {name1}', color = colors[2], linewidth = 2.4)
    plt.plot(df2['angle_dec'], df2[f'By'], label=f'By - {name2}', color = colors[3], linewidth = 2.4)

    plt.plot(df1['angle_dec'], df1[f'Bz'], label=f'Bz - {name1}', color = colors[4], linewidth = 2.4)
    plt.plot(df2['angle_dec'], df2[f'Bz'], label=f'Bz - {name2}', color = colors[5], linewidth = 2.4)

    plt.xlabel('Kut (dec)', fontsize=18)
    plt.ylabel('Gustoća magnetskog polja (µT)', fontsize=18) # In Croatian
    plt.xticks(fontsize=15)
    plt.yticks(fontsize=15)
    plt.legend(fontsize=18)
    plt.grid(True)
    plt.axhline(0, color='black', linewidth=1.5)

    plt.show(block=False)
    input("Press Enter to close all plots and exit...")
    plt.close('all')

#--------------------------------------------------------------------------------------------------

if __name__ == "__main__":

    #-----CHANGE-PARAMETERS-HERE----------------- (TODO: Add these as some form of inputs insead of hardcoding)
    name_1 = " "
    name_2 = " "
    file_path_1 = " "
    file_path_2 = " "
    #--------------------------------------------

    plot_data(file_path_1, file_path_2, name_1, name_2)