#!/usr/bin/env python3

# This code creates plots from .csv files produced by claw_sweep_logger.py

import pandas as pd
import matplotlib.pyplot as plt

#--------------------------------------------------------------------------------------------------

def plot_data(filepath):

    colors = [
        '#FF4C4C',  # bright red
        '#CC0000',  # darker red
        '#4CFF88',  # bright green
        '#00CC44',  # darker green
        '#4C9AFF',  # bright blue
        '#0047CC',  # darker blue
        "#FFA600"   # orange/yellow
    ]

    df = pd.read_csv(filepath)

    # Plot design
    plt.plot(df['angle_dec'], df['|B|'], label='|B|', color = colors[6], linewidth = 2.4)
    plt.plot(df['angle_dec'], df['Bx'],  label='Bx', color = colors[1], linewidth = 2.4)
    plt.plot(df['angle_dec'], df['By'],  label='By', color = colors[3], linewidth = 2.4)
    plt.plot(df['angle_dec'], df['Bz'],  label='Bz', color = colors[5], linewidth = 2.4)

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
    file_path = " "
    #--------------------------------------------

    plot_data(file_path)