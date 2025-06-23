#!/usr/bin/env python3

import math
import matplotlib.pyplot as plt

#--------------------------------------------------------------------------------------------------

def reformat(abs_path):
    print(abs_path)

    with open(abs_path, "r") as file:

        data_points = []
        n_valid_data = 0
        n_corrupted_data = 0

        for line in file:
            if line.startswith("data:"):   
                try:
                    split1 = line.split('"')[1]                          # Extract String from brackets
                    split2 = split1.split("\\")[0]                       # Remove \n
                    numbers = list(map(int, split2.strip().split(',')))  # Exctract numbers
                    if len(numbers) == 3:
                        data_points.append(numbers)
                        n_valid_data += 1
                    else:
                        n_corrupted_data += 1

                except (IndexError, ValueError) as e:
                    n_corrupted_data += 1

    print(f"Valid: {n_valid_data}, Corrupted: {n_corrupted_data}\n")
    return data_points

#--------------------------------------------------------------------------------------------------

def process(data_points):

    set_num = 0
    x_sum = 0
    y_sum = 0
    z_sum = 0

    for set in data_points:
        set_num = set_num + 1
        x_sum = x_sum + set[0]
        y_sum = y_sum + set[1]
        z_sum = z_sum + set[2]

    x_avg = x_sum/set_num
    y_avg = y_sum/set_num
    z_avg = z_sum/set_num
    r_avg = round(math.sqrt(x_avg**2 + y_avg**2 + z_avg**2), 2)

    x_fields.append(x_avg)
    y_fields.append(y_avg)
    z_fields.append(z_avg)
    r_fields.append(r_avg)

#--------------------------------------------------------------------------------------------------

def generate_plot(angles, xf, yf, zf, rf, plot_name):

    plt.plot(angles, rf, label='avg_r', linewidth=2)
    plt.plot(angles, xf, label='avg_x', linewidth=2)
    plt.plot(angles, yf, label='avg_y', linewidth=2)
    plt.plot(angles, zf, label='avg_z', linewidth=2)

    plt.xlabel('Angle (dec)', fontsize=16)
    plt.ylabel('Magnetic Field (uT)', fontsize=16)
    plt.xticks(fontsize=14)
    plt.yticks(fontsize=14)

    plt.title(plot_name, fontsize=22)

    plt.legend(fontsize=16)

    plt.grid(True)
    plt.axhline(0, color='black', linewidth=1.5)

#--------------------------------------------------------------------------------------------------

def show_plots():
    print("Please check which angle cooresponds to open and closed claw")
    plt.show(block=False)
    input("Press Enter to close all plots and exit...")
    plt.close('all')

#--------------------------------------------------------------------------------------------------

if __name__ == "__main__":

    #(todo later: make a script that performs full data acquisition)
    # Data files are (currently) generated from /sensor_data/sensor<SENSOR_NUMBER> using the command below

#   $ rostopic echo -n <DATA_SAMPLE> /sensor_data/sensor<SENSOR_NUMBER> > <FILE_NAME.txt>


    # THE PROGRAM EXPECTS FILES WITH THE FOLLOWING NAME FORMAT: "<ANGLE>.txt"
    # ONLY MODIFY THESE 4 (todo later: add as inputs)
    #------------------------
    angle_min = 700
    angle_max = 1340
    angle_inc = 20
    folder = "/home/luka/catkin_ws/src/ZavrsniRad/tests/claw_tests_v1/test_16_spider/data"
    test_name = "TEST 16 - SPIDER"
    #------------------------

    angles = []
    files = []
    x_fields = []
    y_fields = []
    z_fields = []
    r_fields = []

    # Generate angles[] and files[] for processing
    for angle in range(angle_min, angle_max + 1, angle_inc):
        file = (f"{angle}.txt")
        angles.append(angle)
        files.append(file)

    # Reformat data and process it
    print(f"Processing .txt-s from folder: {folder}")
    for file in files:
        file_dataset = reformat(f"{folder}/{file}")
        process(file_dataset)

    # Plot and display data
    generate_plot(angles, x_fields,  y_fields, z_fields, r_fields, test_name)
    show_plots()