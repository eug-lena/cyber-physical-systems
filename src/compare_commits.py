import pandas as pd
from matplotlib import pyplot as plt
import sys

def display_chart():
    # Set the plot size and layout
    plt.rcParams["figure.figsize"] = [13.00, 5.0]
    plt.rcParams["figure.autolayout"] = True
    fig, ax = plt.subplots()

    # Add the watermark
    ax.text(0.5, 0.5, 'Group 18', transform=ax.transAxes,
            fontsize=40, color='green', alpha=0.5,
            ha='center', va='center', rotation=0)
    # Plot original groundSteering data
    plt.plot(df_original['groundSteering'], color='red', label='Original groundSteering')
    # Plot our algorithm's current output
    plt.plot(df_current['output'], color='blue', label='Current commit output')
    # Plot the output from the previous commit
    plt.plot(df_previous['output'], color='green', label='Previous commit output', alpha=0.5)

    # Set the x-axis labels to sampleTimeStamp
    # It makes the chart a bit unreadable, but it's in the requirements
    plt.xticks(ticks=df_current['sampleTimeStamp'], rotation=90)

    # Set the plot title
    plt.title("groundSteering")
    # Set the x-axis label
    plt.xlabel("sampleTimestamp")
    # Set the y-axis label
    plt.ylabel("groundSteering angle")
    # Display the legend
    plt.legend()

    # Display the plot
    plt.savefig(f'plot_{sys.argv[1]}.png')


def compare_values():
    # Variables to keep count of all data points, and the ones that fit in the error margin
    data_points, valid = 0, 0

    # Iterate through the data frame rows
    for index, row in df_current.iterrows():
        groundSteering = row['groundSteering']
        output = row['output']

        # Ignore rows where the original groundSteering angle is 0
        if groundSteering != 0:
            data_points += 1

            # Calculate the error margins
            lower_bound = groundSteering * 0.75
            upper_bound = groundSteering * 1.25

            # Check if our output is within +/- 25% of the original groundSteering
            if output >= 0:
                if output > lower_bound and output < upper_bound:
                    valid += 1
                    is_valid = True
                else:
                    is_valid = False
            else:
                if output < lower_bound and output > upper_bound:
                    valid += 1
                    is_valid = True
                else:
                    is_valid = False
        
            # Printing the data for debugging
            # It's expensive, so keep it commented out if not needed
            # print("groundSteering: ", groundSteering, ", output: ", output, "; ", is_valid)

    print("Percentage: ", round((valid/data_points)*100, 2), "%")


def main():
    # compare_values()
    display_chart()


if __name__ == '__main__':
    try:
        # Read the .csv file or handle the exception if it doesn't exist
        df_original = pd.read_csv(f'../recordings/original{sys.argv[1]}.csv', sep=';')
        df_current = pd.read_csv(f'../recordings/current{sys.argv[1]}.csv', sep=';')
        df_previous = pd.read_csv(f'../recordings/previous{sys.argv[1]}.csv', sep=';')
        main()
    except FileNotFoundError:
        print("File not found.")