import pandas as pd
from matplotlib import pyplot as plt

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
    plt.plot(df['groundSteering'], color='red', label='Original groundSteering')
    # Plot our algorithm's output
    plt.plot(df['output'], color='blue', label='Our output')
    # Set the x-axis labels to sampleTimeStamp
    plt.xticks(ticks=df['sampleTimeStamp'], rotation=90)
    # Set the plot title
    plt.title("groundSteering")
    # Display the legend
    plt.legend()

    # Display the plot
    plt.show()


def compare_values():
    # Variables to keep count of all data points, and the ones that fit in the error margin
    data_points, valid = 0, 0

    # Iterate through the data frame rows
    for index, row in df.iterrows():
        groundSteering = row['groundSteering']
        output = row['output']

        # Ignore rows where the original groundSteering angle is 0
        if groundSteering != 0:
            data_points += 1

            # Calculate the error margins
            lower_bound = groundSteering * 0.75
            upper_bound = groundSteering * 1.25

            # Check if our output is within +/- 25% of the original groundSteering
            if output > lower_bound and output < upper_bound:
                valid += 1
                is_valid = True
            else:
                is_valid = False
        
            # Printing the data for debugging
            # It's expensive, so keep it commented out if not needed
            # print("groundSteering: ", groundSteering, ", output: ", output, "; ", is_valid)

    print("Percentage: ", round((valid/data_points)*100, 2), "%")


def main():
    compare_values()
    display_chart()


if __name__ == '__main__':
    try:
        # Read the .csv file or handle the exception if it deosn't exist
        df = pd.read_csv("src/output.csv", sep=',')
        main()
    except FileNotFoundError:
        print("File not found.")