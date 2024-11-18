import serial
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Function to check Pico connection and return data
def get_quaternion_data(port='/dev/ttyACM0', baudrate=115200, timeout=1):
    try:
        ser = serial.Serial(port, baudrate, timeout=timeout)
        time.sleep(2)  # Wait for the connection to stabilize
        print("Pico is connected")
        
        # This will collect quaternion data
        quaternions = []
        
        while True:
            # Read line from serial, decode and strip the newline
            line = ser.readline().decode('utf-8').strip()
            if line:
                q_values = line.split(',')
                if len(q_values) == 4:
                    q0, q1, q2, q3 = map(float, q_values)
                    quaternions.append((q0, q1, q2, q3))
                    yield quaternions  # Generator that yields the updated quaternion data

    except serial.SerialException:
        print("Not connected")

# Real-time plot update function
def update_plot(frame, quaternions, q0_line, q1_line, q2_line, q3_line):
    # Get the most recent quaternion data
    if len(quaternions) > 0:
        q0, q1, q2, q3 = quaternions[-1]
        
        # Update the data for each line (q0, q1, q2, q3)
        q0_line.set_data(range(len(quaternions)), [q[0] for q in quaternions])
        q1_line.set_data(range(len(quaternions)), [q[1] for q in quaternions])
        q2_line.set_data(range(len(quaternions)), [q[2] for q in quaternions])
        q3_line.set_data(range(len(quaternions)), [q[3] for q in quaternions])

    return q0_line, q1_line, q2_line, q3_line

# Main function to start the real-time plot
def main():
    # Set up the serial connection generator
    quaternion_data_gen = get_quaternion_data()

    # Set up the real-time plot
    fig, ax = plt.subplots(figsize=(10, 6))
    ax.set_xlim(0, 100)  # X-axis limits, adjust as needed
    ax.set_ylim(-1, 1)   # Y-axis limits for quaternion values, [-1, 1] for normalized quaternions
    ax.set_xlabel('Time (samples)')
    ax.set_ylabel('Quaternion Components')

    # Plot lines for each quaternion component
    q0_line, = ax.plot([], [], label='q0', color='r')
    q1_line, = ax.plot([], [], label='q1', color='g')
    q2_line, = ax.plot([], [], label='q2', color='b')
    q3_line, = ax.plot([], [], label='q3', color='y')

    # Add legend
    ax.legend(loc='upper right')

    # Animation function to update the plot
    ani = FuncAnimation(fig, update_plot, fargs=(quaternion_data_gen, q0_line, q1_line, q2_line, q3_line),
                        interval=100, blit=False)  # Update every 100 ms

    # Show the plot
    plt.show()

if __name__ == '__main__':
    main()
