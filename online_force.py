import matplotlib.pyplot as plt
import numpy as np
import random
from matplotlib.animation import FuncAnimation
from module.six_force_read import *  # Assuming this is the module for reading force data

class ForceVisualizer:
    def __init__(self, com_port="com4", time_interval=0.1, max_time=10):
        # Initialize serial port and force data lists
        self.ser = open_serial_port(com_port)
        self.force_data = []
        self.time_data = []

        # Set up plotting window
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], lw=2)
        self.ax.set_xlim(0, max_time)  # Set initial time window
        self.ax.set_ylim(-3, 3)  # Set force data range
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Force (N)')
        self.ax.set_title('Real-time Force Data')

        # Time interval between data updates (in seconds)
        self.time_interval = time_interval

    def new_force(self):
        """ Interface to return the latest force data from the sensor. """
        # You can replace this with actual force reading logic
        fx, fy, fz, mx, my, mz = six_force_read(self.ser)
        return fz  # Assuming fz is the force in z-axis

    def init_plot(self):
        """ Initialize the plot with no data. """
        self.line.set_data([], [])
        return self.line,

    def update_plot(self, frame):
        """ Update the plot with new force data. """
        new_force = self.new_force()  # Get the latest force data
        current_time = len(self.time_data) * self.time_interval  # Calculate current time based on data length

        self.force_data.append(new_force)
        self.time_data.append(current_time)

        # Update x-axis range if needed
        if current_time > self.ax.get_xlim()[1]:
            self.ax.set_xlim(0, current_time + 1)
            self.fig.canvas.draw()

        # Update the line plot with the new data
        self.line.set_data(self.time_data, self.force_data)
        return self.line,

    def start_animation(self, output_file="force_animation.mp4", fps=10):
        """ Start the real-time plotting and save the animation. """
        ani = FuncAnimation(
            self.fig,
            self.update_plot,
            init_func=self.init_plot,
            blit=True,
            interval=int(self.time_interval * 1000),  # Interval in ms
            save_count=200
        )

        ani.save(output_file, fps=fps, extra_args=['-vcodec', 'libx264'])
        plt.show()

# Usage
if __name__ == "__main__":
    visualizer = ForceVisualizer(com_port="com4", time_interval=0.1)
    visualizer.start_animation(output_file="force_animation.mp4", fps=10)
