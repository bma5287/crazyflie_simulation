import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Slider, Button

class InteractivePlot:
    def __init__(self):
        # The parametrized function to be plotted
        self.t = np.linspace(0, 1, 1000)
        self.init_amplitude = 5
        self.init_frequency = 3

        ## SAR DIMENSIONAL CONSTRAINTS
        self.gamma = 45             # Leg Angle [m]
        self.L = 150.0e-3           # Leg Length [m]
        self.PD = 75e-3             # Prop Distance from COM [m]

        # Create the figure and the line that we will manipulate
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([0,0],[1,1], lw=2)
        self.ax.set_xlabel('Time [s]')
        self.ax.set_xlim(-1,1)
        self.ax.set_ylim(-1,1)
        self.ax.hlines(0,-5,5)

        # adjust the main plot to make room for the sliders
        self.fig.subplots_adjust(left=0.25, bottom=0.25)

        # Make a horizontal slider to control the frequency.
        axfreq = self.fig.add_axes([0.25, 0.1, 0.65, 0.03])
        self.freq_slider = Slider(
            ax=axfreq,
            label='Leg Length [m]',
            valmin=0.1,
            valmax=30,
            valinit=self.init_frequency,
            valstep=0.1
        )

        # Make a vertically oriented slider to control the amplitude
        axamp = self.fig.add_axes([0.1, 0.25, 0.0225, 0.63])
        self.amp_slider = Slider(
            ax=axamp,
            label="Gamma [deg]",
            valmin=0,
            valmax=90,
            valinit=self.init_amplitude,
            valstep=0.1,
            orientation="vertical"
        )

        # register the update function with each slider
        self.freq_slider.on_changed(self.update)
        self.amp_slider.on_changed(self.update)

        # Create a `matplotlib.widgets.Button` to reset the sliders to initial values.
        resetax = self.fig.add_axes([0.8, 0.025, 0.1, 0.04])
        self.button = Button(resetax, 'Reset', hovercolor='0.975')
        self.button.on_clicked(self.reset)

    def f(self, t, amplitude, frequency):
        return amplitude * np.sin(2 * np.pi * frequency * t)

    def update(self, val):
        self.line.set_data([0,0.5],[0,0.5])
        self.fig.canvas.draw_idle()

    def reset(self, event):
        self.freq_slider.reset()
        self.amp_slider.reset()

    def show(self):
        plt.show()


if __name__ == '__main__':
    # Create an instance of the interactive plot and show it
    plot = InteractivePlot()
    plot.show()