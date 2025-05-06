import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


def run_plot(queue):
    data = [0]

    fig, ax = plt.subplots()
    line, = ax.plot([], [], 'b-')

    def update(frame):
        while not queue.empty():
            print("ds")
            val = queue.get()
            if val is None:
                plt.close(fig)
                return None
            data.append(val)
        line.set_data(range(len(data)), data)
        ax.relim()
        ax.autoscale_view()
        return line,

    ani = FuncAnimation(fig, update, interval=500, cache_frame_data=False)
    plt.show()
