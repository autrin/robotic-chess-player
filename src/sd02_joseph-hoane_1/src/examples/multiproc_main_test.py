from multiprocessing import Process, Manager
import time


def main(queue):
    while True:
        val = input(">> ")
        if val.strip().lower() == "exit":
            queue.put(None)
            break
        try:
            num = float(val)
            queue.put(num)
        except ValueError:
            pass


if __name__ == "__main__":
    from multiproc_mpl_test import run_plot

    manager = Manager()
    queue = manager.Queue()

    plot_process = Process(target=run_plot, args=(queue,))
    plot_process.start()

    main(queue)
    plot_process.join()
