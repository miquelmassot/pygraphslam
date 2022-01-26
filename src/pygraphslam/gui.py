import atexit
import imageio
import matplotlib.pyplot as plt
import numpy as np
import time
from matplotlib.patches import Ellipse


class GraphSlamGui:
    def __init__(self, save_gif):
        plt.gcf().canvas.mpl_connect('key_release_event',
                lambda event: [exit() if event.key == 'escape' else None])
        plt.gcf().gca().set_aspect('equal')
        plt.gcf().canvas.set_window_title('float')
        plt.gcf().tight_layout(pad=0)

        self.max_x = -float('inf')
        self.max_y = -float('inf')
        self.min_x = float('inf')
        self.min_y = float('inf')

        self.save_gif = save_gif
        self.images = []
        if save_gif:
            atexit.register(
                lambda: imageio.mimsave(
                    f'./slam_{int(time.time())}.gif',
                    self.images, fps=10))

    def draw(self, traj, point_cloud):
        current_max = np.max(point_cloud, axis=0)
        current_min = np.min(point_cloud, axis=0)
        self.max_x = max(self.max_x, current_max[0])
        self.max_y = max(self.max_y, current_max[1])
        self.min_x = min(self.min_x, current_min[0])
        self.min_y = min(self.min_y, current_min[1])

        plt.cla()
        plt.axis([self.min_x, self.max_x, self.min_y, self.max_y])

        plt.plot(traj[:, 0], traj[:, 1], '-g')
        plt.plot(point_cloud[:, 0], point_cloud[:, 1], '.b', markersize=0.1)
        plt.pause(0.0001)

        if self.save_gif:
            plt.gcf().canvas.draw()
            image = np.frombuffer(plt.gcf().canvas.tostring_rgb(), dtype='uint8')
            image  = image.reshape(plt.gcf().canvas.get_width_height()[::-1] + (3,))
            self.images.append(image)