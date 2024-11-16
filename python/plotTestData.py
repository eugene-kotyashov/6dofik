import matplotlib.pyplot as plt
import numpy as np

data = np.loadtxt("outFile.txt")
fig, ax = plt.subplots(2, 1, sharex=True)
ax[0].plot(data[:, 0], data[:, 1], label='distnce')
ax[1].plot(data[:, 0], data[:, 2], label='derivative')
ax[0].legend()
ax[1].legend()
plt.show()
