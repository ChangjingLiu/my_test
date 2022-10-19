import matplotlib.pyplot as plt
import numpy as np

data=np.loadtxt("a.txt")
t = np.arange(0, data.shape[0], 1)
for i in range(10):
    plt.plot(t, data[:,i],label="s"+str(i))
plt.legend(loc="upper right")

plt.show()
print(data.shape[0])
print(data)