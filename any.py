import numpy as np

a = np.array([1, 2, 3])

a[a<2] = 0
print(a)
