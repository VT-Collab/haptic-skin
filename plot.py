import numpy as np
import matplotlib.pyplot as plt


gm = [5.5, 5.1, 3.9, 5.4, 5.2, 4.7]
gs = np.asarray([1.312334646, 1.646545205, 1.542004467, 1.61245155, 1.843908891, 2.110818693])/np.sqrt(10)
tm = [4.6, 4.65, 4.45, 5.5, 4.55, 4.4]
ts = np.asarray([1.663329993, 2.147996483, 2.033743128, 1.398411798, 1.935774321, 1.95505044])/np.sqrt(10)
rm = [6.2, 6.2, 6.35, 5.623443556, 6.1, 6.5]
rs = np.asarray([0.888194173, 1.110555417, 0.6258327785, 1.048808848, 0.6582805886, 0.8498365856])/np.sqrt(10)

x = np.asarray([1, 2, 3, 4, 5, 6])
plt.bar(x-0.25, gm, yerr=gs, width=0.25)
plt.bar(x, tm, yerr=ts, width=0.25)
plt.bar(x+0.25, rm, yerr=rs, width=0.25)
plt.show()
