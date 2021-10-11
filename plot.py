import numpy as np
import matplotlib.pyplot as plt
import pickle
import sys

### CODE FOR THE LIKERT SCALE PLOTS

# gm = [5.5, 5.1, 3.9, 5.4, 5.2, 4.7]
# gs = np.asarray([1.312334646, 1.646545205, 1.542004467, 1.61245155, 1.843908891, 2.110818693])/np.sqrt(10)
# tm = [4.6, 4.65, 4.45, 5.5, 4.55, 4.4]
# ts = np.asarray([1.663329993, 2.147996483, 2.033743128, 1.398411798, 1.935774321, 1.95505044])/np.sqrt(10)
# rm = [6.2, 6.2, 6.35, 5.623443556, 6.1, 6.5]
# rs = np.asarray([0.888194173, 1.110555417, 0.6258327785, 1.048808848, 0.6582805886, 0.8498365856])/np.sqrt(10)
#
# x = np.asarray([1, 2, 3, 4, 5, 6])
# plt.bar(x-0.25, gm, yerr=gs, width=0.25)
# plt.bar(x, tm, yerr=ts, width=0.25)
# plt.bar(x+0.25, rm, yerr=rs, width=0.25)
# plt.show()



### CODE FOR THE OBJECTIVE PLOTS

SEGMENT ={(1,1,'gui'):1,(1,2,'gui'):1,(1,3,'gui'):3,
    (1,1,'table'):1,(1,2,'table'):3,(1,3,'table'):1,
    (1,1,'robot'):3,(1,2,'robot'):1,(1,3,'robot'):1,

    (2,1,'gui'):1,(2,2,'gui'):3,(2,3,'gui'):1,
    (2,1,'table'):3,(2,2,'table'):3,(2,3,'table'):1,
    (2,1,'robot'):1,(2,2,'robot'):1,(2,3,'robot'):3,

    (3,1,'gui'):1,(3,2,'gui'):3,(3,3,'gui'):1,
    (3,1,'table'):3,(3,2,'table'):1,(3,3,'table'):3,
    (3,1,'robot'):3,(3,2,'robot'):3,(3,3,'robot'):1,

    (4,1,'gui'):1,(4,2,'gui'):1,(4,3,'gui'):3,
    (4,1,'table'):1,(4,2,'table'):3,(4,3,'table'):1,
    (4,1,'robot'):3,(4,2,'robot'):1,(4,3,'robot'):1,

    (5,1,'gui'):1,(5,2,'gui'):3,(5,3,'gui'):1,
    (5,1,'table'):3,(5,2,'table'):3,(5,3,'table'):1,
    (5,1,'robot'):1,(5,2,'robot'):1,(5,3,'robot'):3,

    (6,1,'gui'):1,(6,2,'gui'):3,(6,3,'gui'):1,
    (6,1,'table'):3,(6,2,'table'):1,(6,3,'table'):3,
    (6,1,'robot'):3,(6,2,'robot'):3,(6,3,'robot'):1,

    (7,1,'gui'):1,(7,2,'gui'):1,(7,3,'gui'):3,
    (7,1,'table'):1,(7,2,'table'):3,(7,3,'table'):1,
    (7,1,'robot'):3,(7,2,'robot'):1,(7,3,'robot'):1,

    (8,1,'gui'):1,(8,2,'gui'):3,(8,3,'gui'):1,
    (8,1,'table'):3,(8,2,'table'):3,(8,3,'table'):1,
    (8,1,'robot'):1,(8,2,'robot'):1,(8,3,'robot'):3,

    (9,1,'gui'):1,(9,2,'gui'):3,(9,3,'gui'):1,
    (9,1,'table'):3,(9,2,'table'):1,(9,3,'table'):3,
    (9,1,'robot'):3,(9,2,'robot'):3,(9,3,'robot'):1,

    (10,1,'gui'):1,(10,2,'gui'):1,(10,3,'gui'):3,
    (10,1,'table'):1,(10,2,'table'):3,(10,3,'table'):1,
    (10,1,'robot'):3,(10,2,'robot'):1,(10,3,'robot'):1,}

def get_len(user, method, task, trial):
    file = "demos/user" + str(user) + "/" + method + "/task" + str(task) + "_trial" + str(trial) + ".pkl"
    with open(file, "rb") as f:
            traj = pickle.load(f, encoding="latin1")
            return len(traj)

def total_times(method):
    trial = 2
    lengths = []
    for user in range(4, 14):
        for task in range(1, 4):
            lengths.append(get_len(user, method, task, trial))
    return np.asarray(lengths) * 0.05

def time_metric():
    gl = total_times("gui")
    gt = total_times("table")
    gr = total_times("robot")
    mgl = np.mean(gl)
    mgt = np.mean(gt)
    mgr = np.mean(gr)
    sgl = np.std(gl) / np.sqrt(len(gl))
    sgt = np.std(gt) / np.sqrt(len(gt))
    sgr = np.std(gr) / np.sqrt(len(gr))
    return [mgl, mgt, mgr, sgl, sgt, sgr]

def get_xyz(user, method, task, trial):
    file = "demos/user" + str(user) + "/" + method + "/task" + str(task) + "_trial" + str(trial) + ".pkl"
    ee_path = []
    with open(file, "rb") as f:
            traj = pickle.load(f, encoding="latin1")
            for state in traj:
                pose = state[1]
                xyz = pose[:3,3]
                ee_path.append([xyz[0], xyz[1], xyz[2]])
            return np.asarray(ee_path)

def classify_xyz(task, xyz):
    x = xyz[0]
    y = xyz[1]
    z = xyz[2]
    if task == 1:
        if x > 0.055:
            return 1
        elif x <= 0.055 and x >= -0.140:
            return 2
        else:
            return 3
    elif task == 2:
        if x > 0.06:
            return 1
        elif x <= 0.06 and x >= -0.100:
            return 2
        else:
            return 3
    elif task == 3:
        if x > -0.060:
            return 1
        elif x < -0.060 and y > 0.7:
            return 2
        else:
            return 3

def classify_traj(user, method, task, trial):
    ee_path = get_xyz(user, method, task, trial)
    ee_label = []
    segment_counts = [0, 0, 0]
    for xyz in ee_path:
        segment = classify_xyz(task, xyz)
        ee_label.append([xyz[0], xyz[1], xyz[2], segment])
        segment_counts[segment - 1] += 1
    return segment_counts#, ee_label

def total_segments(method):
    trial = 2
    percentage_correct = []
    for user in range(4, 14):
        for task in range(1, 4):
            correct_segment = SEGMENT[(user-3, task, method)] - 1
            segment_counts = classify_traj(user, method, task, trial)
            correct = segment_counts[correct_segment]
            percentage_correct.append(correct * 1.0 / sum(segment_counts))
    return np.asarray(percentage_correct)

def segment_metric():
    gc = total_segments("gui")
    tc = total_segments("table")
    rc = total_segments("robot")
    mgc = np.mean(gc)
    mtc = np.mean(tc)
    mrc = np.mean(rc)
    sgc = np.std(gc) / np.sqrt(len(gc))
    stc = np.std(tc) / np.sqrt(len(tc))
    src = np.std(rc) / np.sqrt(len(rc))
    return [mgc, mtc, mrc, sgc, stc, src]

def uncertainty_metric():
    U = pickle.load(open("demos/uncertainty.pkl", "rb"))
    g = U[0]
    t = U[1]
    r = U[2]
    mg = np.mean(g)
    mt = np.mean(t)
    mr = np.mean(r)
    sg = np.std(g) / np.sqrt(len(g))
    st = np.std(t) / np.sqrt(len(t))
    sr = np.std(r) / np.sqrt(len(r))
    return [mg, mt, mr, sg, st, sr]



t = time_metric()
print(t)
x = [1, 2, 3]
plt.bar(x, t[:3], yerr=t[3:])
plt.show()

t = segment_metric()
print(t)
x = [1, 2, 3]
plt.bar(x, t[:3], yerr=t[3:])
plt.show()

t = uncertainty_metric()
print(t)
x = [1, 2, 3]
plt.bar(x, t[:3], yerr=t[3:])
plt.show()
