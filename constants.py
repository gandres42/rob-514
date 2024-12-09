import numpy as np

aruco_position = np.array([
    [-5, 5, 0],
    [5, 5, 0],
    [5, -5, 0],
    [-5, -5, 0]
])

mtx = np.array([
    [915.25367912, 0.           , 663.44366295 ],
    [  0.        , 916.31391357 , 349.45412908 ],
    [  0.        , 0.           , 1.           ]
])
dist = np.array([[ 1.48259279e-01, -9.76755846e-01, 5.37416525e-04, -7.64305837e-04, 1.24263608e+00]])
pos = np.array([0, 0, 0])
rot = np.array([
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1]
])