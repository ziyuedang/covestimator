# -*- coding: utf-8 -*-
"""
Created on Fri Feb 21 17:22:36 2020

Covariance estimate for results given by openMVG bundle adjustment.
One extra iteration is performed on the results, thus giving us the ability to 
compute covariance for parameters and 
@author: zdang2
"""
import sfm_IO as IO
import numpy as np
import math as m
from collections import defaultdict 
from scipy.linalg import inv
from numpy.linalg import multi_dot


root_path, extrinsics, intrinsics, coords_3d, views_meta, control_points = read_sfm('sfm.json', 'D:/Documents/Research/covestimator/Data/')

imgID = []
img_obs = []
XYZ_3D = []

for i in range(0, len(coords_3d)):
    coords_3d[i]['index_3D'] = i
    
for i in range(0, len(coords_3d)):
    pnt_3d = coords_3d[i]['observations']
    index_3d = coords_3d[i]['index_3D']  
    for img in pnt_3d:
        img.update({"index_3D": index_3d})
    img_obs.extend(pnt_3d)

n_imgs = len(views_meta)
obs_count = []

for i in range(0, n_imgs):
    obs_count.append(imgID.count(i))

# list of 2-D image observations for all images
    
merged_obs = []
for i in range(0, n_imgs):
    temp = np.array([])
    a = []
    b = 0
    for d in range(0, len(img_obs)):
        keyID = img_obs[d]['key']
        if keyID == i:
            a = img_obs[d]['value']['x']
            b = img_obs[d]['index_3D']
            c = np.array(a.append(b))
            temp = np.append(temp, c)
    merged_obs.append(temp)


            
    
# 3-D coordinates corresponding to 2-D image observations


def initParams(obs_count, extrinsics, intrinsics, n_imgs):
    
    # Number of 2D image observations
    n = 2 * sum(obs_count) 
    
    # Number of exterior parameters
    ue = 6 * n_imgs
    
    # Number of 3D object points
    uo = len(coords_3d)
    
    # Exterior design matrix
    Ae = np.empty([n, ue])
    
    # Object point design matrix
    Ao = np.empty([n, uo])
    
    # dxe, dxo
    dxe = np.ones((ue, 1))
    
    dxo = np.ones((uo, 1))
    
    # Exterior - Xc, Yc, Zc, Rotation
    XYZc = []
    rotation = []
    for i in range(0, n_imgs):
        XYZc[i] = extrinsics[i]['center']
        rotation[i] = extrinsics[i]['rotation']
        
    
        

    
    