#!/usr/bin/env python3

import numpy as np
import math

def linear_approx(time, pos_x, pos_y, heading):
    #returns velocities
    t = np.array(time)
    x_path = np.array(pos_x)
    y_path = np.array(pos_y)
    thetas = np.array(heading)
    
    dt = np.ediff1d(t)
    dx = np.ediff1d(x_path)
    dy = np.ediff1d(y_path)
    dtheta = np.ediff1d(thetas)
    ds = np.sqrt(dx * dx + dy * dy)
    # v = np.average(ds/dt)
    # w = np.average(dtheta/dt)su
    
    v = abs(sum(ds)/sum(dt))
    #w = abs((heading[-1] - heading[0])/(time[-1] - time[0]))
    w = abs(sum(dtheta)/sum(dt))

    return v, w


def circum_center(pos_x, pos_y, num_iterations = 50, eps = 1e-04):
    #returns center and radius
    n = len(pos_x)
    cen_x = []
    cen_y = []
    for _ in range(num_iterations):
        id1 = np.random.choice(range(n//3), 1)[0]
        id2 = np.random.choice(range(n//3, 2 * n//3), 1)[0]
        id3 = np.random.choice(range(2 * n//3, n), 1)[0]
        x_a, y_a = (pos_x[id1] + pos_x[id2])/2, (pos_y[id1] + pos_y[id2])/2
        x_b, y_b = (pos_x[id2] + pos_x[id3])/2, (pos_y[id2] + pos_y[id3])/2
        
        if abs(pos_y[id1] -  pos_y[id2]) <= eps:
            cen_x.append(x_a)
            m2 = (pos_x[id3] - pos_x[id2])/(pos_y[id2] - pos_y[id3])
            cen_y.append(m2 * x_a + y_b - m2 * x_b)
        
        elif abs(pos_y[id2] -  pos_y[id3]) <= eps:
            cen_x.append(x_b)
            m1 = (pos_x[id1] - pos_x[id2])/(pos_y[id2] - pos_y[id1])
            cen_y.append(m1 * x_b + y_a - m1 * x_a)

        else:
            m1 = (pos_x[id1] - pos_x[id2])/(pos_y[id2] - pos_y[id1]) 
            m2 = (pos_x[id3] - pos_x[id2])/(pos_y[id2] - pos_y[id3]) 
            cen_x.append((y_a - y_b - m1  * x_a + m2 * x_b) / (m2 - m1))
            cen_y.append(m1 * cen_x[-1] + y_a - m1 * x_a)


    x = float(sum(cen_x)/num_iterations)
    y = float(sum(cen_y)/num_iterations) 
    pos_x = np.array(pos_x)
    pos_y = np.array(pos_y)
    r = np.average(np.sqrt((pos_x - x) ** 2 + (pos_y - y) ** 2))

    return x, y, r

def get_velocities(cen, rad, pos_1, pos_2, pos_3, duration, motion , heading, time , pos_x, pos_y):
    
    #ang_vel = 1
    #lin_vel = 1
    
    sign_ang_vel = 1
    sign_lin_vel = 1
    
    ang1 = math.atan2(pos_1[1] - cen[1], pos_1[0] - cen[0]) % (2 * math.pi)
    ang2 = math.atan2(pos_2[1] - cen[1], pos_2[0] - cen[0]) % (2 * math.pi)
    ang3 = math.atan2(pos_3[1] - cen[1], pos_3[0] - cen[0]) % (2 * math.pi)
    
    S = np.array([pos_1[0]-cen[0],pos_1[1]-cen[1],0])
    M = np.array([pos_2[0]-cen[0],pos_2[1]-cen[1],0])
    E = np.array([pos_3[0]-cen[0],pos_3[1]-cen[1],0])
    khat = np.array([0,0,1])
    
    crossSM = np.cross(S,M)
    crossME = np.cross(M,E)
    
    exp1 = crossSM.dot(khat)
    exp2 = crossME.dot(khat)
    
    exp3 = S.dot(M)
    exp4 = M.dot(E)
    
    #exp3 = S[0]*M[0] + S[1]*M[1] + S[2]*M[2]
    #exp4 = M[0]*E[0] + M[1]*E[1] + M[2]*E[2]
    #exp1 = (pos_1[0] - cen[0]) * (pos_2[1] - cen[1]) - (pos_2[0] - cen[0]) * (pos_1[1] - cen[1])
    #exp2 = (pos_2[0] - cen[0]) * (pos_3[1] - cen[1]) - (pos_3[0] - cen[0]) * (pos_2[1] - cen[1])

    if exp1 < 0 and exp2 < 0:
        sign_ang_vel = -1
    #elif exp1 > 0 and exp2 < 0:
        #ang_vel = -1
    #print("norm(S)={}".format(np.linalg.norm(S)))
    #print("norm(M)={}".format(np.linalg.norm(M)))
    #print("norm(E)={}".format(np.linalg.norm(E)))
    #print("cos(exp3) = {}".format(exp3/(np.linalg.norm(S)*np.linalg.norm(M))))
    #print("cos(exp4) = {}".format(exp4/(np.linalg.norm(M)*np.linalg.norm(E))))



    cos_thetaSM = exp3/np.linalg.norm(S)*np.linalg.norm(M)
    cos_thetaME = exp4/np.linalg.norm(M)*np.linalg.norm(E)
    
    #flag = 0
    
    #if cos_thetaSM>1:
        #cos_thetaSM = 1
        #flag = 1
    
    #if cos_thetaME>1:
        #cos_thetaME = 1
        #flag = 1
    
    #ang = (math.acos(cos_thetaSM) + math.acos(cos_thetaME)) 
    #ang_vel = ang_vel*ang/duration
    #print("Angular Displacement:{}".format(ang*180/math.pi))
    
    if motion == 'backward':
        sign_lin_vel = -1
                                                                                                                                                                                                                                                                                                                                                                                        
    #ang = ang3 - ang1
    #if ang < 0 and ang_vel >= 0:
        #ang_vel = ang % (2 * math.pi) / duration
    #elif ang < 0 and ang_vel < 0:
        #ang_vel = ang / duration 
    #elif ang >= 0 and ang_vel >= 0:
        #ang_vel = ang / duration
    #else:
        #ang_vel = ang % (2 * math.pi) / (-duration)
    #if flag == 0:
    #lin_vel *= abs(rad * ang_vel)
    #else:
        #lin_vel *= (((pos_3[0] - pos_1[0])**2 + (pos_3[1] - pos_1[1])**2)**0.5)/duration
        
    lin_vel, ang_vel = linear_approx(time, pos_x, pos_y, heading)
    
    lin_vel = sign_lin_vel*lin_vel
    
    ang = math.acos(range_limiter(cos_thetaSM)) + math.acos(range_limiter(cos_thetaME))
    duration = time[-1] - time[0]
    ang_vel = ang/duration
    
    ang_vel = sign_ang_vel*ang_vel
    return lin_vel, ang_vel

def range_limiter(cos_angle):

    if(cos_angle < -1):
        cos_angle = -1
    if(cos_angle > 1):
        cos_angle = 1
    return cos_angle

def jacobian(xs, ys, beta, batch_size = 10):

    jacob = -np.ones((batch_size, 3))
    den = np.sqrt((xs - beta[0]) ** 2 + (ys - beta[1]) ** 2)
    jacob[:, 0] = (beta[0] - xs)/den
    jacob[:, 1] = (beta[1] - ys)/den
    
    return jacob



def gauss_newton(pos_x, pos_y, eps = 1e-04, batch_size = 10):

    # t = np.array(time)
    x_path = np.array(pos_x)
    y_path = np.array(pos_y)
    # thetas = np.array(heading)
    n = len(pos_x)

    beta = circum_center(x_path, y_path)
    err = np.inf
    count = 0
    while err > eps:
        count += 1
        ids = np.random.choice(range(n), batch_size)
        xs = x_path[ids]
        ys = y_path[ids]

        jacob = jacobian(xs, ys, beta)
        batch_err = np.sqrt((xs - beta[0]) ** 2 + (ys - beta[1]) ** 2) - beta[2]
        update = np.matmul(np.linalg.pinv(jacob), batch_err)
        beta -= update
        err = max(abs(np.sqrt((x_path - beta[0]) ** 2 + (y_path - beta[1]) ** 2) - beta[2]))
        # print (beta, err)
        if count > 1001:
            break
    return beta


def circle_test_gen(cen, rad, num_points, err = 1e-03):

    thetas = np.random.random(num_points) * 2 * math.pi
    pos_x = rad * np.cos(thetas) + cen[0] + np.random.random(num_points) * err
    pos_y = rad * np.sin(thetas) + cen[1] + np.random.random(num_points) * err

    return pos_x, pos_y




