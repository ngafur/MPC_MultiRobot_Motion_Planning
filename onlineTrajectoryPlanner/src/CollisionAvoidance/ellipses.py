from casadi import cos, sin, SX, DM, vcat, hcat, mtimes, sqrt


import numpy as np

def ellipses(theta1,theta2,theta3,theta4,theta5,theta6,Param):
    
    r1 = Param.Radius['r1']
    r2 = Param.Radius['r2']
    r3 = Param.Radius['r3']
    r4 = Param.Radius['r4']
    r5 = Param.Radius['r5']
    r6 = Param.Radius['r6']

    eps_r = 1.1
    rGr = Param.Radius['rGripper']

    H1l = Param.Ellipse['H1l']
    H1d = Param.Ellipse['H1d']
    H2l = Param.Ellipse['H2l']
    H2d = Param.Ellipse['H2d']
    H3l = Param.Ellipse['H3l']
    H3d = Param.Ellipse['H3d']
    H4l = Param.Ellipse['H4l']
    H4d = Param.Ellipse['H4d']
    H5l = Param.Ellipse['H5l']
    H5d = Param.Ellipse['H5d']
    H6l = Param.Ellipse['H6l']
    H6d = Param.Ellipse['H6d']

    d0 = Param.DH['d0']
    d1 = Param.DH['d1']
    d3 = Param.DH['d3']
    d4 = Param.DH['d4']
    d5 = Param.DH['d5']
    d6 = Param.DH['d6']
    a2 = Param.DH['a2']
    a3 = Param.DH['a3']

    lGripper = Param.Length['lGripper']
    
    x0 = Param.Base['Shift'][0]
    y0 = Param.Base['Shift'][1]
    z0 = Param.Base['Shift'][2]
    theta_base = Param.Base['Rotation']
    
    res = {}
    
    
    # Basisvektoren der Gelenke
    A00 = vcat([hcat([cos(theta_base), -sin(theta_base), 0, x0]),hcat([sin(theta_base), cos(theta_base), 0, y0]), hcat([0,0,1,z0]), hcat([0,0,0,1])])
    A10 = vcat([hcat([cos(theta1), 0, sin(theta1), 0]),hcat([sin(theta1), 0, -cos(theta1), 0]), hcat([0,1,0,d1]), hcat([0,0,0,1])])
    A21_0 = vcat([hcat([cos(theta2), -sin(theta2), 0, 0]),hcat([sin(theta2), cos(theta2), 0, 0]), hcat([0,0,1,d0]), hcat([0,0,0,1])])
    A21_1 = vcat([hcat([cos(theta2), -sin(theta2), 0, cos(theta2)*a2]),hcat([sin(theta2), cos(theta2), 0, sin(theta2)*a2]), hcat([0,0,1,d0]), hcat([0,0,0,1])])
    A32_0 = vcat([hcat([cos(theta3), -sin(theta3), 0, 0]),hcat([sin(theta3), cos(theta3), 0, 0]), hcat([0,0,1,-d3]), hcat([0,0,0,1])])
    A32_1 = vcat([hcat([cos(theta3), -sin(theta3), 0, cos(theta3)*a3]),hcat([sin(theta3), cos(theta3), 0, sin(theta3)*a3]), hcat([0,0,1,-d3]), hcat([0,0,0,1])])
    A43 = vcat([hcat([cos(theta4), 0, sin(theta4), 0]),hcat([sin(theta4), 0, -cos(theta4), 0]), hcat([0,1,0,d4]), hcat([0,0,0,1])])
    A54 = vcat([hcat([cos(theta5), 0, -sin(theta5), 0]),hcat([sin(theta5), 0, cos(theta5), 0]), hcat([0,-1,0,d5]), hcat([0,0,0,1])])
    A65 = vcat([hcat([cos(theta6), -sin(theta6), 0, 0]),hcat([sin(theta6), cos(theta6), 0, 0]), hcat([0,0,1,d6]), hcat([0,0,0,1])])
    
    
    # Transformationsmatrizen
    T01 = mtimes(A00,A10)
    T02 = mtimes(T01,A21_0)
    T03 = mtimes(T01,A21_1)
    T03t = mtimes(T03,A32_0)
    T04 = mtimes(T03, A32_1)
    T05 = mtimes(T04, A43)
    T06 = mtimes(T05, A54)
    T07 = mtimes(T06, A65)
    
    # Halbachsen der Ellipsoiden
    res['H1'] = np.diag([1 / (H1d / 2)**2, 1 / (H1d / 2)**2, 1 / (H1l / 2)**2])
    res['H2'] = np.diag([1 / (H2l / 2)**2, 1 / (H2d / 2)**2, 1 / (H2d / 2)**2])
    res['H3'] = np.diag([1 / (H3l / 2)**2, 1 / (H3d / 2)**2, 1 / (H3d / 2)**2])
    res['H4'] = np.diag([1 / (H4d / 2)**2, 1 / (H4l / 2)**2, 1 / (H4d / 2)**2])
    res['H5'] = np.diag([1 / (H5d / 2)**2, 1 / (H5l / 2)**2, 1 / (H5d / 2)**2])
    res['H6'] = np.diag([1 / (H6d / 2)**2, 1 / (H6d / 2)**2, 1 / (H6l / 2)**2])




    # res['H1'] = np.diag([1/(sqrt(2)*r1 + eps_r*r1)**2, 1/(sqrt(2)*r1 + eps_r*r1)**2, 1/(sqrt(2)*d1/2 + 2*r1 + 0.1*d1/2)**2])
    # res['H2'] = np.diag([1/(sqrt(2)*abs(a2)/2 + 2*r2 + 0.1*abs(a2)/2)**2, 1/(sqrt(2)*r2 + eps_r*r2)**2, 1/(sqrt(2)*r2 + eps_r*r2)**2])
    # res['H3'] = np.diag([1/(sqrt(2)*abs(a3)/2 + 2*r3 + 0.1*abs(a3)/2)**2, 1/(sqrt(2)*r3 + eps_r*r3)**2, 1/(sqrt(2)*r3 + eps_r*r3)**2])
    # res['H4'] = np.diag([1/(sqrt(2)*r3 + eps_r*r3)**2, 1/(sqrt(2)*d4/2 + 2*r3 + 0.1*d4/2)**2, 1/(sqrt(2)*r3 + eps_r*r3)**2])
    # res['H5'] = np.diag([1/(sqrt(2)*r5 + eps_r*r5)**2, 1/(sqrt(2)*d5/2 + 2*r5 + 0.1*d5/2)**2, 1/(sqrt(2)*r5 + eps_r*r5)**2]) 
    # res['H6'] = np.diag([1/(sqrt(2)*rGr+eps_r*rGr)**2, 1/(sqrt(2)*rGr+eps_r*rGr)**2, 1/(sqrt(2)*d6/2 + 2*r6 + 0.1*d6)**2])


    
    
    # Rotationsmatrizen
    res['R1'] = T01[0:3,0:3]
    res['R2'] = T03[0:3,0:3]
    res['R3'] = T04[0:3,0:3]
    res['R4'] = T05[0:3,0:3]
    res['R5'] = T06[0:3,0:3]
    res['R6'] = T07[0:3,0:3]
    
    # Gelenkkoordinaten
    B00 = Param.Base['Shift']
    B01 = T01[0:3,3]
    B02 = T02[0:3,3]
    B03 = T03[0:3,3]
    B03t = T03t[0:3,3]
    B04 = T04[0:3,3]
    B05 = T05[0:3,3]
    B06 = T06[0:3,3]
    B07 = T07[0:3,3]
    
    # Mittelpunkte der Ellipsoiden
    res['rm001'] = 0.5 * (B01 + B00)
    res['rm012'] = 0.5 * (B02 + B01)
    res['rm023'] = 0.5 * (B03 + B02)
    res['rm03t'] = 0.5 * (B03t + B03)
    res['rm03t4'] = 0.5 * (B04 + B03t)
    res['rm045'] = 0.5 * (B05 + B04)
    res['rm056'] = 0.5 * (B06 + B05)
    res['rm06t'] = 0.5 * (B07 + B06)
    
    return res
    
    
    
    
