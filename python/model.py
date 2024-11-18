import sympy
from sympy import *

theta, alpha, r, d = symbols('theta alpha r d')
generalTransform = Matrix(
    [
    [cos(theta), -sin(theta), 0, r],
    [sin(theta) * cos(alpha), cos(theta) * cos(alpha),  -sin(alpha), -d*sin(alpha)],
    [sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), d*cos(alpha)],
          [0, 0, 0, 1]
    ]
    )
#init_printing(use_unicode=True)
theta0, theta1, theta2, theta3, theta4, theta5, theta6 = symbols('theta0 theta1 theta2 theta3 theta4 theta5 theta6')
m0=generalTransform.subs([ (theta,theta0), (alpha,pi/2), (r,0), (d,0) ])
m1=generalTransform.subs([ (theta,theta1), (alpha,pi/2)])
m2=generalTransform.subs([ (theta,theta2), (alpha,pi/2)])
m3=generalTransform.subs([ (theta,theta3), (alpha,pi/2)])
m4=generalTransform.subs([ (theta,theta4), (alpha,pi/2)])
m5=generalTransform.subs([ (theta,theta5), (alpha,pi/2)])
m6=generalTransform.subs([ (theta,0), (alpha,0), (d,0)])
origin=Matrix([0,0,0,1])
xyz=m0*m1*m2*m3*m4*m5*m6*origin
xyzinit= xyz.subs([ 
    (theta0,0), (theta1,0), 
    (theta2,0), (theta3,0),
      (theta4,0), (theta5,0),
        (r, 3), (d,1)])
tx,ty,tz=symbols('tx ty tz')
target=Matrix([tx,ty,tz,1])
c0,c1,c2,c3,c4,c5,s0,s1,s2,s3,s4,s5 = \
    symbols('c0 c1 c2 c3 c4 c5 s0 s1 s2 s3 s4 s5') 
targetDistanceSquared=transpose(target-xyz)*(target-xyz)
sampleDist=targetDistanceSquared.subs([(tx,3),(ty,2),(tz,1),(r, 3), (d,1),
    (theta0,0), (theta1,0), 
    (theta2,0), (theta3,0),
      (theta4,0), (theta5,0)])
diffDistTheta0 = diff(targetDistanceSquared,theta0).subs(
    [
        (cos(theta0),c0), (sin(theta0), s0),
        (cos(theta1),c1), (sin(theta1), s1),
        (cos(theta2),c2), (sin(theta2), s2),
        (cos(theta3),c3), (sin(theta3), s3),
        (cos(theta4),c4), (sin(theta4), s4),
        (cos(theta5),c5), (sin(theta5), s5)
    ])
diffDistTheta1 = diff(targetDistanceSquared,theta1).subs(
    [
        (cos(theta0),c0), (sin(theta0), s0),
        (cos(theta1),c1), (sin(theta1), s1),
        (cos(theta2),c2), (sin(theta2), s2),
        (cos(theta3),c3), (sin(theta3), s3),
        (cos(theta4),c4), (sin(theta4), s4),
        (cos(theta5),c5), (sin(theta5), s5)
    ])
diffDistTheta2 = diff(targetDistanceSquared,theta2).subs(
    [
        (cos(theta0),c0), (sin(theta0), s0),
        (cos(theta1),c1), (sin(theta1), s1),
        (cos(theta2),c2), (sin(theta2), s2),
        (cos(theta3),c3), (sin(theta3), s3),
        (cos(theta4),c4), (sin(theta4), s4),
        (cos(theta5),c5), (sin(theta5), s5)
    ])
diffDistTheta3 = diff(targetDistanceSquared,theta3).subs(
    [
        (cos(theta0),c0), (sin(theta0), s0),
        (cos(theta1),c1), (sin(theta1), s1),
        (cos(theta2),c2), (sin(theta2), s2),
        (cos(theta3),c3), (sin(theta3), s3),
        (cos(theta4),c4), (sin(theta4), s4),
        (cos(theta5),c5), (sin(theta5), s5)
    ]) 
diffDistTheta4 = diff(targetDistanceSquared,theta4).subs(
    [
        (cos(theta0),c0), (sin(theta0), s0),
        (cos(theta1),c1), (sin(theta1), s1),
        (cos(theta2),c2), (sin(theta2), s2),
        (cos(theta3),c3), (sin(theta3), s3),
        (cos(theta4),c4), (sin(theta4), s4),
        (cos(theta5),c5), (sin(theta5), s5)
    ]) 
diffDistTheta5 = diff(targetDistanceSquared,theta5).subs(
    [
        (cos(theta0),c0), (sin(theta0), s0),
        (cos(theta1),c1), (sin(theta1), s1),
        (cos(theta2),c2), (sin(theta2), s2),
        (cos(theta3),c3), (sin(theta3), s3),
        (cos(theta4),c4), (sin(theta4), s4),
        (cos(theta5),c5), (sin(theta5), s5)
    ]) 
diffDistTheta6 = diff(targetDistanceSquared,theta6).subs(
    [
        (cos(theta0),c0), (sin(theta0), s0),
        (cos(theta1),c1), (sin(theta1), s1),
        (cos(theta2),c2), (sin(theta2), s2),
        (cos(theta3),c3), (sin(theta3), s3),
        (cos(theta4),c4), (sin(theta4), s4),
        (cos(theta5),c5), (sin(theta5), s5)
    ]) 