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
init_printing(use_unicode=True)
print("hello")
m1=Matrix(
    [[cos(theta), -sin(theta), 0, r],
      [0, 0, -1, -d],
        [sin(theta), cos(theta), 0, 0],
          [0, 0, 0, 1]]
    )
str(m1)