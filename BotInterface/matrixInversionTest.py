from sympy import MatrixSymbol, Matrix

X = MatrixSymbol('X', 3, 3)
Y = MatrixSymbol('Y', 3,3)
print((X.T*X).I*Y)
print(Matrix(X))
