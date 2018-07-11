# Write the function "Kalman Filter" that implements a ...
# multi-dimensional Kalman Filter 
from math import *

class matrix:

    #implement basic operation of matrix class
    def __init__(self,value):
        self.value = value
        self.dimx = len(value)
        self.dimy = len(value[0])
        if value == [[]]:
            self.dimx = 0
    
    def zero (self, dimx, dimy):
        #check it valid dimension
        if dimx <1 or dimy <1: 
            raise ValueError("Invalid size of matrix")
        else:
            self.dimx=dimx
            self.dimy=dimy
            self.value = [[0 for row in range(dimy)] for col in range(dimx)]

    def identity(self,dim):
        #check if valid dimension
        if dim < 1:
            raise ValueError("Invalid size of matrix")
        else:
            self.dimx = dim
            self.dimy = dim 
            self.value = [[0 for row in range(dim)] for col in range(dim)]
            for i in range(dim):
                self.value[i][i]=1

    def show(self):
        for i in range(self.dimx):
            print(self.value[i])
        print(' ')
    
    def __add__(self, other): # function (+) 2 matrix
        #check if correct dimensions
        if self.dimx != other.dimx or self.dimy != other.dimy:
            raise ValueError("Matrices must be equal dimension to add")
        else:
        # add if correct dimensions
            res = matrix([[]])
            res.zero(self.dimx, self.dimy)
            for i in range(self.dimx):
                for j in range(self.dimy):
                    res.value[i][j]=self.value[i][j] +other.value[i][j]
            return res
    def __sub__(self, other): # function (-) 2 matrix
        if self.dimx != other.dimx or self.dimy != other.dimy:
            raise ValueError("Matrices must be equal dimension to subtract")
        else:
            res = matrix([[]])
            res.zero(self.dimx, self.dimy)
            for i in range(self.dimx):
                for j in range(self.dimy):
                    res.value[i][j]=self.value[i][j] -other.value[i][j]
            return res
    def __mul__(self, other): # function (*) 2 matrix
        # check if correct dimensions
        if self.dimy != other.dimx:
            raise ValueError, "Matrices must be m*n and n*p to multiply"
        else:
            # multiply if correct dimensions
            res = matrix([[]])
            res.zero(self.dimx, other.dimy)
            for i in range(self.dimx):
                for j in range(other.dimy):
                    for k in range(self.dimy):
                        res.value[i][j] += self.value[i][k] * other.value[k][j]
            return res

    def transpose(self):
        res = matrix([[]])
        res.zero(self.dimy,self.dimx)
        for i in range(self.dimx):
            for j in range(self.dimy):
                res.value[j][i] = self.value[i][j]
        return res
    # Inverse Matrix
    def Cholesky(self, ztol=1.0e-5):
        # Computes the upper triangular Cholesky factorization 
        # a positive definite matrix.
        res = matrix([[]])
        res.zero(self.dimx,self.dimx)

        for i in range(self.dimx):
            S=sum([(res.value[k][i])**2 for k in range(i)])
            d= self.value[i][i] -S
            if abs(d) < ztol:
                res.value[i][i]=0.0
            else:
                if d<0.0:
                    raise ValueError("Matrix not positive definite")
                res.value[i][i] =sqrt(d)
            for j in range(i+1, self.dimx):
                S = sum([res.value[k][i]*res.value[k][i] for k in range(self.dimx)])
                if abs(S) <ztol:
                    S=0.0
                try:
                    res.value[i][j]=(self.value[i][j]-S)/res.value[i][i]
                except:
                    raise ValueError("Zero diagonal")
            return res
    def CholeskyInverse(self):
        #Computer inverse matrix given its Cholesky upper Triangular
        # decomposition of Matrix
        res = matrix([[]])
        res.zero(self.dimx, self.dimx)

        #Backward step for inverse.
        for j in reversed(range(self.dimx)):
            tjj = self.value[j][j]
            S = sum([self.value[j][k]*res.value[j][k] for k in range(j+1,self.dimx)])
            res.value[j][j]=1.0/tjj**2 -S/tjj
            for i in reversed(range(j)):
                res.value[j][i] = res.value[i][j]= -sum([self.value[i][k]*res.value[k][j] for k in range(i+1, self.dimx)])/self.value[i][i]
            return res
    def inverse(self):
        aux = self.Cholesky()
        res = aux.CholeskyInverse()
        return res

    def __repr__(self):
        return repr(self.value)
#############################################
# Implement Kalman Filter function
def kalman_filter(x, P):
    for n in range(len(measurements)):
        # measurement update
        y = matrix([[measurements[n]]]) - H*x
        S = H*P*H.transpose() + R
        K = P*H.transpose()*S.inverse()
        x = x + (K*y)
        P = (I-(K*H))*P

        # prediction
        x = (F*x) +u
        P = F*P*F.transpose()
        print('x=')
        x.show()
        print('P=')
        P.show()
       
    return x,P
########################################
## Testing code
measurements =[1, 2, 3]

#x = matrix([[0.], [0.]]) # intintial state (location and velocity)
x = matrix([[0.], [0.]])
P = matrix([[1000.0, 0.],[0.,1000.]]) #initial uncertainty
u = matrix([[0.],[0.]]) #external motion
F = matrix([[1.,1.],[0,1.]]) # next state function
H = matrix([[1.,0.]]) #measurement function
R = matrix([[1.]]) #measurement uncertainty
I = matrix([[1.,0.],[0.,1.]]) #identity matrix

print(kalman_filter(x,P))


