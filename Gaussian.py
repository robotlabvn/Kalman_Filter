import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np

def f(mu, sigma2,x):
    return 1/np.sqrt(2.*3.14*sigma2)*np.exp(-.5*(x-mu)**2/sigma2)
print f(10.,4.0,10.)