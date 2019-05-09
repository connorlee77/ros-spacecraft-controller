import numpy as np

class Dynamics:
    
    def __init__(self, L=1, b=1, m=1, I=1):
        self.L = L
        self.b = b
        self.m = m
        self.I = I

        self.H = np.array([
            [-1, -1, 0, 0, 1, 1, 0, 0], 
            [0, 0, -1, -1, 0, 0, 1, 1],
            [-L, L, -b, b, -L, L, -b, b]])

    # theta - in radians
    # u - 3x3 control input
    def output(self, u, theta):
        H = self.H
        m = self.m
        I = self.I

        T = np.array([
            [np.cos(theta)/m, np.sin(theta)/m, 0],
            [-np.sin(theta)/m, np.cos(theta)/m, 0],
            [0, 0, 1/(2.0*I)]]);

        F = np.linalg.pinv(H).dot(np.linalg.inv(T)).dot(u);

        return T.dot(H).dot(F)

def test():
    dynamics = Dynamics()
    dynamics.output(np.eye(3), np.pi/3)

if __name__ == "__main__":
    test()

