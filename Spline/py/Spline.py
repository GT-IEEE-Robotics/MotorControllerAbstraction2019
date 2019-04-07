import numpy as np

class Spline:
    def __init__(self, n, begin):
        self.self.points = n
        calculate(self.self.points, False, begin, 0)

    def calculate(self, n, constant, begin, end):
        estEnd = (n[len(n) - 1].getY() - n[len(n) - 2].getY())/(n[len(n) - 1].getX() - n[len(n) - 2].getX())

        d = [ [0] * 1] * len(n)
        a = [ [0] * len(n)] * len(2)

        for i in range(0, len(n)):
            if i == 0:
                h = n[i + 1].getX() - n[i].getX()
                d[i][0] = (6/h)*(n[i + 1].getY() - n[i].getY()) - 6*begin
                a[i][i] = h*2
                a[i+1][i] = h
            elif i == (len(n) - 1):
                h = n[i].getX() - n[i - 1].getX()
                d[i][0] = 6*estEnd - (6/h)*(n[i].getY() - n[i - 1].getY())
                a[i][i] = 2*h
                a[i-1][i] = h
            else:
                h_upper = n[i + 1].getX() - n[i].getX()
                h_lower = n[i].getX() - n[i - 1].getX()

                d[i][0] = (6/h_upper)*(n[i + 1].getY() - n[i].getY()) - (6/h_lower)*(n[i].getY() - n[i - 1].getY())
                a[i][i] = 2*(h_upper+h_lower)
                a[i+1][i] = h_upper
                a[i-1][i] = h_lower

        z_inv = np.multiply(np.linalg.inv(np.array(a)), d)
        self.z = [0] * len(n)
        for i in range(0, len(z_inv)):
            self.z[i] = z_inv[i][0]

    def getY(self, x, n):
        h = self.points[n + 1].getX() - self.points[n].getX()
        t_upper = self.points[n + 1].getX() - x
        t_lower = x - self.points[n].getX()
        return getA(n)*t_upper**3 + getB(n)*t_lower**3 + getC(n)*t_lower + getD(n)*t_upper

    def getA(self, n):
        h = self.points[n + 1].getX() - self.points[n].getX()
        return (self.z[n]/(6*h))

    def getB(self, n):
        h = self.points[n + 1].getX() - self.points[n].getX()
        return (self.z[n + 1]/(6*h))

    def getC(self, n):
        h = self.points[n + 1].getX() - self.points[n].getX()
        return ((self.points[n + 1].getY()/h) - ((z[n + 1]*h)/6))

    def getD(self, n):
        h = self.points[n + 1].getX() - self.points[n].getX()
        return ((self.points[n].getY()/h) - ((z[n]*h)/6))

    def getXYSet(self):
        fullSet = []
        count = 0
        for i in range(0, len(self.points)):
            diffx = self.points[i].getX() - self.points[i-1].getX()
            step = diffx/20
            for j in range(self.points[i-1].getX(), self.points[i].getX(), step):
                y = getY(j,i-1)
                fullSet.append(Point(j, y))
                count = count + 1

        return fullSet








