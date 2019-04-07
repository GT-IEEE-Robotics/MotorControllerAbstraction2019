import numpy as np
import math


class Point:
    def __init__(self, x_temp, y_temp):
        self.x = float(x_temp)
        self.y = float(y_temp)
    def getX(self):
        return float(self.x)
    def getY(self):
        return float(self.y)
    def getDistance(self, two):
        return float(math.sqrt((two.getX() - self.x)**2 + ((two.getY() - self.y)**2)))






class Spline:
    def __init__(self, n, begin):
        self.points = n
        self.calculate(self.points, False, begin, 0)

    def calculate(self, n, constant, begin, end):
        self.z = [0] * len(n)

        estEnd = float(float((n[len(n) - 1].getY() - n[len(n) - 2].getY()))/(n[len(n) - 1].getX() - n[len(n) - 2].getX()))

        d = [[0 for j in range(1)] for i in range(len(n))]
        a = [[0 for i in range(len(n))] for j in range(len(n))]

        for i in range(0, len(n)):
            if i == 0:
                h = float(n[i + 1].getX() - n[i].getX())
                d[i][0] = (6.0/h)*(n[i + 1].getY() - n[i].getY()) - 6.0*float(begin)
                a[i][i] = h*2.0
                a[i+1][i] = h
            elif i == (len(n) - 1):
                h = float(n[i].getX() - n[i - 1].getX())
                d[i][0] = 6.0*estEnd - (6.0/h)*(n[i].getY() - n[i - 1].getY())
                a[i][i] = 2.0*h
                a[i-1][i] = h
            else:
                h_upper = float(n[i + 1].getX() - n[i].getX())
                h_lower = float(n[i].getX() - n[i - 1].getX())

                d[i][0] = (6.0/h_upper)*(n[i + 1].getY() - n[i].getY()) - (6/h_lower)*(n[i].getY() - n[i - 1].getY())
                a[i][i] = 2*(h_upper+h_lower)
                a[i+1][i] = h_upper
                a[i-1][i] = h_lower


        try:
            z_inv = self.matmult(np.linalg.inv(a), d)
        except numpy.linalg.LinAlgError:
            print("Oh no")
            pass
        else:
            for i in range(0, len(z_inv)):
                self.z[i] = float(z_inv[i][0])


    def matmult(self, m1,m2):
        r=[]
        m=[]
        for i in range(len(m1)):
            for j in range(len(m2[0])):
                sums=0
                for k in range(len(m2)):
                    sums=sums+(m1[i][k]*m2[k][j])
                r.append(sums)
            m.append(r)
            r=[]
        return m

    def getY(self, x, n):
        x = float(x)
        h = float(self.points[n + 1].getX() - self.points[n].getX())
        t_upper = float(self.points[n + 1].getX() - x)
        t_lower = float(x - self.points[n].getX())
        value = float(self.getA(n)*t_upper**3 + self.getB(n)*t_lower**3 + self.getC(n)*t_lower + self.getD(n)*t_upper)
        return value

    def getA(self, n):
        h = float(self.points[n + 1].getX() - self.points[n].getX())
        # print("HI: %d"%self.z[n])
        return float(self.z[n]/(6.0*h))

    def getB(self, n):
        h = float(self.points[n + 1].getX() - self.points[n].getX())
        return float(self.z[n + 1]/(6*h))

    def getC(self, n):
        h = float(self.points[n + 1].getX() - self.points[n].getX())
        return float((self.points[n + 1].getY()/h) - ((self.z[n + 1]*h)/6))

    def getD(self, n):
        h = float(self.points[n + 1].getX() - self.points[n].getX())
        return float((self.points[n].getY()/h) - ((self.z[n]*h)/6))

    def getXYSet(self):
        fullSet = []
        count = 0
        for i in range(1, len(self.points)):
            diffx = float(self.points[i].getX() - self.points[i - 1].getX())
            step = float(diffx/20.0)
            j = float(self.points[i - 1].getX())
            while (j < self.points[i].getX()):
                y = self.getY(j,i - 1)
                #print("HEY: %d"%y)
                fullSet.append(Point(j, y))
                count = count + 1
                j = j + step

        return fullSet









class Trajectory:
    theta_arr = [0] * 2
    velocity = [0] * 2
    wheelbase_inv = [[0] * 2] * 2
    wheelSpeeds = [[0] * 2] * 2

    def __init__(self, old_x, old_y, init_theta_input, r_wheel, l_wheelbase, ratio_input):
        self.new_x = old_x
        self.new_y = old_y

        self.init_theta = init_theta_input
        self.theta_arr[1] = self.init_theta
        self.ratio = ratio_input
        self.r = r_wheel
        self.l = l_wheelbase

    def getNextPoint(self, new_x_input, new_y_input):
        self.curr_x = self.new_x
        self.curr_y = self.new_y
        self.new_x = new_x_input
        self.new_y = new_y_input

        self.dx = self.new_x - self.curr_x
        self.dy = self.new_y - self.curr_y
        if (self.dx == 0):
            if (self.dy == 0):
                self.theta = 0
            elif (self.dy > 0):
                self.theta = math.pi/2
            else:
                self.theta = -math.pi/2
        else:
            self.theta = math.atan(self.dy/self.dx)

    def setLinVel(self):
        v1 = 0
        v2 = 0
        if (self.theta == math.pi/2 or self.theta == -math.pi/2):
            v1 = 0
            v2 = self.dy / math.sin(self.theta)

        elif (self.theta == 0 or self.theta == math.pi):
            v1 = self.dx / math.cos(self.theta)
            v2 = 0
        else:
            v1 = self.dx / math.cos(self.theta)
            v2 = self.dy / math.sin(self.theta)

        self.lin_vel = math.sqrt(v1*v1 + v2*v2)

    def setAngVel(self):
        self.ang_vel = self.dtheta

    def setThetaArr(self):
        self.theta_arr[0] = self.theta_arr[1]
        self.theta_arr[1] = self.theta

    def setDtheta(self):
        self.dtheta = self.theta_arr[1] - self.theta_arr[0]

    def makeArrays(self):
        self.velocity[0] = self.lin_vel
        self.velocity[1] = self.ang_vel
        self.wheelbase_inv[0][0] = -self.r / (2 * self.l)
        self.wheelbase_inv[0][1] = -self.r / 2
        self.wheelbase_inv[1][0] = -self.r / (2 * self.l)
        self.wheelbase_inv[1][1] = self.r / 2

    def setDet(self):
        self.det = -(self.r*self.r)/(2*self.l)

    def getRatio(self):
        self.ratio = 2.5 / (abs(self.velocity[1]) + 0.05)

    def getWheelSpeed(self):
        self.wheelSpeeds[0] = self.ratio * (1/self.det) * (self.wheelbase_inv[0][0] * self.velocity[0] + self.wheelbase_inv[0][1] * self.velocity[1])
        self.wheelSpeeds[1] = self.ratio * (1/self.det) * (self.wheelbase_inv[1][0] * self.velocity[0] + self.wheelbase_inv[1][1] * self.velocity[1])
        self.wheelSpeeds[0] = abs(self.wheelSpeeds[0])
        self.wheelSpeeds[1] = abs(self.wheelSpeeds[1])
        return self.wheelSpeeds







class Velocity:
    def __init__(self, coordinates, subsection, constantVelocity, endRatio):
        f = open("hello.csv", "w")
        speeds = [[]]
        lastRatio = len(coordinates)*20.0*endRatio
        trajectory = Trajectory(coordinates[0].getX(), coordinates[0].getY(), 0, 5.0, 10.0, 1)
        i = 0

        endPoint = coordinates[len(coordinates) - 1]
        distance = coordinates[0].getDistance(endPoint)
        endDistance = endRatio*distance

        xValues = []
        yValues = []
        deriv = 0
        end = False
        finish = False

        while (not finish) and ((i <= (len(coordinates) - subsection)) or end):
            if end:
                sub = coordinates[i:]
                finish = True
            else:
                sub = coordinates[i:(i + subsection)]

            spline = Spline(sub, deriv)
            subPoints = spline.getXYSet()

            for j in range(0, len(subPoints)):
                ratio = 1
                tempDistance = subPoints[j].getDistance(endPoint)
                if (tempDistance < endDistance):
                    ratio = tempDistance/endDistance

                x = subPoints[j].getX()
                y = subPoints[j].getY()
                trajectory.getNextPoint(x, y)

                xValues.append(float(x))
                yValues.append(float(y))
                trajectory.setLinVel()
                trajectory.setThetaArr()
                trajectory.setDtheta()
                trajectory.setAngVel()
                trajectory.makeArrays()
                trajectory.setDet()
                trajectory.getRatio()
                wheelSpeeds = trajectory.getWheelSpeed()
                wheelSpeeds[0] = ratio*wheelSpeeds[0]
                wheelSpeeds[1] = ratio*wheelSpeeds[1]
                speeds.append(wheelSpeeds)

            i += subsection - 1

            lastX = xValues[len(xValues) - 1]
            lastY = yValues[len(yValues) - 1]
            secondLastX = xValues[len(xValues) - 20]
            secondLastY = yValues[len(yValues) - 20]
            deriv = (lastY - secondLastY)/(lastX - secondLastX)

            if (i > (len(coordinates) - subsection) and not end):
                end = True

        for k in range(0, len(xValues)):
            f.write("%f,%f\n"%(xValues[k], yValues[k]))

        f.close()







def main():
    p = Point(0,0)
    test = [
        Point(0.0, 0.0),
        Point(1.0, 3.0),
        Point(5.0, 6.0),
        Point(7.0, 10.5),
        Point(8.0, 11.0),
        Point(9.0, 12.9),
        Point(11.0, 15.6),
        Point(15.0, 16.8),
        Point(17.0, 17.0),
        Point(18.0, 19.1)
    ]
    vel = Velocity(test, 5, 10.0, .1)
    print("here!")

if __name__ == "__main__":
    main()