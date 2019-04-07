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
            self.theta = math.atan(self.dy,self.dx)

    def setLinVel(self):
        v1 = 0
        v2 = 0
        if (self.theta == math.pi/2 or self.theta == -math.pi/2):
            v1 = 0
            v2 = self.dy / Math.sin(self.theta)

        elif (self.theta == 0 or self.theta == math.pi):
            v1 = self.dx / Math.cos(self.theta)
            v2 = 0
        else:
            v1 = self.dx / Math.cos(self.theta)
            v2 = self.dy / Math.sin(self.theta)

        self.lin_vel = sqrt(v1*v1 + v2*v2)

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
        self.wheelSpeeds[0] = ratio * (1/self.det) * (self.wheelbase_inv[0][0] * self.velocity[0] + self.wheelbase_inv[0][1] * self.velocity[1])
        self.wheelSpeeds[1] = ratio * (1/self.det) * (self.wheelbase_inv[1][0] * self.velocity[0] + self.wheelbase_inv[1][1] * self.velocity[1])
        self.wheelSpeeds[0] = Math.abs(self.wheelSpeeds[0])
        self.wheelSpeeds[1] = Math.abs(self.wheelSpeeds[1])
        return self.wheelSpeeds







