class Velocity:
    def __init__(self, coordinates, subsection, constantVelocity, endRatio):
        f = open("hello.csv", "a")
        speeds = [[]]
        lastRatio = len(coordinates)*20*endRatio
        trajectory = Trajectory(coordinates[0].getX(), coordinates[0].getY(), 0, 5.0, 10.0, 1)
        i = 0

        endPoint = coordinates[coordinates.length - 1]
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
                xValues.add(x)
                yValues.add(y)
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

            if (i > (coordinates.length - subsection) and not end):
                end = True

        for k in range(0, len(xValues)):
            f.write("%d,%d"%(xValues[k], yValues[k]))

        f.close()










