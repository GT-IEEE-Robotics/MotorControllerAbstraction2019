public class Velocity {
    public Velocity(Point[] coordinates, int subsection, double constantVelocity) {
        double[][] speeds = new double[coordinates.length*20][2];
        int lastTen = (int)(coordinates.length*20*.1);
        int index = 0;
        int lastTenIndex = 0;
        for (int i = 0; i < (coordinates.length - subsection); i += subsection) {
            Point[] sub = subSection(coordinates, i, i + subsection);
            Spline spline = new Spline(sub);
            Point[] subPoints = spline.getXYSet();
            for (int j = 0; j < subPoints.length - 1; j++) {
                double ratio = 1.0;
                if (index > (coordinates.length*20 - lastTen)) {
                    ratio = (double)(lastTen - lastTenIndex)/(double)lastTen;
                    lastTenIndex++;
                }
                Trajectory trajectory = new Trajectory(subPoints[j].getX(), subPoints[j].getY(),
                        subPoints[j + 1].getX(), subPoints[j + 1].getY(), constantVelocity);
                double[] wheelSpeeds = trajectory.getWheelSpeed();
                speeds[index][0] = ratio*wheelSpeeds[0];
                speeds[index][1] *= ratio*wheelSpeeds[1];
                index++;
            }
        }
    }

    private Point[] subSection(Point[] arr, int start, int end) {
        Point[] sub = new Point[end - start];
        for (int i = 0; i < (end - start); i++) {
            sub[i] = arr[start + i];
        }
        return sub;
    }

}
