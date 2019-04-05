import java.io.File;
import java.io.PrintWriter;

public class Velocity {
    public Velocity(Point[] coordinates, int subsection, double constantVelocity, double endRatio) throws Exception {
        File f = new File("hello.csv");
        PrintWriter writer = new PrintWriter(f);
        double[][] speeds = new double[coordinates.length*20][2];
        int lastRatio = (int)(coordinates.length*20*endRatio);
        int index = 0;
        int lastRatioIndex = 0;
        Trajectory trajectory = new Trajectory(coordinates[0].getX(), coordinates[0].getY(), 0, 5.0, 10.0, 1);
        int i = 0;
        while (i <= (coordinates.length - subsection)) {
            Point[] sub = subSection(coordinates, i, i + subsection);
            Spline spline = new Spline(sub);
            Point[] subPoints = spline.getXYSet();
            for (int j = 0; j < subPoints.length; j++) {
                double ratio = 1.0;
                if (index > (coordinates.length*20 - lastRatio)) {
                    ratio = (double)(lastRatio - lastRatioIndex)/(double)lastRatio;
                    lastRatioIndex++;
                }
                trajectory.getNextPoint(subPoints[j].getX(), subPoints[j].getY());
                trajectory.setLinVel();
                trajectory.setThetaArr();
                trajectory.setDtheta();
                trajectory.setAngVel();
                trajectory.makeArrays();
                trajectory.setDet();
                trajectory.getRatio();
                double[] wheelSpeeds = trajectory.getWheelSpeed();
                speeds[index][0] = ratio*wheelSpeeds[0];
                speeds[index][1] = ratio*wheelSpeeds[1];
                System.out.println("Left: " + speeds[index][0] + "; Right: " + speeds[index][1]);
                writer.println(subPoints[j].getX() + "," + subPoints[j].getY() + "," + speeds[index][0] + "," + speeds[index][1]);
                index++;
            }
            i += (subsection - 1);
        }
        writer.close();
    }

    private Point[] subSection(Point[] arr, int start, int end) {
        Point[] sub = new Point[end - start];
        for (int i = 0; i < (end - start); i++) {
            sub[i] = arr[start + i];
            System.out.println(i + start);
        }
        return sub;
    }

}
