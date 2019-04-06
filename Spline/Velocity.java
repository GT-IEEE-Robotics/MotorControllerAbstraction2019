import java.io.File;
import java.io.PrintWriter;
import java.util.ArrayList;

public class Velocity {
    public Velocity(Point[] coordinates, int subsection, double constantVelocity, double endRatio) throws Exception {
        File f = new File("hello.csv");
        PrintWriter writer = new PrintWriter(f);
        ArrayList<Double[]> speeds = new ArrayList<>();
        int lastRatio = (int)(coordinates.length*20*endRatio);
        int index = 0;
        int lastRatioIndex = 0;
        Trajectory trajectory = new Trajectory(coordinates[0].getX(), coordinates[0].getY(), 0, 5.0, 10.0, 1);
        int i = 0;

        double endPoint = coordinates[coordinates.length - 1];
        double distance = coordinates[0].getDistance(endPoint);
        double endDistance = endRatio*distance;

        ArrayList<Double> xValues = new ArrayList<>();
        ArrayList<Double> yValues = new ArrayList<>();
        double deriv = 0;
        boolean end = false;
        boolean finish = false;
        while (!finish && (i <= (coordinates.length - subsection) || end)) {

            Point[] sub;
            if (end) {
                sub = subSection(coordinates, i, coordinates.length);
                finish = true;
            } else {
                sub = subSection(coordinates, i, i + subsection);
            }
            Spline spline = new Spline(sub, deriv);
            Point[] subPoints = spline.getXYSet();

            for (int j = 0; j < subPoints.length; j++) {
                double ratio = 1.0;
                double tempDistance = subPoints[j].getDistance(endPoint);
                if (tempDistance < endDistance) {
                    ratio = tempDistance/endDistance;
                }
                double x = subPoints[j].getX();
                double y = subPoints[j].getY();
                trajectory.getNextPoint(x, y);
                xValues.add(x);
                yValues.add(y);
                trajectory.setLinVel();
                trajectory.setThetaArr();
                trajectory.setDtheta();
                trajectory.setAngVel();
                trajectory.makeArrays();
                trajectory.setDet();
                trajectory.getRatio();
                Double[] wheelSpeeds = trajectory.getWheelSpeed();
                wheelSpeeds[0] = ratio*wheelSpeeds[0];
                wheelSpeeds[1] = ratio*wheelSpeeds[1];
                speeds.add(wheelSpeeds);
            }

            i += subsection - 1;

            double lastX = xValues.get(xValues.size() - 1);
            double lastY = yValues.get(yValues.size() - 1);
            double secondLastX = xValues.get(xValues.size() - 20);
            double secondLastY = yValues.get(yValues.size() - 20);
            deriv = (lastY - secondLastY)/(lastX - secondLastX);
            prevIndex = 0;

            if (i > (coordinates.length - subsection)) {
                end = true;
                i = coordinates.length - (coordinates.length % subsection);
                if (i >= coordinates.length - 1) {
                    finish = true;
                }
            }
            System.out.println("I: " + i);
        }

        for (int k = 0; k < xValues.size(); k++) {
            writer.println(xValues.get(k) + "," + yValues.get(k));
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
