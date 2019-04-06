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
        ArrayList<Double> prevOverlap = new ArrayList<>();
        ArrayList<Double> currOverlap = new ArrayList<>();
        ArrayList<Double> xValues = new ArrayList<>();
        ArrayList<Double> yValues = new ArrayList<>();
        boolean first = true;
        while (i < (coordinates.length - subsection)) {
            //currOverlap.add(coordinates[i].getY());
            prevOverlap = currOverlap;
            currOverlap = new ArrayList<>();
            Point[] sub = subSection(coordinates, i, i + subsection);
            Spline spline = new Spline(sub);
            Point[] subPoints = spline.getXYSet();
            int prevIndex = 0;
            for (int j = 0; j < subPoints.length; j++) {
                double ratio = 1.0;
                // if (index > (coordinates.length*20 - lastRatio)) {
                //     ratio = (double)(lastRatio - lastRatioIndex)/(double)lastRatio;
                //     lastRatioIndex++;
                // }
                double x = subPoints[j].getX();
                double y;
                if (j < subPoints.length/2) {
                    if (first) {
                        y = subPoints[j].getY();
                    } else {
                        y = (prevOverlap.get(j) + subPoints[j].getY())/2.0;
                    }
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
                } else {
                    y = subPoints[j].getY();
                    currOverlap.add(y);
                }

                index++;
            }
            System.out.println("SIZES: " + currOverlap.size() + ", " + prevOverlap.size());
            first = false;
            i += subsection/2;
            index -= subsection/2;
            prevIndex = 0;
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
