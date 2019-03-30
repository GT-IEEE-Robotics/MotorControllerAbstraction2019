import java.io.File;
import java.io.PrintWriter;

public class Test {
    public static void main(String[] args) throws Exception {
        File f = new File("hello.csv");
        PrintWriter writer = new PrintWriter(f);
        Point[] test = {new Point(0, 0), new Point(1, 3), new Point(5, 6),
                new Point(7, 10)};
        Spline spline = new Spline(test);
        Point[] points = spline.getXYSet();
        for (int i = 0; i < points.length; i++) {
            writer.println(points[i].getX() + "," + points[i].getY());
        }
        writer.close();
    }
}