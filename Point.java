public class Point {

    double x = 0.0;
    double y = 0.0;
    double far = 0.0;
    double time = 0.0;

    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public void setDistance(double distance) {
        far = distance;
    }

    public double getDistance() {
        return far;
    }

    public void setTime(double time) {
        this.time = time;
    }

    public double getTime() {
        return time;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getDistance(Point two) {
        return Math.sqrt(Math.pow((two.getX() - x), 2) + Math.pow((two.getY() - y), 2));
    }

    public double getAngle(Point two) {
        return Math.tan((two.getY() - y)/(two.getX() - x));
    }

    public String toString() {
        String ret = "X: " + x + "; Y: " + y;
        return ret;
    }

}