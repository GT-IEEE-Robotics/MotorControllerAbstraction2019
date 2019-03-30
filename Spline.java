public class Spline {

    double[] z;
    Point[] points;
    Point target;
    double tolerance = .0001;

    public Spline(Point[] n, double begin, double end, Point target) {
        this.target = target;
        points = n;
        for (int i = 0; i < n.length; i++) {
            n[i].setDistance(n[i].getDistance(target));
        }
        calculate(points, false, begin, end);
    }

    public Spline(Point[] n, Point target) {
        this.target = target;
        points = n;
        for (int i = 0; i < n.length; i++) {
            n[i].setDistance(n[i].getDistance(target));
        }
        calculate(points, true, 0, 0);
    }

    public Spline(Point[] n) {
        points = n;
        calculate(points, true, 0, 0);
    }

    public void calculate(Point[] n, boolean constant, double begin, double end) {
        z = new double[n.length];

        double[][] d = new double[n.length][n.length];
        double[][] a = new double[n.length][n.length];

        double estBegin;
        double estEnd;

        if (constant) {
            estBegin = (n[1].getY() - n[0].getY())/(n[1].getX() - n[0].getX());
            estEnd = (n[n.length - 1].getY() - n[n.length - 2]
                .getY())/(n[n.length - 1].getX() - n[n.length - 2].getX());
        } else {
            estBegin = begin;
            estEnd = end;
        }

        for (int i = 0; i < n.length; i++) {
            if (i == 0) {
                double h = n[i + 1].getX() - n[i].getX();
                d[i][i] = (6/h)*(n[i + 1].getY() - n[i].getY()) - 6*begin;
                a[i][i] = h*2;
                a[i+1][i] = h;
            } else if (i == (n.length - 1)) {
                double h = n[i].getX() - n[i - 1].getX();
                d[i][i] = 6*end - (6/h)*(n[i].getY() - n[i - 1].getY());
                a[i][i] = 2*h;
                a[i-1][i] = h;
            } else {
                double h_upper = n[i + 1].getX() - n[i].getX();
                double h_lower = n[i].getX() - n[i - 1].getX();
                d[i][i] = (6/h_upper)*(n[i + 1].getY() - n[i].getY()) - (6/h_lower)*(n[i].getY() - n[i - 1].getY());
                a[i][i] = 2*(h_upper+h_lower);
                a[i+1][i] = h_upper;
                a[i-1][i] = h_lower;
            }
            double[][] dinv = Matrix.multiply(a, Matrix.inverse(d));
            z = dinv[0];
        }

    }

    public double getY(double x, int n) {
        double h = points[n + 1].getX() - points[n].getX();
        double t_upper = points[n + 1].getX() - x;
        double t_lower = x - points[n].getX();
        return getA(n)*Math.pow(t_upper, 3) + getB(n)*Math.pow(t_lower, 3)
         + getC(n)*t_lower + getD(n)*t_upper;
    }

    public double getA(int n) {
        double h = points[n + 1].getX() - points[n].getX();
        return (z[n]/(6*h));
    }

    public double getB(int n) {
        double h = points[n + 1].getX() - points[n].getX();
        return (z[n + 1]/(6*h));
    }

    public double getC(int n) {
        double h = points[n + 1].getX() - points[n].getX();
        return ((points[n + 1].getY()/h) - ((z[n + 1]*h)/6));
    }

    public double getD(int n) {
        double h = points[n + 1].getX() - points[n].getX();
        return ((points[n].getY()/h) - ((z[n]*h)/6));
    }

    public Point[] getXYSet() {
        Point[] fullSet = new Point[20*(points.length - 1) + 1];
        int count = 0;
        for (int i = 1; i < points.length; i++) {
            double diffx = points[i].getX() - points[i-1].getX();
            double step = (diffx / 20);
            //Point[] set = new Point[20];
            for (double j = points[i-1].getX(); j < points[i].getX(); j += step) {
                System.out.println(j);
                double y = getY(j,i-1);
                fullSet[count] = new Point(j, y);
                count++;
            }
        }
        return fullSet;
    }

}