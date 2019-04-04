public class Trajectory {
    private double lin_vel; //linear velocity of robot
    private double ang_vel; //angular velocity of robot
    private double r; //radius of wheel
    private double l; //half the wheelbase
    private double dx; //x change of center of wheelbase
    private double dy; //y change of center of wheelbase
    private double dtheta; //angle change from last to current position
    private double theta; //current angle of velocity
    private double[] theta_arr = {0,0};
    private double[][] velocity = new double[2][1];
    private double[][] wheelbase_inv = new double[2][2];
    private double det;

    public Trajectory(double x1, double y1, double x2, double y2) {
        dx = x2 - x1;
        dy = y2 - y1;
        theta = Math.tan(dy/dx);
        setLinVel();
        setAngVel();
        setThetaArr();
        setDtheta();
        makeArrays();
        setDet();
    }

    public Trajectory(double x1, double y1, double x2, double y2, double linear) {
        dx = x2 - x1;
        dy = y2 - y1;
        theta = Math.tan(dy/dx);
        makeArrays(linear, linear/r);
        setDet();
    }

    public void setLinVel() {
        double v1 = dx / Math.cos(theta);
        double v2 = dy / Math.sin(theta);
        lin_vel = (v1 + v2) / 2;
    }
    public void setAngVel() {
        ang_vel = dtheta;
    }
    public void setThetaArr() {
        theta_arr[0] = theta_arr[1];
        theta_arr[1] = theta;
    }
    public void setDtheta() {
        dtheta = theta_arr[1] - theta_arr[0];
    }
    public void makeArrays() {
        makeArrays(lin_vel, ang_vel);

    }
    public void makeArrays(double linear, double angular) {
        velocity[0][0] = (theta/Math.PI)*linear;
        velocity[1][0] = (theta/Math.PI)*angular;
        wheelbase_inv[0][0] = -r / (2 * l);
        wheelbase_inv[0][1] = -r / 2;
        wheelbase_inv[1][0] = r / (2 * l);
        wheelbase_inv[1][1] = r / 2;
    }
    public void setDet() {
        det = -(r*r) / (2*l);
    }
    public double[] getWheelSpeed() {
        double[] wheelSpeeds = new double[2];
        wheelSpeeds[0] = det * (wheelbase_inv[0][0] * velocity[0][0] + wheelbase_inv[0][1] * velocity[1][0]);
        wheelSpeeds[1] = det * (wheelbase_inv[1][0] * velocity[0][0] + wheelbase_inv[1][1] * velocity[1][0]);
        return wheelSpeeds;

    }

}