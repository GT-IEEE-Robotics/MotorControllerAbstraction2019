public class Trajectory
{
    static double PI = 3.1415926;
    double lin_vel;             //linear velocity of robot
    double ang_vel;             //angular velocity of robot
    double r;                   //radius of wheel
    double l;                   //half the wheelbase
    double curr_x;
    double curr_y;
    double new_x;
    double new_y;
    double dx;                  //x change of center of wheelbase
    double dy;                  //y change of center of wheelbase
    double init_theta;
    double dtheta;              //angle change from last to current position
    double theta;               //current angle of velocity
    double[] theta_arr = new double[2];
    double[] velocity = new double[2];
    double[][] wheelbase_inv = new double[2][2];
    Double[] wheelSpeeds = new Double[2];
    double det;
    double ratio;
public Trajectory(double old_x, double old_y, double init_theta_input, double r_wheel, double l_wheelbase, double ratio_input)
{
    new_x = old_x;
    new_y = old_y;

    init_theta = init_theta_input;
    theta_arr[1] = init_theta;
    ratio = ratio_input;
    r = r_wheel;
    l = l_wheelbase;
}

public void getNextPoint(double new_x_input, double new_y_input)
{
    curr_x = new_x;
    curr_y = new_y;
    new_x = new_x_input;
    new_y = new_y_input;

    dx = new_x - curr_x;
    dy = new_y - curr_y;
    if (dx == 0)
    {
        if (dy == 0)
            theta = 0;
        else if (dy > 0)
            theta = PI/2;
        else
            theta = -PI/2;
    }
    else
    {
        theta = Math.atan2(dy,dx);
    }

    // setLinVel();
    // setThetaArr();
    // setDtheta();
    // setAngVel();
    // makeArrays();
    // setDet();
    // getRatio();
}

public void setLinVel()
{
    double v1;
    double v2;
    if (theta == PI/2 || theta == -PI/2)
    {
        v1 = 0;
        v2 = dy / Math.sin(theta);
    }
    else if (theta == 0 || theta == PI)
    {
        v1 = dx / Math.cos(theta);
        v2 = 0;
    }
    else
    {
        v1 = dx / Math.cos(theta);
        v2 = dy / Math.sin(theta);
    }

    lin_vel = Math.sqrt(v1*v1 + v2*v2);
}

public void setAngVel()
{
    ang_vel = dtheta;
}

public void setThetaArr()
{
    theta_arr[0] = theta_arr[1];
    theta_arr[1] = theta;
}

public void setDtheta()
{
    dtheta = theta_arr[1] - theta_arr[0];
}

public void makeArrays()
{
    velocity[0] = lin_vel;
    velocity[1] = ang_vel;
    wheelbase_inv[0][0] = -r / (2 * l);
    wheelbase_inv[0][1] = -r / 2;
    wheelbase_inv[1][0] = -r / (2 * l);
    wheelbase_inv[1][1] = r / 2;
}

public void setDet()
{
    det = -(r*r) / (2*l);
}

public void getRatio()
{
    ratio = 2.5 / (Math.abs(velocity[1]) + 0.05);
}

public Double[] getWheelSpeed()
{
    wheelSpeeds[0] = ratio * (1/det) * (wheelbase_inv[0][0] * velocity[0] + wheelbase_inv[0][1] * velocity[1]);
    wheelSpeeds[1] = ratio * (1/det) * (wheelbase_inv[1][0] * velocity[0] + wheelbase_inv[1][1] * velocity[1]);
    wheelSpeeds[0] = Math.abs(wheelSpeeds[0]);
    wheelSpeeds[1] = Math.abs(wheelSpeeds[1]);
    return wheelSpeeds;
}

public static void main(String[] args)
{
    double curr_x = 0;
    double curr_y = 0;
    double next_x;
    double next_y;
    double[] x_trajectory = {0.1,0.2,0.3,0.35,0.35,0.3,0.2,0.15,0.15,0.15};
    double[] y_trajectory = {0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1};
    int i;
    Trajectory test_trajectory = new Trajectory(0, 0, PI/2, 2, 5, 1);
    for (i=0; i<10; i++)
    {

        test_trajectory.getNextPoint(x_trajectory[i], y_trajectory[i]);
        test_trajectory.setLinVel();
        test_trajectory.setThetaArr();
        test_trajectory.setDtheta();
        test_trajectory.setAngVel();
        test_trajectory.makeArrays();
        test_trajectory.setDet();
        test_trajectory.getRatio();
        test_trajectory.getWheelSpeed();

        System.out.println("point #: " + i);
        //cout << "dtheta: " << test_trajectory.dtheta << "\n";
        //cout << "linvel: " << test_trajectory.velocity[0] << "\n";
        //cout << "angvel: " << test_trajectory.velocity[1] << "\n";
        //cout << "ratio: " << test_trajectory.ratio << "\n";
        System.out.println("left wheel speed: " + test_trajectory.wheelSpeeds[1]);
        System.out.println("right wheel speed: " + test_trajectory.wheelSpeeds[0]);
    }
}
}