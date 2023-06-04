/*
    Name: Aniket Patil
    Task: RBE450x HW04
*/

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <image_transport/image_transport.h>
#include <controller_manager_msgs/SwitchController.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/JointState.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <string>
#include <iostream>
#include <fstream>

using namespace cv;
using namespace std;

Point red_center;
Point yellow_center;
Point green_center;
Point blue_center;

static const double link1 = 0.5;
static const double link2 = 0.5;

static const Scalar red_low = Scalar(0, 205, 110);
static const Scalar red_high = Scalar(10, 255, 160);
static const Scalar green_low = Scalar(50, 205, 110);
static const Scalar green_high = Scalar(70, 255, 160);
static const Scalar blue_low = Scalar(110, 205, 110);
static const Scalar blue_high = Scalar(130, 255, 160);
static const Scalar yellow_low = Scalar(20, 205, 110);
static const Scalar yellow_high = Scalar(40, 255, 160);

float lambda = 0.002;
double l_Le[2][8] = {{0.25*lambda, 0, 0.25*lambda, 0, 0.25*lambda, 0, 0.25*lambda, 0},
                {0, 0.25*lambda, 0, 0.25*lambda, 0, 0.25*lambda, 0, 0.25*lambda}};


class VisualServoing
{
private:
    // Declare required variables
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub_;
    ros::Publisher pub_q1_pos;
    ros::Publisher pub_q2_pos;
    ros::Publisher pub_q1_vel;
    ros::Publisher pub_q2_vel;
    ros::ServiceClient client;
    ros::Subscriber joint_sub;

    std_msgs::Float64 q1_pos;
    std_msgs::Float64 q2_pos;
    std_msgs::Float64 q1_vel;
    std_msgs::Float64 q2_vel;
    Point current_centers[4];
    Point goal_centers[4];
    double errors[8][8];
    double Vc[2][8];
    double inv_Jacob[2][8];
    double joint_vel[2][8];
    double theta1;
    double theta2;
    Mat img;

    bool vel_ctrl_flag;
    ofstream myFile;

public:
    VisualServoing()
        : it(nh)
    {

        // Initialise Publisher and Subscriber
        pub_q1_pos = nh.advertise<std_msgs::Float64>("/vbmbot/joint1_position_controller/command", 10);
        pub_q2_pos = nh.advertise<std_msgs::Float64>("/vbmbot/joint2_position_controller/command", 10);
        pub_q1_vel = nh.advertise<std_msgs::Float64>("/vbmbot/joint1_velocity_controller/command", 10);
        pub_q2_vel = nh.advertise<std_msgs::Float64>("/vbmbot/joint2_velocity_controller/command", 10);
        image_sub_ = it.subscribe("/vbmbot/camera1/image_raw", 1,
                                   &VisualServoing::imageCb, this);
        joint_sub = nh.subscribe ("/vbmbot/joint_states", 10, &VisualServoing::find_joint_states, this);
        sleep(1);

        // Switch to Position control (in case, it was set in velocity control in previous run)
        switch_control(1);

        motion();
    }

    ~VisualServoing()
    {
        
    }
    
    void motion()
    {
        // Set Position 1
        q1_pos.data = 0.5;
        q2_pos.data = -1.0;
        pub_q1_pos.publish(q1_pos);
        pub_q2_pos.publish(q2_pos);
        cout << "Position 1" << endl;
        sleep(8);
        // Subscriber is defined, spinOnce will initiate callback
        ros::spinOnce();
        goal_centers[0] = red_center;
        goal_centers[1] = green_center;
        goal_centers[2] = blue_center;
        goal_centers[3] = yellow_center;
        waitKey(4000);
        imwrite("screenshots/image_view_pos1.jpg", img);
        // Set Position 2
        q1_pos.data = 1;
        q2_pos.data = -2;
        pub_q1_pos.publish(q1_pos);
        pub_q2_pos.publish(q2_pos);
        sleep(8);
        cout << "Position 2" << endl;
        // Subscriber is defined, spinOnce will initiate callback
        ros::spinOnce();
        current_centers[0] = red_center;
        current_centers[1] = green_center;
        current_centers[2] = blue_center;
        current_centers[3] = yellow_center;
        waitKey(4000);
        imwrite("screenshots/image_view_pos2.jpg", img);

        // Switch to velocity control
        switch_control(2);
        ros::spin();
    }

    void switch_control(int i)
    {
        // Switch Controller between position and velocity controllers
        try
        {
            client = nh.serviceClient<controller_manager_msgs::SwitchController>("/vbmbot/controller_manager/switch_controller");
            controller_manager_msgs::SwitchController sw_ctrl_srv;
            sw_ctrl_srv.request.strictness = 2;
            sw_ctrl_srv.request.start_asap = false;
            sw_ctrl_srv.request.timeout = 0.0;
            if(i == 2)
            {
                vel_ctrl_flag = true;
                sw_ctrl_srv.request.start_controllers = {"joint1_velocity_controller", "joint2_velocity_controller"};
                sw_ctrl_srv.request.stop_controllers = {"joint1_position_controller", "joint2_position_controller"};
            }
            else if(i == 1)
            {
                vel_ctrl_flag = false;
                sw_ctrl_srv.request.start_controllers = {"joint1_position_controller", "joint2_position_controller"};
                sw_ctrl_srv.request.stop_controllers = {"joint1_velocity_controller", "joint2_velocity_controller"};
            }
            client.call(sw_ctrl_srv);
            sleep(1);
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
    }

    void calc_error()
    {
        // Calculate errors during velocity control motion
        errors[0][0] = (current_centers[0].x - goal_centers[0].x);
        errors[1][0] = (current_centers[0].y - goal_centers[0].y);
        errors[2][0] = (current_centers[1].x - goal_centers[1].x);
        errors[3][0] = (current_centers[1].y - goal_centers[1].y);
        errors[4][0] = (current_centers[2].x - goal_centers[2].x);
        errors[5][0] = (current_centers[2].y - goal_centers[2].y);
        errors[6][0] = (current_centers[3].x - goal_centers[3].x);
        errors[7][0] = (current_centers[3].y - goal_centers[3].y);
    }
    
    void publish_Vel()
    {
        // Function for velocity control

        // Update current centers using global values of all 4 centers
        current_centers[0] = red_center;
        current_centers[1] = green_center;
        current_centers[2] = blue_center;
        current_centers[3] = yellow_center;

        cout << "Theta 1: " << theta1 << endl;
        cout << "Theta 2: " << theta2 << endl;

        // Calculate Error by subtracting goal position co-ordinates and current position co-ordinates
        calc_error();
        cout << "Error Matrix: " << endl;
        for(int i = 0; i < 8; i++)
        {
            cout << errors[i][0] << ", ";
        }
        cout << endl;

        // Multiply matrix Lambda * Le_inverse to find Vc
        multiply_mat(l_Le, errors, Vc, 2, 8, 8, 1);
        calc_invJacob();

        // Multiply inv_Jacobian with Vc to find joint velocities
        multiply_mat(inv_Jacob, Vc, joint_vel, 2, 2, 2, 1);
        joint_vel[0][0] = joint_vel[0][0] > 1 ? 1 : joint_vel[0][0];
        joint_vel[1][0] = joint_vel[1][0] > 1 ? 1 : joint_vel[1][0];
        joint_vel[0][0] = joint_vel[0][0] < -1 ? -1 : joint_vel[0][0];
        joint_vel[1][0] = joint_vel[1][0] < -1 ? -1 : joint_vel[1][0];

        cout << "Joint Velocity" << endl;
        display_mat(joint_vel, 2, 1);
        
        // Get and publish joint velocities to the controller
        q1_vel.data = joint_vel[0][0];
        q2_vel.data = joint_vel[1][0];
        pub_q1_vel.publish(q1_vel);
        pub_q2_vel.publish(q2_vel);
    }

    void find_joint_states(const sensor_msgs::JointStatePtr & msg)
    {
        // Use Joint State Publisher to get values of position and store them in theta
        theta1 = msg->position[0];
        theta2 = msg->position[1];
    }

    void calc_invJacob()
    {
        // Jacobian calculation for 2 revolute joints
        double a = (-1*link1*sin(theta1))-(link2*sin(theta1 + theta2));
        double b = -1*link2*sin(theta1 + theta2);
        double c = (link1*cos(theta1)) + (link2*cos(theta1 + theta2));
        double d = (link2*cos(theta1 + theta2));

        // Inverse of 2x2 matrix
        double denom = (a*d) - (b*c);
        inv_Jacob[0][0] = (d / denom);
        inv_Jacob[0][1] = (-1*b / denom);
        inv_Jacob[1][0] = (-1*c / denom);
        inv_Jacob[1][1] = (a / denom);

        cout << "Inverse Jacobian: " << endl;
        display_mat(inv_Jacob, 2, 2);
    }

    void multiply_mat(double firstMatrix[][8], double secondMatrix[][8], double mult[][8], int rowFirst, int columnFirst, int rowSecond, int columnSecond)
    {
        int i, j, k;

        // Initializing elements of matrix mult to 0.
        for(i = 0; i < rowFirst; ++i)
        {
            for(j = 0; j < columnSecond; ++j)
            {
                mult[i][j] = 0;
            }
        }

        // Multiplying matrix firstMatrix and secondMatrix and storing in array mult
        for(i = 0; i < rowFirst; ++i)
        {
            for(j = 0; j < columnSecond; ++j)
            {
                for(k=0; k<columnFirst; ++k)
                {
                    mult[i][j] += firstMatrix[i][k] * secondMatrix[k][j];
                    //cout << mult[i][j] << " " << firstMatrix[i][k] << " * " << secondMatrix[k][j] << endl;
                }
            }
        }
    }

    void display_mat(double mult[][8], int rowFirst, int columnSecond)
    {
        // Function iterates over the matrix and displays the output
        for(int i = 0; i < rowFirst; ++i)
        {
            for(int j = 0; j < columnSecond; ++j)
            {
                cout << mult[i][j] << " ";
                if(j == columnSecond - 1)
                cout << endl << endl;
            }
        }
    }

    void imageCb(const sensor_msgs::ImageConstPtr &msg)
    {
        // Subsrciber Call Back function (CV Bridge)
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // Process image to find centers of the cylinders
        process_img(cv_ptr);

        // Publish velocity controller values and saving real-time values of the bot needs to be done only when velocity control is enabled
        if(vel_ctrl_flag == 1)
        {
            publish_Vel();
            save_data();
        }

    }

    void process_img(const cv_bridge::CvImagePtr &img_ptr)
    {
        // This function thresholds HSV image and finds centers of all 4 cylinders on the block
        Mat hsv_img, thresh_img;
        img = img_ptr->image.clone();
        cvtColor(img_ptr->image, hsv_img, CV_BGR2HSV);

        // Threshold the HSV image and call find_center functions
        inRange(hsv_img, green_low, green_high, thresh_img);
        green_center = find_center(thresh_img);
        inRange(hsv_img, red_low, red_high, thresh_img);
        red_center = find_center(thresh_img);
        inRange(hsv_img, blue_low, blue_high, thresh_img);
        blue_center = find_center(thresh_img);
        inRange(hsv_img, yellow_low, yellow_high, thresh_img);
        yellow_center = find_center(thresh_img);
        
        // Draw black dots on the image where centers of circles are detected
        circle(img, red_center, 1, Scalar(0, 0, 0), 2, 8, 0);
        circle(img, green_center, 1, Scalar(0, 0, 0), 2, 8, 0);
        circle(img, blue_center, 1, Scalar(0, 0, 0), 2, 8, 0);
        circle(img, yellow_center, 1, Scalar(0, 0, 0), 2, 8, 0);

        // Print colour centers
        cout << "====================" << endl << "Centers for the 4 circles:" << endl;
        cout << "Red Center: " << red_center << endl;
        cout << "Yellow Center: " << yellow_center << endl;
        cout << "Green Center: " << green_center << endl;
        cout << "Blue Center: " << blue_center << endl;
    }

    Point find_center(Mat bin_img)
    {
        // Find pixel center using averaging of pixels in binary image
        int j_avg = 0, i_avg = 0, elements = 0;
        for (int i = 0; i<bin_img.cols ; i++)
        {
            for (int j = 0; j<bin_img.rows; j++)
            {
                if((int)bin_img.at<uchar>(i,j) != 0)
                {
                    j_avg += j;
                    i_avg += i;
                    elements += 1;
                }
            }
        }
        return Point((j_avg/elements), (i_avg/elements));
    }

    void save_data()
    {
        // Write Data to CSV file
        myFile.open("out_data.csv", ios::app);

        myFile << red_center.x << "," << red_center.y << ",";
        myFile << green_center.x << "," << green_center.y << ",";
        myFile << blue_center.x << "," << blue_center.y << ",";
        myFile << yellow_center.x << "," << yellow_center.y << ",";
        myFile << "\n";

        myFile.close();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "visual_servoing");
    VisualServoing vs;
    ROS_INFO("End of Motion");
    return 0;
}