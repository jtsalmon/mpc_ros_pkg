#include <iostream>
#include <cmath>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <boost/thread.hpp>
#include "ros/ros.h"
#include "tf/tf.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "actionlib/server/simple_action_server.h"

using namespace std;

// Global Variables

    const float pi = 3.1415926535;

    const float V = 1;                 // Desired Tractor Velocity
    float U1 = V;                           // Actual Tractor Velocity
    const float a = 0.75;              // (m) - est length from GPS receiver to front of robot
    const float b = 1.9558-a;           // (m) - est length from GPS receiver to robot back axle
    const float c = 2.4892-a;           // (m) - est length from robot GPS receiver to hitch
    const float d = 3.0;               // (m) - est length from trailer GPS receiver to hitch
    const float e = 4-d;                // (m) - est length from CG of trailer to back axle

    float horizon = 4;                  // Prediction Horizon
    const float predTol = 0.01;         // Prediction Tolerance
    const float tstep = 0.025;          // Time Step Value (for model)
    const float Tstep = .1;             // For Predictor

    float tractposX,tractposY,tractposq,trailposX,trailposY,trailposq;

boost::mutex robot_state_mutex_;
boost::mutex robot_and_trailer_state_mutex_;

std::string robot_topic_;
std::string trailer_topic_;
std::string command_topic_;

inline float square(float x) {return x*x;}
inline float absolute(float x) {return sqrt(x*x);}
float sign(float);

void robot_odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
void trailer_odom_callback(const nav_msgs::Odometry::ConstPtr& msg);

int main(int argc, char **argv)
{
    // BUILDING THE PATH //

    float Xmax = 6;                     // Maximum X value of path
    float R = 5;                        // Turn Radius

    const float pi = 3.1415926535;

    float Xinc = tstep;                 // Increment of position step
    float qpathinc = tstep/R;           // Increment of angular step (for turns)

    float XR1 = Xmax;                   // X-position of center of first turn
    float YR1 = R;                      // Y-position of center of first turn
    float XR2 = -Xmax;                  // X-position of center of second turn
    float YR2 = YR1*3;                  // Y-position of center of second turn

    int lengthQR = pi/qpathinc + 1;     // Index length of turns

    float QR1[lengthQR],QR2[lengthQR];

    int j = 0;
    float k1 = 0;

    for (int i = 0; i < lengthQR; i++)
    {
        QR1[i] = -pi/2 + qpathinc*k1;    // Angles for first turn
        QR2[i] = (pi/2)*3 - qpathinc*k1; // Angles for second turn
        k1 = k1+1;
    }
    k1 = 0;

    int straight = 2*Xmax/tstep + 1;    // Index length of straight portions of S-curve

    int lengthPath = 3*straight+2*lengthQR - 1;     // Total Length of Path

    float Xpathfull[lengthPath],Ypathfull[lengthPath];

    // Bottom of S-curve
    for (int i = 0; i < straight; i++)
    {
        Xpathfull[i] = -XR1 + Xinc*k1;
        Ypathfull[i] = 0;
        k1 = k1+1;
    }
    k1 = 0;

    // First curve
    for (int i = straight; i < straight + lengthQR; i++)
    {
        Xpathfull[i] = XR1 + R*cos(QR1[j]);
        Ypathfull[i] = YR1 + R*sin(QR1[j]);
        j = j+1;
    }

    j = 0;

    // Middle of S-curve
    for (int i = straight+lengthQR; i < 2*straight+lengthQR-1; i++)
    {
        Xpathfull[i] = XR1-Xinc - Xinc*k1;
        Ypathfull[i] = YR1 + R;
        k1 = k1+1;
    }

    k1 = 0;

    // Second curve
    for (int i = 2*straight+lengthQR-1; i < 2*straight+2*lengthQR-1; i++)
    {
        Xpathfull[i] = XR2 + R*cos(QR2[j]);
        Ypathfull[i] = YR2 + R*sin(QR2[j]);
        j = j+1;
    }

    j = 0;

    // Top of S-curve
    for (int i = 2*straight+2*lengthQR-1; i < 3*straight+2*lengthQR-1; i++)
    {
        Xpathfull[i] = XR2+Xinc + Xinc*k1;
        Ypathfull[i] = YR2 + R;
        k1 = k1+1;
    }

    k1 = 0;

    int lengthXpath = sizeof(Xpathfull)/sizeof(int);

    // Making a text file of the path using Matlab syntax //

/*
    ofstream outFile;
    outFile.open("pathout.txt");
    outFile << "A = [";
    for (int i = 0; i < lengthXpath; i++)
        outFile << Xpathfull[i] << " " << Ypathfull[i] << endl;
    outFile << "];" << endl << endl;
    outFile << "figure" << endl;
    outFile << "plot(A(:,1),A(:,2))" << endl;
    outFile << "axis equal" << endl;
    outFile.close();
*/

    // INITIAL SETUP //

    // Initialize ROS
    ros::init(argc,argv,"dubins_node");
    ros::NodeHandle n;

    // Subscribe to Odometry
    ros::Subscriber robot_odom_subscriber_ = n.subscribe(robot_topic_,1000,robot_odom_callback);
    ros::Subscriber trailer_odom_subscriber_ = n.subscribe(trailer_topic_,1000,trailer_odom_callback);
    

    //ros::init(argc,argv,"Motion");
    //ros::NodeHandle n;
    ros::Publisher movement_pub;
    //Publisher movement_pub = n.advertise<std_msgs::String>("/Segway/Motion",1000);
    movement_pub = n.advertise<geometry_msgs::Twist>(command_topic_,0);
    ros::Rate loop_rate(40);

    ros::spinOnce();




    vector<float> Xtract(1);
    vector<float> Ytract(1);
    vector<float> qtract(1);
    vector<float> Xtrail(1);
    vector<float> Ytrail(1);
    vector<float> qtrail(1);

    Xtrail[0] = trailposX;
    Ytrail[0] = trailposY;
    qtrail[0] = trailposq;
    qtract[0] = tractposq;
    Xtract[0] = tractposX;
    Ytract[0] = tractposY;

    // Use if no simulator:
    Xtrail[0] = Xpathfull[0];
    Ytrail[0] = Ypathfull[0];
    qtrail[0] = 0.0;
    qtract[0] = 0.0;
    Xtract[0] = Xtrail[0] + d*cos(qtrail[0]) + c*cos(qtract[0]);
    Ytract[0] = Xtrail[0] + d*sin(qtrail[0]) + c*sin(qtract[0]);

    int ijk = 0;
    int datachunksize = Tstep/tstep;

    vector<float> OMEGAtract(1);
    OMEGAtract[0] = 0;

    float tcount;

    // CREATING PATH CHUNK //


    float distances[lengthXpath];
    float smallest = 6;
    int index;
    for (int i = 0; i < lengthXpath; i++)
    {
        distances[i] = sqrt(square(Xtrail[0]-Xpathfull[i])+square(Ytrail[0]-Ypathfull[i]));
        if (distances[i] < smallest)
        {
            smallest = distances[i];
            index = i;
        }
    }

    int pathChunkSize = horizon/tstep + 2;
    float Xpath[pathChunkSize];
    float Ypath[pathChunkSize];

    if (index + pathChunkSize <= lengthXpath)
    {
        for (int kk = 0; kk < pathChunkSize; kk++)
        {
            Xpath[kk] = Xpathfull[index+kk];
            Ypath[kk] = Ypathfull[index+kk];
        }
    }
    else
    {
        float Xstep = Xpathfull[lengthXpath-1] - Xpathfull[lengthXpath-2];
        float Ystep = Ypathfull[lengthXpath-1] - Ypathfull[lengthXpath-2];
        float pathAngle = atan(Ystep/Xstep);
        for (int kk = 0; kk < lengthXpath-index; kk++)
        {
            Xpath[kk] = Xpathfull[index+kk];
            Ypath[kk] = Ypathfull[index+kk];
        }
        for (int kk = lengthXpath-index; kk < pathChunkSize; kk++)
        {
            Xpath[kk] = Xpath[kk-1] + Xstep*cos(pathAngle);
            Ypath[kk] = Ypath[kk-1] + Ystep*sin(pathAngle);
        }
        if (index >= lengthXpath-1)
            horizon = 0;
    }

    index = 0;

/*
    ofstream outFile;
    outFile.open("pathshortout.txt");
    outFile << "A = [";
    for (int i = 0; i < pathChunkSize; i++)
        outFile << Xpath[i] << " " << Ypath[i] << endl;
    outFile << "];" << endl << endl;
    outFile << "figure" << endl;
    outFile << "plot(A(:,1),A(:,2))" << endl;
    outFile << "axis equal" << endl;
    outFile.close();
*/


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // MPC Algorithm

    cout << "Running MPC Algorithm" << endl;

    int i,k;


    while (horizon > 0 && ros::ok())
    {
        i = 0;                                      // Keeps track of the predictions done for each cycle
        k = Xtract.size() - 1;                      // Keeps track of the time-step updates

        float OMEGAtest[20];                        // An array of steering angles being tested
        float J[20],Jprime[20];                     // Cost Functions and their derivatives wrt steer angle
        OMEGAtest[i] = OMEGAtract[k];

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Setting Up First Prediction
        k1 = 0;

        int lengtht_model = (horizon)/(V*tstep)+2;

        float t_model[lengtht_model];
        for (int ii = 0; ii < lengtht_model; ii++)
        {
            t_model[ii] = tstep*k1;
            k1 = k1+1;
        }

        k1 = 0;

        float q1tst[lengtht_model],q2tst[lengtht_model],deltatst[lengtht_model],X1tst[lengtht_model],X2tst[lengtht_model];
        float Y1tst[lengtht_model],Y2tst[lengtht_model],dq1dttst[lengtht_model],dq2dttst[lengtht_model];
        float dX1dttst[lengtht_model],dY1dttst[lengtht_model],dX2dttst[lengtht_model],dY2dttst[lengtht_model];

        q1tst[0] = qtract[k];
        q2tst[0] = qtrail[k];

        X2tst[0] = Xtrail[k] - e*cos(q2tst[0]);
        Y2tst[0] = Ytrail[k] - e*sin(q2tst[0]);
        X1tst[0] = Xtrail[k] + d*cos(q2tst[0]) + (c-b)*cos(q1tst[0]);
        Y1tst[0] = Ytrail[k] + d*sin(q2tst[0]) + (c-b)*sin(q1tst[0]);
        //X1tst[0] = Xtract[k] - b*cos(q1tst[0]);
        //Y1tst[0] = Ytract[k] - b*sin(q1tst[0]);

        deltatst[0] = qtract[k] - qtrail[k];

        float OMEGA = OMEGAtest[i];

        float k2,k3,k4;

        for (int ii = 0; ii < lengtht_model-1; ii++)
        {
            dq1dttst[ii] = OMEGA;
            k1 = tstep*dq1dttst[ii];
            k2 = tstep*(dq1dttst[ii] + .5*k1);
            k3 = tstep*(dq1dttst[ii] + .5*k2);
            k4 = tstep*(dq1dttst[ii] + k3);
            q1tst[ii+1] = q1tst[ii] + (k1 + 2*k2 + 2*k3 + k4)/6;

            dq2dttst[ii] = U1/(d+e)*sin(deltatst[ii]) - (c-b)*dq1dttst[ii]/(d+e)*cos(deltatst[ii]);
            k1 = tstep*dq2dttst[ii];
            k2 = tstep*(U1/(d+e)*sin(deltatst[ii]) - (c-b)*(dq1dttst[ii]+.5*k1)/(d+e)*cos(deltatst[ii]));
            k3 = tstep*(U1/(d+e)*sin(deltatst[ii]) - (c-b)*(dq1dttst[ii]+.5*k2)/(d+e)*cos(deltatst[ii]));
            k4 = tstep*(U1/(d+e)*sin(deltatst[ii]) - (c-b)*(dq1dttst[ii]+k3)/(d+e)*cos(deltatst[ii]));
            q2tst[ii+1] = q2tst[ii] + (k1 + 2*k2 + 2*k3 + k4)/6;

            deltatst[ii+1] = q1tst[ii+1] - q2tst[ii+1];

            dX1dttst[ii] = U1*cos(q1tst[ii]);
            k1 = tstep*dX1dttst[ii];
            k2 = tstep*(U1*cos(q1tst[ii]+.5*k1));
            k3 = tstep*(U1*cos(q1tst[ii]+.5*k2));
            k4 = tstep*(U1*cos(q1tst[ii]+k3));
            X1tst[ii+1] = X1tst[ii] + (k1 + 2*k2 + 2*k3 + k4)/6;

            dY1dttst[ii] = U1*sin(q1tst[ii]);
            k1 = tstep*dY1dttst[ii];
            k2 = tstep*(U1*sin(q1tst[ii]+.5*k1));
            k3 = tstep*(U1*sin(q1tst[ii]+.5*k2));
            k4 = tstep*(U1*sin(q1tst[ii]+k3));
            Y1tst[ii+1] = Y1tst[ii] + (k1 + 2*k2 + 2*k3 + k4)/6;

            //if (ijk == 59)
                //cout << X1tst[ii+1] << " " << Y1tst[ii+1] << " " << endl;
        }
//        cout << sizeof(X1tst)/sizeof(int) << endl;

        k1 = 0;

        int horizint = horizon;

        float q1pred[horizint],X1pred[horizint],Y1pred[horizint],q2pred[horizint],X2pred[horizint],Y2pred[horizint];

        for (int j = 0; j < horizint; j++)
        {
            index = (j+1)/(tstep*V);
            q1pred[j] = q1tst[index];
            q2pred[j] = q2tst[index];
            X1pred[j] = X1tst[index] + b*cos(q1pred[j]);
            Y1pred[j] = Y1tst[index] + b*sin(q1pred[j]);
            X2pred[j] = X1pred[j] - c*cos(q1pred[j]) - d*cos(q2pred[j]);
            Y2pred[j] = Y1pred[j] - c*sin(q1pred[j]) - d*sin(q2pred[j]);

            //if (ijk == 59)
            {
                //cout << X1pred[jj] << " " << Y1pred[jj] << " " << X2pred[jj] << " " << Y2pred[jj] << endl;
                //cout << Y2pred[jj] << " " << Ytrail[k] << endl;
            }
        }

        index = 0;

        // Determining Cost of First Prediction

        float sum = 0;
        float L[horizint],thetapath[horizint],thetapred[horizint],theta[horizint];
        float Lengths[pathChunkSize],Xpathmin[horizint],Ypathmin[horizint];
        int ind[horizint];
        for (int j = 0; j < horizint; j++)
        {
            L[j] = 6;
            for (int p = 0; p < pathChunkSize; p++)
            {
                Lengths[p] = sqrt(square(X2pred[j]-Xpath[p])+square(Y2pred[j]-Ypath[p]));
                if (Lengths[p] < L[j])
                {
                    L[j] = Lengths[p];
                    ind[j] = p;
                }
            }
            Xpathmin[j] = Xpath[ind[j]];
            Ypathmin[j] = Ypath[ind[j]];
            thetapath[j] = atan((Ypathmin[j]-Ytrail[k])/(Xpathmin[j]-Xtrail[k]));
            thetapred[j] = atan((Y2pred[j]-Ytrail[k])/(X2pred[j]-Xtrail[k]));
            theta[j] = thetapath[j]-thetapred[j];
            L[j] = L[j]*sign(theta[j]);
            sum = sum + L[j];
        }
        J[i] = sum;
        Jprime[i] = 0;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Setting Up Subsequent Predictions (Newton's Method Process)

        float costChange = 5;

        i = i+1;

        OMEGAtest[i] = OMEGAtest[i-1] + pi/180*sign(J[i-1]);

        while (costChange > predTol && i <= 19)
        {
            OMEGA = OMEGAtest[i];

            for (int ii = 0; ii < lengtht_model-1; ii++)
            {
                dq1dttst[ii] = OMEGA;
                k1 = tstep*dq1dttst[ii];
                k2 = tstep*(dq1dttst[ii] + .5*k1);
                k3 = tstep*(dq1dttst[ii] + .5*k2);
                k4 = tstep*(dq1dttst[ii] + k3);
                q1tst[ii+1] = q1tst[ii] + (k1 + 2*k2 + 2*k3 + k4)/6;

                dq2dttst[ii] = U1/(d+e)*sin(deltatst[ii]) - (c-b)*dq1dttst[ii]/(d+e)*cos(deltatst[ii]);
                k1 = tstep*dq2dttst[ii];
                k2 = tstep*(U1/(d+e)*sin(deltatst[ii]) - (c-b)*(dq1dttst[ii]+.5*k1)/(d+e)*cos(deltatst[ii]));
                k3 = tstep*(U1/(d+e)*sin(deltatst[ii]) - (c-b)*(dq1dttst[ii]+.5*k2)/(d+e)*cos(deltatst[ii]));
                k4 = tstep*(U1/(d+e)*sin(deltatst[ii]) - (c-b)*(dq1dttst[ii]+k3)/(d+e)*cos(deltatst[ii]));
                q2tst[ii+1] = q2tst[ii] + (k1 + 2*k2 + 2*k3 + k4)/6;

                deltatst[ii+1] = q1tst[ii+1] - q2tst[ii+1];

                dX1dttst[ii] = U1*cos(q1tst[ii]);
                k1 = tstep*dX1dttst[ii];
                k2 = tstep*(U1*cos(q1tst[ii]+.5*k1));
                k3 = tstep*(U1*cos(q1tst[ii]+.5*k2));
                k4 = tstep*(U1*cos(q1tst[ii]+k3));
                X1tst[ii+1] = X1tst[ii] + (k1 + 2*k2 + 2*k3 + k4)/6;

                dY1dttst[ii] = U1*sin(q1tst[ii]);
                k1 = tstep*dY1dttst[ii];
                k2 = tstep*(U1*sin(q1tst[ii]+.5*k1));
                k3 = tstep*(U1*sin(q1tst[ii]+.5*k2));
                k4 = tstep*(U1*sin(q1tst[ii]+k3));
                Y1tst[ii+1] = Y1tst[ii] + (k1 + 2*k2 + 2*k3 + k4)/6;
            }

            int horizint = horizon;

            float q1pred[horizint],X1pred[horizint],Y1pred[horizint],q2pred[horizint],X2pred[horizint],Y2pred[horizint];

            for (int jj = 0; jj < horizint; jj++)
            {
                index = (jj+1)/(tstep*V);
                q1pred[jj] = q1tst[index];
                q2pred[jj] = q2tst[index];
                X1pred[jj] = X1tst[index] + b*cos(q1pred[jj]);
                Y1pred[jj] = Y1tst[index] + b*sin(q1pred[jj]);
                X2pred[jj] = X1pred[jj] - c*cos(q1pred[jj]) - d*cos(q2pred[jj]);
                Y2pred[jj] = Y1pred[jj] - c*sin(q1pred[jj]) - d*sin(q2pred[jj]);
            }
            index = 0;

            // Determine Cost of Second Iteration

            sum = 0;
            for (int j = 0; j < horizint; j++)
            {
                L[j] = 6;
                for (int p = 0; p < pathChunkSize; p++)
                {
                    Lengths[p] = sqrt(square(X2pred[j]-Xpath[p])+square(Y2pred[j]-Ypath[p]));
                    if (Lengths[p] < L[j])
                    {
                        L[j] = Lengths[p];
                        ind[j] = p;
                    }
                }
                Xpathmin[j] = Xpath[ind[j]];
                Ypathmin[j] = Ypath[ind[j]];
                thetapath[j] = atan((Ypathmin[j]-Ytrail[k])/(Xpathmin[j]-Xtrail[k]));
                thetapred[j] = atan((Y2pred[j]-Ytrail[k])/(X2pred[j]-Xtrail[k]));
                theta[j] = thetapath[j]-thetapred[j];
                L[j] = L[j]*sign(theta[j]);
                sum = sum + L[j];
            }
            J[i] = sum;
            Jprime[i] = (J[i]-J[i-1])/(OMEGAtest[i]-OMEGAtest[i-1]);
            sum = 0;
            i = i+1;

            costChange = absolute(J[i-1]/Jprime[i-1]);

            OMEGAtest[i] = OMEGAtest[i-1] - J[i-1]/Jprime[i-1];
        }


            i = i-1;
//        cout << L[0] << " " << L[1] << " " << L[2] << " " << L[3] << endl;
//        cout << J[i] << " " << DELTAtest[i]*180/pi << endl;

        int datachunkstart = k;
        int datachunkend = datachunkstart + datachunksize;

        if (i >= 18)
        {
            float Jmin = 1000;
            for (int ij = 0; ij < 19; ij++)
            {
                if (J[ij] < Jmin)
                {
                    Jmin = J[ij];
                    index = ij;
                }
            }
            for (int kk = datachunkstart; kk < datachunkend+1; kk++)
            {
                if (kk < OMEGAtract.size())
                    OMEGAtract[kk] = OMEGAtest[index];
                else
                {
                    OMEGAtract.push_back(1);
                    OMEGAtract[kk] = OMEGAtest[index];

                }
            }
            index = 0;
        }
        else
        {
            for (int kk = datachunkstart; kk < datachunkend+1; kk++)
            {

                if (kk < OMEGAtract.size())
                    OMEGAtract[kk] = OMEGAtest[i];
                else
                {
                    OMEGAtract.push_back(1);
                    OMEGAtract[kk] = OMEGAtest[i];
                }
            }

        }
//        cout << DELTAtract[k] << " " << J[i] << endl;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Send Steer Angle Command to Robot

        OMEGA = OMEGAtract[k+horizint]; // OMEGA is the turn rate in rad/sec

        geometry_msgs::Twist msg;
        msg.linear.x = V;
        msg.linear.y = 0.0;
        msg.linear.z = 0.0;
        msg.angular.x = 0.0;
        msg.angular.y = 0.0;
        msg.angular.z = OMEGA;
        //movement_pub.publish("/Segway/Motion",msg);
        movement_pub.publish(msg);

        // Collect Feedback from Robot in 0.025 second incrememts

        for (int ii = 0; ii < datachunksize; ii++)
        {
            loop_rate.sleep(); // Wait for 0.025 seconds
            ros::spinOnce(); // Receive current GPS reading

            Xtract.push_back(1);
            Xtract[k+ii+1] = tractposX;
            Ytract.push_back(1);
            Ytract[k+ii+1] = tractposY;
            qtract.push_back(1);
            qtract[k+ii+1] = tractposq;
            Xtrail.push_back(1);
            Xtrail[k+ii+1] = trailposX;
            Ytrail.push_back(1);
            Ytrail[k+ii+1] = trailposY;
            qtrail.push_back(1);
            qtrail[k+ii+1] = trailposq;
        }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        
        for (int ii = 0; ii < datachunksize; ii++)
        {
            dq1dttst[ii] = OMEGA;
            k1 = tstep*dq1dttst[ii];
            k2 = tstep*(dq1dttst[ii] + .5*k1);
            k3 = tstep*(dq1dttst[ii] + .5*k2);
            k4 = tstep*(dq1dttst[ii] + k3);
            q1tst[ii+1] = q1tst[ii] + (k1 + 2*k2 + 2*k3 + k4)/6;

            dq2dttst[ii] = U1/(d+e)*sin(deltatst[ii]) - (c-b)*dq1dttst[ii]/(d+e)*cos(deltatst[ii]);
            k1 = tstep*dq2dttst[ii];
            k2 = tstep*(U1/(d+e)*sin(deltatst[ii]+.5*k1) - (c-b)*(dq1dttst[ii]+.5*k1)/(d+e)*cos(deltatst[ii]+.5*k1));
            k3 = tstep*(U1/(d+e)*sin(deltatst[ii]+.5*k2) - (c-b)*(dq1dttst[ii]+.5*k2)/(d+e)*cos(deltatst[ii]+.5*k2));
            k4 = tstep*(U1/(d+e)*sin(deltatst[ii]+k3) - (c-b)*(dq1dttst[ii]+k3)/(d+e)*cos(deltatst[ii]+k3));
            q2tst[ii+1] = q2tst[ii] + (k1 + 2*k2 + 2*k3 + k4)/6;

            deltatst[ii+1] = q1tst[ii+1] - q2tst[ii+1];

            dX1dttst[ii] = U1*cos(q1tst[ii]);
            k1 = tstep*dX1dttst[ii];
            k2 = tstep*(U1*cos(q1tst[ii]+.5*k1));
            k3 = tstep*(U1*cos(q1tst[ii]+.5*k2));
            k4 = tstep*(U1*cos(q1tst[ii]+k3));
            X1tst[ii+1] = X1tst[ii] + (k1 + 2*k2 + 2*k3 + k4)/6;

            dY1dttst[ii] = U1*sin(q1tst[ii]);
            k1 = tstep*dY1dttst[ii];
            k2 = tstep*(U1*sin(q1tst[ii]+.5*k1));
            k3 = tstep*(U1*sin(q1tst[ii]+.5*k2));
            k4 = tstep*(U1*sin(q1tst[ii]+k3));
            Y1tst[ii+1] = Y1tst[ii] + (k1 + 2*k2 + 2*k3 + k4)/6;

            //if (ijk == 1 || ijk == 2 || ijk == 3 || ijk == 4)
            //cout << X1tst[ii+1] << " " << Y1tst[ii+1] << " " << q1tst[ii+1] << " " << q2tst[ii+1] << endl;

            //qtract.push_back(1);
            qtract[k+ii+1] = q1tst[ii+1];
            //qtrail.push_back(1);
            qtrail[k+ii+1] = q2tst[ii+1];
            //Xtract.push_back(1);
            Xtract[k+ii+1] = X1tst[ii+1] + b*cos(q1tst[ii+1]);
            //Ytract.push_back(1);
            Ytract[k+ii+1] = Y1tst[ii+1] + b*sin(q1tst[ii+1]);
            //Xtrail.push_back(1);
            Xtrail[k+ii+1] = X1tst[ii+1] + (b-c)*cos(q1tst[ii+1]) - d*cos(q2tst[ii+1]);
            //Ytrail.push_back(1);
            Ytrail[k+ii+1] = Y1tst[ii+1] + (b-c)*sin(q1tst[ii+1]) - d*sin(q2tst[ii+1]);
            //if (ijk == 1 || ijk == 2 || ijk == 3 || ijk == 4)
            //cout << Xtract[k+ii+1] << " " << Ytract[k+ii+1] << " " << Xtrail[k+ii+1] << " " << Ytrail[k+ii+1] << endl;


            // For Simulation Purposes:
            tcount = tstep*(k+ii+1);
            cout << Xtract[k+ii+1] << " " << Ytract[k+ii+1] << " " << Xtrail[k+ii+1] << " " << Ytrail[k+ii+1] << " " << OMEGAtract[k+ii+1] << " " << tcount << endl;

        }

        



        k = Xtract.size() - 1;

        k1 = 0;

        // Create Next Path Chunk

        index = 0;
        smallest = 7;
        for (int ii = 0; ii < lengthXpath; ii++)
        {

            distances[ii] = sqrt(square(Xtrail[k]-Xpathfull[ii])+square(Ytrail[k]-Ypathfull[ii]));
            if (distances[ii] < smallest)
            {
                smallest = distances[ii];
                index = ii;
            }
        }
        if (index + pathChunkSize <= lengthXpath)
        {
            for (int kk = 0; kk < pathChunkSize; kk++)
            {
                Xpath[kk] = Xpathfull[index+kk];
                Ypath[kk] = Ypathfull[index+kk];
            }
        }
        else
        {
            float Xstep = Xpathfull[lengthXpath-1] - Xpathfull[lengthXpath-2];
            float Ystep = Ypathfull[lengthXpath-1] - Ypathfull[lengthXpath-2];
            float pathAngle = atan(Ystep/Xstep);
            for (int kk = 0; kk < lengthXpath-index; kk++)
            {
                Xpath[kk] = Xpathfull[index+kk];
                Ypath[kk] = Ypathfull[index+kk];
            }
            for (int kk = lengthXpath-index; kk < pathChunkSize; kk++)
            {
                Xpath[kk] = Xpath[kk-1] + Xstep*cos(pathAngle);
                Ypath[kk] = Ypath[kk-1] + Ystep*sin(pathAngle);
            }
            if (index >= lengthXpath-1)
                horizon = 0;
        }
        //if (ijk == 89)
        {
            //for (int iii = 0; iii < pathChunkSize; iii++)
                //cout << Xpath[iii] << " " << Ypath[iii] << " " << distances[index] << endl;
        }

        index = 0;

        ijk = ijk+1;            // Keeps track of the times the MPC Algorithm recycles
    }

// Stop the robot
cout << "Stopping the robot" << endl;
geometry_msgs::Twist msg2;
msg2.linear.x = 0.0;
msg2.linear.y = 0.0;
msg2.linear.z = 0.0;
msg2.angular.x = 0.0;
msg2.angular.y = 0.0;
msg2.angular.z = 0.0;
//movement_pub.publish("/Segway/Motion",msg);
movement_pub.publish(msg2);
ros::Duration(0.25).sleep();
movement_pub.publish(msg2);
ros::Duration(0.25).sleep();
movement_pub.publish(msg2);
ros::Duration(0.25).sleep();
movement_pub.publish(msg2);
ros::Duration(0.25).sleep();
movement_pub.publish(msg2);

//for (int k = 0; k < Xtrail.size(); k++)
//    cout << Xtract[k] << " " << Ytract[k] << " " << Xtrail[k] << " " << Ytrail[k] << endl;

//cout << Xtract.size() << endl;
    float t[Xtract.size()];
    for (int i = 0; i < Xtract.size(); i++)
    {
        t[i] = tstep*k1;
        k1 = k1+1;
    }

/*
    ofstream outFile;
    outFile.open("MPCout");
    outFile << "clear all, close all, clc" << endl;
    outFile << "A = [";
    for (int i = 0; i < Xtrail.size(); i++)
        outFile << Xtract[i] << " " << Ytract[i] << " " << Xtrail[i] << " " << Ytrail[i] << " " << OMEGAtract[i] << " " << t[i] << endl;
    outFile << "];" << endl << endl;
    outFile << "figure" << endl;
    outFile << "plot(A(:,1),A(:,2),A(:,3),A(:,4))" << endl;
    outFile << "axis equal" << endl;
    outFile << "figure" << endl;
    outFile << "plot(A(:,6),A(:,5)*180/pi)" << endl;
    outFile.close();
    */

    k1 = 0;
}


float sign(float x)
{
    if (x >= 0)
        return 1;
    else
        return -1;
}

void robot_odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // received odometry data with robot position
    // if controller just robot, put values in robot state structure
    // if controlling robot and trailer, put values in robot portion
    // of robot and trailer state structure

        boost::mutex::scoped_lock lock(robot_and_trailer_state_mutex_);
        // convert odom message to robot state
        tractposX=msg->pose.pose.position.x;
        tractposY=msg->pose.pose.position.y;
        // convert orientation to Euler angles to get heading
        // then convert from enu to ned for use in controller
        tractposq=tf::getYaw(msg->pose.pose.orientation);
        //robot_and_trailer_state_.robot_pose.yaw = theta2psi(GetYawFromQuat(msg->pose.pose.orientation));
        U1=msg->twist.twist.linear.x;
        //robot_and_trailer_state_.robot_pose.yawRate=-msg->twist.twist.angular.z; // change sign on yaw rate (enu to ned)

        if (U1 == 0)
            U1 = V;

        U1 = V; // For Simulation
    /*
    if (control_mode_==ROBOT)
    {
        boost::mutex::scoped_lock lock(robot_state_mutex_);
        // convert odom message to robot state
        tractposX=msg->pose.pose.position.x;
        tractposY=msg->pose.pose.position.y;
        // convert orientation to Euler angles to get heading
        // then convert from enu to ned for use in controller
        tractposq=msg->pose.pose.orientation;
        //robot_state_.yaw = theta2psi(GetYawFromQuat(msg->pose.pose.orientation));
        //std::cout << "orig: " << theta2psi(tf::getYaw(msg->pose.pose.orientation)) << " new: " << robot_state_.yaw << std::endl;
        //robot_state_.speed=msg->twist.twist.linear.x;
        //robot_state_.yawRate=-msg->twist.twist.angular.z; // change sign on yaw rate (enu to ned)
    }
    else
    {
        boost::mutex::scoped_lock lock(robot_and_trailer_state_mutex_);
        // convert odom message to robot state
        tractposX=msg->pose.pose.position.x;
        tractposY=msg->pose.pose.position.y;
        // convert orientation to Euler angles to get heading
        // then convert from enu to ned for use in controller
        tractposq=msg->pose.pose.orientation;
        //robot_and_trailer_state_.robot_pose.yaw = theta2psi(GetYawFromQuat(msg->pose.pose.orientation));
        //robot_and_trailer_state_.robot_pose.speed=msg->twist.twist.linear.x;
        //robot_and_trailer_state_.robot_pose.yawRate=-msg->twist.twist.angular.z; // change sign on yaw rate (enu to ned)
    }
    */

    // robot_odom_received_=true;

  }

  void trailer_odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
  {
      boost::mutex::scoped_lock lock(robot_and_trailer_state_mutex_);
      trailposX=msg->pose.pose.position.x;
      trailposY=msg->pose.pose.position.y;
      trailposq=tf::getYaw(msg->pose.pose.orientation);
      //robot_and_trailer_state_.trailer_pose.yaw = theta2psi(GetYawFromQuat(msg->pose.pose.orientation));
      //robot_and_trailer_state_.trailer_pose.speed=msg->twist.twist.linear.x;
      //robot_and_trailer_state_.trailer_pose.yawRate=-msg->twist.twist.angular.z; // change sign on yaw rate (enu to ned)
  }
