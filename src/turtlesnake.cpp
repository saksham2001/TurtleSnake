#include "ros/ros.h"
#include "turtlesim/Spawn.h"
#include "turtlesim/Kill.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include "std_srvs/Empty.h"
#include <cstdlib>
#include <string>
#include <sstream>
#include <cmath>
#include <vector>

// X & Y: 0 to 11
//Theta: -3.14 to 3.14

class TurtleHandler
{
    private:
        // All publishers and subscribers

        int turtle_id = 1;
        float MIN_THETA = -3.14;
        float MAX_THETA = 3.14;
        float MIN_XY = 0.0;
        float MAX_XY = 11.0;

        float turtle1_x = 5.54;
        float turtle1_y = 5.54;
        float turtle1_theta = 0;
        std::string turtle1_name = "turtle1";

        float turtle2_x;
        float turtle2_y;
        float turtle2_theta;
        std::string turtle2_name = "turtle2";
        bool turtle2_flag = false;
        int turtle2_at = 0;

        float turtle3_x;
        float turtle3_y;
        float turtle3_theta;
        std::string turtle3_name = "turtle3";
        bool turtle3_flag = false;
        int turtle3_at = 0;


        float turtle4_x;
        float turtle4_y;
        float turtle4_theta;
        std::string turtle4_name = "turtle4";
        bool turtle4_flag = false;
        int turtle4_at = 0;


        float turtle5_x;
        float turtle5_y;
        float turtle5_theta;
        std::string turtle5_name = "turtle5";
        bool turtle5_flag = false;
        int turtle5_at = 0;


        ros::ServiceClient spawn_client;
        ros::ServiceClient reset_client;

        // Turtle cmd_vel Publishers
        ros::Publisher turtle2_pub;
        ros::Publisher turtle3_pub;
        ros::Publisher turtle4_pub;
        ros::Publisher turtle5_pub;

        // Turtle pose Subscribers
        ros::Subscriber turtle1_sub;
        ros::Subscriber turtle2_sub;
        ros::Subscriber turtle3_sub;
        ros::Subscriber turtle4_sub;
        ros::Subscriber turtle5_sub;

        // Turtle Sequence
        std::vector<std::vector<float*>> sequence;

    public:
        TurtleHandler(ros::NodeHandle *nh) 
        {
            spawn_client = nh->serviceClient<turtlesim::Spawn>("spawn");
            reset_client = nh->serviceClient<std_srvs::Empty>("reset");

            turtle2_pub = nh->advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 10);
            turtle3_pub = nh->advertise<geometry_msgs::Twist>("/turtle3/cmd_vel", 10);
            turtle4_pub = nh->advertise<geometry_msgs::Twist>("/turtle4/cmd_vel", 10);
            turtle5_pub = nh->advertise<geometry_msgs::Twist>("/turtle5/cmd_vel", 10);

            turtle1_sub = nh->subscribe("/turtle1/pose", 10, &TurtleHandler::turtle1_pose_callback, this);
            turtle2_sub = nh->subscribe("/turtle2/pose", 10, &TurtleHandler::turtle2_pose_callback, this);
            turtle3_sub = nh->subscribe("/turtle3/pose", 10, &TurtleHandler::turtle3_pose_callback, this);
            turtle4_sub = nh->subscribe("/turtle4/pose", 10, &TurtleHandler::turtle4_pose_callback, this);
            turtle5_sub = nh->subscribe("/turtle5/pose", 10, &TurtleHandler::turtle5_pose_callback, this);

            std::vector<float*> vec {&turtle1_x, &turtle1_y, &turtle1_theta};
            sequence.push_back(vec);
        }

        void turtle1_pose_callback(const turtlesim::Pose::ConstPtr& pose)
        {
            turtle1_x = pose->x;
            turtle1_y = pose->y;
            turtle1_theta = pose->theta;
        }

        void turtle2_pose_callback(const turtlesim::Pose::ConstPtr& pose)
        {
            turtle2_x = pose->x;
            turtle2_y = pose->y;
            turtle2_theta = pose->theta;
        }

        void turtle3_pose_callback(const turtlesim::Pose::ConstPtr& pose)
        {
            turtle3_x = pose->x;
            turtle3_y = pose->y;
            turtle3_theta = pose->theta;
        }

        void turtle4_pose_callback(const turtlesim::Pose::ConstPtr& pose)
        {
            turtle4_x = pose->x;
            turtle4_y = pose->y;
            turtle4_theta = pose->theta;
        }

        void turtle5_pose_callback(const turtlesim::Pose::ConstPtr& pose)
        {
            turtle5_x = pose->x;
            turtle5_y = pose->y;
            turtle5_theta = pose->theta;
        }

        float random_gen(float min, float max)
        {   
            // Returns Random number between min and max
            return ((((float) rand()) / (float) RAND_MAX)*(max-min)) + min;
        }

        float euclidean_distance(float turtleg_x, float turtleg_y, float turtle_x, float turtle_y)
        {
            // Returns Euclidean distance from turtle1
            return sqrt(pow((turtleg_x - turtle_x), 2) + (pow((turtleg_y - turtle_y), 2)));
        }

        float linear_vel(float turtleg_x, float turtleg_y, float turtle_x, float turtle_y, float constant)
        {
            // Returns Linear Velocity to turtle1
            return constant*euclidean_distance(turtleg_x, turtleg_y, turtle_x, turtle_y);
        }

        float stearing_angle(float turtleg_x, float turtleg_y, float turtle_x, float turtle_y)
        {
            // Returns Stearing angle to turtle1
            return atan2(turtleg_y-turtle_y, turtleg_x-turtle_x);
        }

        float angular_vel(float turtleg_x, float turtleg_y, float turtle_x, float turtle_y, float turtle_theta, float constant)
        {
            // Returns Angular Velocity to turtle1
            return constant*(stearing_angle(turtleg_x, turtleg_y, turtle_x, turtle_y) - turtle_theta);
        }

        bool check_hit()
        {
            if((euclidean_distance(turtle1_x, turtle1_y, turtle2_x, turtle2_y) <= 2.0) && (turtle2_flag == false))
            {
                turtle2_flag = true;

                std::vector<float*> vec {&turtle2_x, &turtle2_y, &turtle2_theta};
                sequence.push_back(vec);
                turtle2_at = sequence.size() - 1;

                return true;
            } else if((euclidean_distance(turtle1_x, turtle1_y, turtle3_x, turtle3_y) <= 2.0) && (turtle3_flag == false))
            {
                turtle3_flag = true;

                std::vector<float*> vec {&turtle3_x, &turtle3_y, &turtle3_theta};
                sequence.push_back(vec);
                turtle3_at = sequence.size() - 1;

                return true;
            } else if((euclidean_distance(turtle1_x, turtle1_y, turtle4_x, turtle4_y) <= 2.0) && (turtle4_flag == false))
            {
                turtle4_flag = true;

                std::vector<float*> vec {&turtle4_x, &turtle4_y, &turtle4_theta};
                sequence.push_back(vec);
                turtle4_at = sequence.size() - 1;

                return true;
            } else if((euclidean_distance(turtle1_x, turtle1_y, turtle5_x, turtle5_y) <= 2.0) && (turtle5_flag == false))
            {
                turtle5_flag = true;

                std::vector<float*> vec {&turtle5_x, &turtle5_y, &turtle5_theta};
                sequence.push_back(vec);
                turtle5_at = sequence.size() - 1;

                return true;
            }

            return false;
        }

        bool go2turtle1()
        {
            geometry_msgs::Twist turtle2_vel;
            geometry_msgs::Twist turtle3_vel;
            geometry_msgs::Twist turtle4_vel;
            geometry_msgs::Twist turtle5_vel;

            if (turtle2_flag == true)
            {   
                std::vector<float*> vec = sequence.at(turtle2_at);

                float turtleg_x = *(vec.at(0));
                float turtleg_y = *(vec.at(1));
                float turtleg_theta = *(vec.at(2));

                if ((euclidean_distance(turtleg_x, turtleg_y, turtle2_x, turtle2_y) >= 0.01))
                {
                    turtle2_vel.linear.x = linear_vel(turtleg_x, turtleg_y, turtle2_x, turtle2_y, 1.0);
                    turtle2_vel.linear.y = 0;
                    turtle2_vel.linear.z = 0;

                    turtle2_vel.angular.x = 0;
                    turtle2_vel.angular.y = 0;
                    turtle2_vel.angular.z = angular_vel(turtleg_x, turtleg_y, turtle2_x, turtle2_y, turtle2_theta, 6.0);

                   ROS_WARN("turtle2_x: %f, turtle2_y: %f", turtle2_x, turtle2_y);
                } else 
                {
                    turtle2_vel.linear.x = 0;
                    turtle2_vel.linear.y = 0;
                    turtle2_vel.linear.z = 0;

                    turtle2_vel.angular.x = 0;
                    turtle2_vel.angular.y = 0;
                    turtle2_vel.angular.z = 0;
                }

                turtle2_pub.publish(turtle2_vel);
            }

            if (turtle3_flag == true)
            {
                std::vector<float*> vec = sequence.at(turtle3_at);

                float turtleg_x = *(vec.at(0));
                float turtleg_y = *(vec.at(1));
                float turtleg_theta = *(vec.at(2));

                if ((euclidean_distance(turtleg_x, turtleg_y, turtle3_x, turtle3_y) >= 0.01) && turtle3_flag == true)
                {
                    turtle3_vel.linear.x = linear_vel(turtleg_x, turtleg_y, turtle3_x, turtle3_y, 1.0);
                    turtle3_vel.linear.y = 0;
                    turtle3_vel.linear.z = 0;

                    turtle3_vel.angular.x = 0;
                    turtle3_vel.angular.y = 0;
                    turtle3_vel.angular.z = angular_vel(turtleg_x, turtleg_y, turtle3_x, turtle3_y, turtle3_theta, 6.0);

                    ROS_WARN("turtle3_x: %f, turtle3_y: %f", turtle3_x, turtle3_y);
                } else 
                {
                    turtle3_vel.linear.x = 0;
                    turtle3_vel.linear.y = 0;
                    turtle3_vel.linear.z = 0;

                    turtle3_vel.angular.x = 0;
                    turtle3_vel.angular.y = 0;
                    turtle3_vel.angular.z = 0;
                }

                turtle3_pub.publish(turtle3_vel);
            }

            if (turtle4_flag == true)
            {
                std::vector<float*> vec = sequence.at(turtle4_at);

                float turtleg_x = *(vec.at(0));
                float turtleg_y = *(vec.at(1));
                float turtleg_theta = *(vec.at(2));

                if ((euclidean_distance(turtleg_x, turtleg_y, turtle4_x, turtle4_y) >= 0.01) && turtle4_flag == true)
                {
                    turtle4_vel.linear.x = linear_vel(turtleg_x, turtleg_y, turtle4_x, turtle4_y, 1.0);
                    turtle4_vel.linear.y = 0;
                    turtle4_vel.linear.z = 0;

                    turtle4_vel.angular.x = 0;
                    turtle4_vel.angular.y = 0;
                    turtle4_vel.angular.z = angular_vel(turtleg_x, turtleg_y, turtle4_x, turtle4_y, turtle4_theta, 6.0);

                    ROS_WARN("turtle4_x: %f, turtle4_y: %f", turtle4_x, turtle4_y);
                } else 
                {
                    turtle4_vel.linear.x = 0;
                    turtle4_vel.linear.y = 0;
                    turtle4_vel.linear.z = 0;

                    turtle4_vel.angular.x = 0;
                    turtle4_vel.angular.y = 0;
                    turtle4_vel.angular.z = 0;
                }

                turtle4_pub.publish(turtle4_vel);
            }

            if (turtle5_flag == true)
            {
                std::vector<float*> vec = sequence.at(turtle5_at);

                float turtleg_x = *(vec.at(0));
                float turtleg_y = *(vec.at(1));
                float turtleg_theta = *(vec.at(2));

                if ((euclidean_distance(turtleg_x, turtleg_y, turtle5_x, turtle5_y) >= 0.01) && turtle5_flag == true)
                {
                    turtle5_vel.linear.x = linear_vel(turtleg_x, turtleg_y, turtle5_x, turtle5_y, 1.0);
                    turtle5_vel.linear.y = 0;
                    turtle5_vel.linear.z = 0;

                    turtle5_vel.angular.x = 0;
                    turtle5_vel.angular.y = 0;
                    turtle5_vel.angular.z = angular_vel(turtleg_x, turtleg_y, turtle5_x, turtle5_y, turtle5_theta, 6.0);

                    ROS_WARN("turtle5_x: %f, turtle5_y: %f", turtle5_x, turtle5_y);
                } else 
                {
                    turtle5_vel.linear.x = 0;
                    turtle5_vel.linear.y = 0;
                    turtle5_vel.linear.z = 0;

                    turtle5_vel.angular.x = 0;
                    turtle5_vel.angular.y = 0;
                    turtle5_vel.angular.z = 0;
                }

                turtle5_pub.publish(turtle5_vel);
            }
        
            return true;
        }
        
        bool spawn_turtle()
        {

            turtlesim::Spawn srv;
            
            srv.request.x = random_gen(MIN_XY, MAX_XY);
            srv.request.y = random_gen(MIN_XY, MAX_XY);
            srv.request.theta = random_gen(MIN_THETA, MAX_THETA);

            std::stringstream ss;
            ss << "turtle" << ++turtle_id;
            srv.request.name = ss.str();

            if (spawn_client.call(srv))
            {
                ROS_INFO("New Turtle %s spawned [X:%d, Y:%d, Theta:%d]", srv.response.name.c_str(), srv.request.x, srv.request.y, srv.request.theta);
                if (turtle_id == 2)
                {
                    turtle2_x = srv.request.x;
                    turtle2_y = srv.request.y;
                    turtle2_theta = srv.request.theta;
                } else if (turtle_id == 3)
                {
                    turtle3_x = srv.request.x;
                    turtle3_y = srv.request.y;
                    turtle3_theta = srv.request.theta;
                } else if (turtle_id == 4)
                {
                    turtle4_x = srv.request.x;
                    turtle4_y = srv.request.y;
                    turtle4_theta = srv.request.theta;
                } else if (turtle_id == 5)
                {
                    turtle5_x = srv.request.x;
                    turtle5_y = srv.request.y;
                    turtle5_theta = srv.request.theta;
                } else
                return true;
            } else 
            {
                ROS_INFO("Failed to call service");
                return false;
            }
        }

        bool game_over_graphics()
        {
            //Colour Changes

            
            //Reset
            std_srvs::Empty reset_srv;

            if (reset_client.call(reset_srv))
            {
                ROS_INFO("Reseting Turtlesim...");
            } else 
            {
                ROS_INFO("Failed to Reset");
                return false;
            }
        }

        bool controller()
        {
            go2turtle1();
        }

        void run()
        {
            ros::Rate rate(2); 

            for(int i=0; i<5; i++)
            {
                spawn_turtle();
            }

            while(ros::ok())
            {
                ros::spinOnce();

                ROS_WARN("x: %f, y: %f", turtle1_x, turtle1_y);

                bool flag = check_hit();

                controller();

                rate.sleep();
            }
        }

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtlesnake");

    ros::NodeHandle nh;

    TurtleHandler th(&nh);

    th.run();

    return 0;
}