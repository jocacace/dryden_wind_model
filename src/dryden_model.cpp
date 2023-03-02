#include "ros/ros.h"
#include "Eigen/Dense"
#include "dryden_model.h"
#include "geometry_msgs/WrenchStamped.h"
#include "std_msgs/Bool.h"
#include <fstream>

using namespace Eigen;
using namespace std;

#define LOGS 0


int main(int argc, char** argv) {

    ros::init(argc, argv, "dryden_wind");
    ros::NodeHandle nh;
    dryden_model::DrydenWind wind;

    double max_force;
    double max_gust;
    string topic_name;
    int motors;
    ros::Publisher ft_pub;

    if( !nh.getParam("max_force", max_force) ) {
        max_force =  5.0;
    }
    if( !nh.getParam("max_gust", max_gust) ) {
        max_gust =  0.2;
    }
    if( !nh.getParam("topic_name", topic_name ) ) {
        topic_name = "hummingbird";
    }
    if( !nh.getParam("motors", motors ) ) {
        motors = 4;
    }

    ft_pub = nh.advertise<geometry_msgs::WrenchStamped>(topic_name, 1);
    
    Vector3d ws; //wind speed
    Vector3d wg; //wind gust

    const float p_blink_range_from       = 0.0;
    const float p_blink_range_to         = 1.0; //0.0 to max_force N
    std::random_device                  p_blink_rand_dev;
    std::mt19937                        p_blink_rand_gen(p_blink_rand_dev());
    std::uniform_real_distribution<float>  p_blink_ws(p_blink_range_from, p_blink_range_to);
    
    const float xyz_ws_range_from       = 0.0;
    const float xyz_ws_range_to         = max_force; //0.0 to max_force N
    std::random_device                  xyz_ws_rand_dev;
    std::mt19937                        xyz_ws_rand_gen(xyz_ws_rand_dev());
    std::uniform_real_distribution<float>  xyz_ws(xyz_ws_range_from, xyz_ws_range_to);
    
    const float xyz_wg_range_from       = 0.0;
    const float xyz_wg_range_to         = max_gust; //0.0 to max_force N
    std::random_device                  xyz_wg_rand_dev;
    std::mt19937                        xyz_wg_rand_gen(xyz_wg_rand_dev());
    std::uniform_real_distribution<float>  xyz_wg(xyz_wg_range_from, xyz_wg_range_to);

    const int gen_fault_num_range_from  = 0;
    const int gen_fault_num_range_to    = motors-1;
    std::random_device                  gen_fault_rand_dev;
    std::mt19937                        gen_fault_generator(gen_fault_rand_dev());
    std::uniform_int_distribution<int>  gen_fault_distr(gen_fault_num_range_from, gen_fault_num_range_to);

    ros::AsyncSpinner spinner(0);
    spinner.start();

    
    ofstream log_file( "/tmp/wind_model.txt",  std::ios_base::app);
    /*if (!log_file.is_open()) {
        std::cout << "Error opening file" << endl;
        exit(0);
    }
    */


    double altitude = 2.0;

    double dt = 0.0;
    ros::Rate r(5);

    geometry_msgs::WrenchStamped wind_w;

    dt = 0.0;
    //ws << xyz_ws( xyz_ws_rand_gen ), xyz_ws( xyz_ws_rand_gen ), xyz_ws( xyz_ws_rand_gen );
    //ws = ws / max_force;
    //wg << xyz_wg(xyz_wg_rand_gen), xyz_wg(xyz_wg_rand_gen), xyz_wg(xyz_wg_rand_gen);
    ws << max_force, max_force, max_force;
    wg << max_gust, max_gust, max_gust;
    
    wind.initialize(ws[0], ws[1], ws[2], wg[0], wg[1], wg[2], altitude);
    wind_w.header.frame_id = "base_link";

    while(  ros::ok()) {
        wind_w.header.stamp = ros::Time::now();
        dt += 1.0/5.0;
        Eigen::Vector3d wind_velocity = wind.getWind(dt);
                
        wind_w.wrench.force.x = wind_velocity[0];
        wind_w.wrench.force.y = wind_velocity[1];
        wind_w.wrench.force.z = wind_velocity[2]; 
        ft_pub.publish( wind_w);

        r.sleep();
           
    }
    return 0;
}