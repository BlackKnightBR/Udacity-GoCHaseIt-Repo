#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include "sensor_msgs/Image.h"

//Define a global client that can request services
ros::ServiceClient client;

//Chama a função que dirige o robo ao alvo (bola branca)
void drive_robot(float lin_x, float ang_z){

    ROS_INFO("Diriga o robot: ");

    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    if (!client.call(srv)){
        ROS_ERROR("Não foi possivel chamar a função 'DriveToTarget'");
    }
}

//Callback function executa a todo momento analisando a imagem em busca da bola
void process_image_callback(const sensor_msgs::Image img){

    int white_pixel = 255;
    int left_bound = int(img.width / 3);
    int right_bound = 2 * int(img.width / 3);
    int right_count = 0; int mid_count = 0; int left_count = 0;
    int max_count = 0;

    for (int i = 0; i < img.height; i++){
        for (int j = 0; j < img.step; j += 3){
            int index = j + (i * img.step);
            if (img.data[index] == white_pixel && img.data[index + 1] == white_pixel && img.data[index + 2] == white_pixel){
                if (j <= left_bound){
                    left_count += 1;
                } else if (j >= right_bound){
                    right_count += 1;
                } else {
                    mid_count += 1;
                }
            } else{
                continue;
            }
        }
    }
    std::count << "esquerda: " << left_count << " ; frente: " << mid_count << " ; direita: " << right_count << std::endl;

    max_count = std::max(std::max(left_count, mid_count), right_count);
    if (max_count == 0){
        drive_robot(0.0, 0.0);
    } else if (max_count == left_count){
        drive_robot(0.5, 0.5);
    } else if (max_count == right_count){
        drive_robot(0.5, -0.5);
    } else{
        drive_robot(0.5, 0.0);
    }

}

int main(int argc, char**argv){
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);
    
    ros::spin();

    return 0;
}
