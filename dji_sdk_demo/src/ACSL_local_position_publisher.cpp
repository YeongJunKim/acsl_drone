#include "dji_sdk_demo/ACSL_local_position_publisher.h"

ros::Publisher ctrlPosYawPub;
ros::Publisher pub_local_position;

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "demo_local_position_control_node");
    ros::NodeHandle nh;

    pub_local_position = nh.advertise<dji_sdk::ACSL_local_position>("ACSL/local_position",10);

    dji_sdk::ACSL_local_position local_position;

    local_position.seq = 0;
    local_position.targetX = 0;
    local_position.targetY = 0;
    local_position.targetZ = 0;
    local_position.targetYaw = 0;
    pub_local_position.publish(local_position);

    while (1)
    {
        char a;
        float target[4] = {0,};
        
        cout << "continue : c exit : q" << endl << " : ";
        cin >> a;

        if( a == 'q')
            break;
        
        cout << "target local x" << endl << " : ";
        cin >> target[0];
        cout << "target local y" << endl << " : ";
        cin >> target[1];
        cout << "target local z" << endl << " : ";
        cin >> target[2];
        cout << "target yaw" << endl << " : ";
        cin >> target[3];

        local_position.seq++;
        local_position.targetX = target[0];
        local_position.targetY = target[1];
        local_position.targetZ = target[2];
        local_position.targetYaw = target[3];
        pub_local_position.publish(local_position);
        ros::spinOnce();
    }

    return 0;
}