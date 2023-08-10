#include "SAR_DataConverter.h"




void SAR_DataConverter::MainInit()
{
    LoadParams();
    Time_start = ros::Time::now();
    BodyCollision_str = SAR_Config + "::Base_Model::SAR_Body::body_collision"; 
    isInit = true;

}

void SAR_DataConverter::MainLoop()
{

    MainInit();
    int loopRate = 1000;     // [Hz]
    ros::Rate rate(loopRate);

    
    while(ros::ok)
    {   
        checkSlowdown();
        
        // PUBLISH ORGANIZED DATA
        Publish_StateData();
        Publish_TriggerData();
        Publish_ImpactData();
        Publish_MiscData();

        rate.sleep();
    }


}

int main(int argc, char** argv)
{
    
    ros::init(argc,argv,"SAR_DataConverter_Node");
    ros::NodeHandle nh;
    SAR_DataConverter SAR_DC = SAR_DataConverter(&nh);
    ros::spin();
    return 0;
}