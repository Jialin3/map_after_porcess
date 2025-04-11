#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>  // 添加 OccupancyGridUpdate 消息头文件

#include <opencv2/opencv.hpp>

const std::string MapSubTopic = "/map_grid_node/map";
const std::string MapPubTopic = "/map_after_inflation";
const std::string MapUpdateTopic = "/map_after_inflation_updates";  // 新增更新话题

// const std::string ImageLogPath = "/mnt/d/image-log/";

const int8_t LAND = 0;
const int8_t WALL = 100;
const int8_t UNKNOWN = -1;

int DILATE_RADIUS = 5;

ros::Publisher mapPub;
ros::Publisher mapUpdatePub;  // 新增更新发布器

std::vector<int8_t> LookupTableImpl(256);

void publishMapUpdate(const cv::Mat_<int8_t>& map, const nav_msgs::OccupancyGrid& original_map)
{
    map_msgs::OccupancyGridUpdate update;
    update.header = original_map.header;
    update.x = 0;
    update.y = 0;
    update.width = original_map.info.width;
    update.height = original_map.info.height;
    
    // 将更新数据转换为vector
    std::vector<int8_t> data(map.data, map.data + map.total());
    update.data = data;
    
    mapUpdatePub.publish(update);
}

void mapCallback(const nav_msgs::OccupancyGrid &subData)
{
    nav_msgs::OccupancyGrid pubData;
    pubData.header = subData.header;
    pubData.info = subData.info;
    
    cv::Mat_<int8_t> map(subData.data);
    map = map.reshape(1, subData.info.height);

    // 保存原始地图用于比较
    cv::Mat_<int8_t> original_map = map.clone();
    
    cv::LUT(map, LookupTableImpl, map);
    cv::Mat wall = map == 100;
    cv::dilate(wall, wall, 
              cv::getStructuringElement(cv::MORPH_ELLIPSE, 
                                      cv::Size(2 * DILATE_RADIUS + 1, 2 * DILATE_RADIUS + 1), 
                                      cv::Point(DILATE_RADIUS, DILATE_RADIUS)));
    wall &= 1;
    wall.convertTo(wall, CV_8S);
    wall = -wall;
    map &= ~wall;
    map |= wall & 100;
    
    // 发布完整地图
    std::vector<int8_t> data;
    data.assign(map.data, map.data + map.total());
    pubData.data = data;
    mapPub.publish(pubData);
    
    // 发布地图更新
    publishMapUpdate(map, subData);

#if 0
    std::string filename = std::to_string(subData.info.map_load_time.toSec());
    try
    {
        cv::imwrite(ImageLogPath + filename + ".png", map);
    }
    catch (cv::Exception e)
    {
        ROS_WARN_STREAM(e.what());
    }
#endif
}

int main(int argc, char *argv[])
{
    for (int i = 0; i < 256; i++)
    {
        if (i >= 51 && i <= 100)
            LookupTableImpl[i] = WALL;
        else if (i <= 50 && i >= 0)
            LookupTableImpl[i] = LAND;
        else
            LookupTableImpl[i] = UNKNOWN;
    }
    
    ros::init(argc, argv, "map_after_inflation");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    
    std::string mapSubTopic, mapPubTopic;
    n.param("map_sub_topic", mapSubTopic, MapSubTopic);
    n.param("map_pub_topic", mapPubTopic, MapPubTopic);
    n.param("radius", DILATE_RADIUS, DILATE_RADIUS);
    
    ROS_INFO_STREAM("Dilate radius: " << DILATE_RADIUS);
    
    ros::Subscriber mapSub = n.subscribe(mapSubTopic, 1, mapCallback);
    mapPub = n.advertise<nav_msgs::OccupancyGrid>(mapPubTopic, 1);
    mapUpdatePub = n.advertise<map_msgs::OccupancyGridUpdate>(MapUpdateTopic, 1);  // 初始化更新发布器
    
    ros::spin();
    return 0;
}