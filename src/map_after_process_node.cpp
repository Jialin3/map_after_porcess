#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <opencv2/opencv.hpp>

const std::string MapSubTopic = "/map_grid_node/map";
const std::string MapPubTopic = "/map_after_process";
const std::string MapUpdateTopic = "/map_after_process_updates";

const int8_t FREE = 0;      // 空闲
const int8_t OCCUPIED = 100; // 占据
const int8_t UNKNOWN = 255; // 未知（Costmap 中用 255 表示）

int DILATE_RADIUS = 0;      // 膨胀半径

ros::Publisher mapPub;
ros::Publisher mapUpdatePub;

std::vector<int8_t> LookupTableImpl(256); // 查找表

// 初始化查找表
void initLookupTable() {
    for (int i = 0; i < 256; i++) {
        if (i == -1) {
            LookupTableImpl[i + 128] = UNKNOWN; // 处理 -1（实际存储为 255）
        } else if (i >= 0 && i <= 20) {
            LookupTableImpl[i] = FREE;
        } else if (i >= 21 && i <= 64) {
            LookupTableImpl[i] = UNKNOWN;
        } else if (i >= 65 && i <= 100) {
            LookupTableImpl[i] = OCCUPIED;
        } else {
            LookupTableImpl[i] = UNKNOWN; // 其他值（如 101-255）也视为未知
        }
    }
}

// 发布地图更新
void publishMapUpdate(const cv::Mat_<int8_t>& map, const nav_msgs::OccupancyGrid& original_map) {
    map_msgs::OccupancyGridUpdate update;
    update.header = original_map.header;
    update.x = 0;
    update.y = 0;
    update.width = original_map.info.width;
    update.height = original_map.info.height;
    update.data.assign(map.data, map.data + map.total());
    mapUpdatePub.publish(update);
}

void mapCallback(const nav_msgs::OccupancyGrid &subData) {
    nav_msgs::OccupancyGrid pubData;
    pubData.header = subData.header;
    pubData.info = subData.info;

    // 将地图数据转换为 OpenCV 矩阵
    cv::Mat_<int8_t> map(subData.info.height, subData.info.width);
    std::memcpy(map.data, &subData.data[0], subData.data.size());

    // 使用查找表转换值
    cv::LUT(map, LookupTableImpl, map);

    // 对障碍物区域进行膨胀（可选）
    if (DILATE_RADIUS > 0) {
        cv::Mat wall = (map == OCCUPIED);
        cv::dilate(wall, wall, 
                  cv::getStructuringElement(cv::MORPH_ELLIPSE, 
                                          cv::Size(2 * DILATE_RADIUS + 1, 2 * DILATE_RADIUS + 1)));
        map.setTo(OCCUPIED, wall);
    }

    // 发布完整地图
    pubData.data.assign(map.data, map.data + map.total());
    mapPub.publish(pubData);

    // 发布地图更新
    publishMapUpdate(map, subData);
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "map_after_process");
    ros::NodeHandle n("~");

    // 初始化查找表
    initLookupTable();

    // 读取参数
    n.param("radius", DILATE_RADIUS, DILATE_RADIUS);
    ROS_INFO_STREAM("Dilate radius: " << DILATE_RADIUS);

    // 订阅和发布
    ros::Subscriber mapSub = n.subscribe(MapSubTopic, 1, mapCallback);
    mapPub = n.advertise<nav_msgs::OccupancyGrid>(MapPubTopic, 1);
    mapUpdatePub = n.advertise<map_msgs::OccupancyGridUpdate>(MapUpdateTopic, 1);

    ros::spin();
    return 0;
}