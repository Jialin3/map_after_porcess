#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <opencv2/opencv.hpp>

class MapConverter {
public:
  MapConverter() : nh_("~") {
    // 参数设置
    nh_.param("map_sub_topic", map_sub_topic_, std::string("/map_grid_node/map"));
    nh_.param("map_pub_topic", map_pub_topic_, std::string("/map_after_process"));
    nh_.param("map_update_topic", map_update_topic_, std::string("/map_after_process_updates"));
    nh_.param("dilate_radius", dilate_radius_, 0);

    ROS_INFO("Dilate radius set to: %d", dilate_radius_);

    // 初始化发布订阅
    map_sub_ = nh_.subscribe(map_sub_topic_, 1, &MapConverter::mapCallback, this);
    map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(map_pub_topic_, 1);
    map_update_pub_ = nh_.advertise<map_msgs::OccupancyGridUpdate>(map_update_topic_, 1);
  }

  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    // 转换原始地图数据到OpenCV格式
    cv::Mat map(msg->info.height, msg->info.width, CV_8SC1);
    memcpy(map.data, &msg->data[0], msg->data.size());

    // 创建输出地图
    cv::Mat output_map = map.clone();

    // 值转换
    for (int y = 0; y < map.rows; ++y) {
      for (int x = 0; x < map.cols; ++x) {
        int8_t val = map.at<int8_t>(y, x);
        
        if (val == -1) {
          output_map.at<int8_t>(y, x) = -1;  // 保持-1表示未知
        } else if (val >= 0 && val <= 20) {
          output_map.at<int8_t>(y, x) = 0;    // 空闲区域
        } else if (val >= 21 && val <= 64) {
          output_map.at<int8_t>(y, x) = -1;   // 未知区域
        } else if (val >= 65 && val <= 100) {
          output_map.at<int8_t>(y, x) = 100;  // 占据区域
        } else {
          output_map.at<int8_t>(y, x) = -1;   // 其他值视为未知
        }
      }
    }

    // 障碍物膨胀处理
    if (dilate_radius_ > 0) {
      cv::Mat occupied = (output_map == 100);
      cv::Mat kernel = cv::getStructuringElement(
          cv::MORPH_ELLIPSE, 
          cv::Size(2 * dilate_radius_ + 1, 2 * dilate_radius_ + 1),
          cv::Point(dilate_radius_, dilate_radius_));
      cv::dilate(occupied, occupied, kernel);
      output_map.setTo(100, occupied);
    }

    // 发布完整地图
    nav_msgs::OccupancyGrid pub_msg;
    pub_msg.header = msg->header;
    pub_msg.info = msg->info;
    pub_msg.data.assign(output_map.datastart, output_map.dataend);
    map_pub_.publish(pub_msg);

    // 发布地图更新
    map_msgs::OccupancyGridUpdate update_msg;
    update_msg.header = msg->header;
    update_msg.x = 0;
    update_msg.y = 0;
    update_msg.width = msg->info.width;
    update_msg.height = msg->info.height;
    update_msg.data.assign(output_map.datastart, output_map.dataend);
    map_update_pub_.publish(update_msg);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber map_sub_;
  ros::Publisher map_pub_;
  ros::Publisher map_update_pub_;
  
  std::string map_sub_topic_;
  std::string map_pub_topic_;
  std::string map_update_topic_;
  int dilate_radius_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "map_converter");
  MapConverter converter;
  ros::spin();
  return 0;
}