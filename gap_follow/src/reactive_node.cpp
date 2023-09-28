#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
/// CHECK: include needed ROS msg type headers and libraries

// Placeholders for binding functions for pub/sub
using std::placeholders::_1;

class ReactiveFollowGap : public rclcpp::Node {
// Implement Reactive Follow Gap on the car
// This is just a template, you are free to implement your own node!

public:
    ReactiveFollowGap() : Node("reactive_node")
    {
        // Create ROS subscribers and publishers
        m_sub_scan  = this->create_subscription<sensor_msgs::msg::LaserScan>(
        lidarscan_topic, 10, std::bind(&ReactiveFollowGap::lidar_callback, this, _1));
        m_pub_drive = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
        drive_topic, 10);

        // Set the parameters
        m_panic_dist = 1.0;              // WHen to greedily go towards furthest point
        m_gap_thresh = 5;                // To find the best gap
        m_consecutive_hits = 3;
        m_lidar_smoothing_kern_size = 3; // For preprocessing lidar
        m_far_thresh = 3.0f;                // Threshold for meters when something wont be considered
    }

private:
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";

    // Parameters
    float m_gap_thresh;
    float m_consecutive_hits;

    // For safety
    int m_safety_bubble_rad;
    float m_panic_dist;

    // For filtering lidar
    int m_lidar_smoothing_kern_size;
    float m_far_thresh;

    // Publisher and Subscriber
    // We want to publish the control (steering angle)
    // We want to read the sensor measurements from the lidar
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_sub_scan;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr m_pub_drive;

    std::vector<float> preprocess_lidar(std::vector<float> ranges)
    {   
        // Preprocess the LiDAR scan array. Expert implementation includes:
        // 1.Setting each value to the mean over some window
        // 2.Rejecting high values (eg. > 3m)
        int idx = 0;
        std::vector<float> preproc_ranges;
        for(auto it = ranges.begin(); it != ranges.end(); ++it)
        {
            float min_kern = std::max(0, idx - m_lidar_smoothing_kern_size);
            float max_kern = std::min(idx + m_lidar_smoothing_kern_size, static_cast<int>(ranges.size()-1));

            std::vector<float> temp;
            for (int i = min_kern; i <= max_kern; ++i)
            {
                float curr_val = ranges.at(i);
                if(curr_val < m_far_thresh)
                {
                    temp.push_back(curr_val);
                }
            }

            // Average
            float filtered_val = *it;
            float sum = std::accumulate(temp.begin(), temp.end(), 0);
            if (sum > 0.0)
            {
                filtered_val = sum / temp.size();
            }

            preproc_ranges.push_back(filtered_val);

            // Increment element of interest
            idx++;
        }

      return preproc_ranges;      
    }


    std::vector<int> find_max_gap(std::vector<float> ranges)
    {   
        // Return the start index & end index of the max gap in free_space_ranges
        int length = 0;
        int best_length = -1;
        int end_idx = -1;
        for (auto it = ranges.begin(); it != ranges.end(); ++it)
        {
            // Increment count
            if (*it != 0.0)
            {
                length++;
            }
            // Reset count
            else
            {
                length = 0;
            }

            if(length > best_length)
            {
                best_length = length;
                end_idx = std::distance(std::begin(ranges), it);
            }
        }
        // Just use the endpoint and best length as intuition
        int start_idx = end_idx - best_length;

        RCLCPP_INFO(get_logger(), start_idx);

        return std::vector<int>{start_idx, end_idx};
    }

    int find_best_point(std::vector<float> ranges, std::vector<int> gap_idxs, int min_sfty_idx, int max_sfty_idx, float closest_dist)
    {   
        // Start_i & end_i are start and end indicies of max-gap range, respectively
        // Return index of best point in ranges
	    // Naive: Choose the furthest point within ranges and go there

        // Find the furthest point in the gap
        int furthest_idx = 0;
        float furthest_dist = -1;
        for(int i = gap_idxs[0]; i <= gap_idxs[1]; ++i)
        {
            if (ranges.at(i) > furthest_dist)
            {
                furthest_dist = ranges.at(i);
                furthest_idx  = i;
            }
        }

        // See which side of the buble we are closest to
        float dist_to_min = furthest_idx - min_sfty_idx;
        float dist_to_max = furthest_dist - max_sfty_idx;

        // Set the second reference point
        int second_pt = min_sfty_idx;
        if(std::abs(dist_to_min) > std::abs(dist_to_max))
        {
            second_pt = max_sfty_idx;
        }

        // Aim somewhere in the middle of the two points
        float panic = std::max(0.0f, closest_dist - m_panic_dist);
        float weight = panic / furthest_dist;

        return (1 - weight) * furthest_idx + weight * second_pt;
    }


    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {   
        // Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        std::vector<float> filtered_lidar = preprocess_lidar(scan_msg->ranges);

        // Find closest point to LiDAR
        auto v = std::min_element(filtered_lidar.begin(), filtered_lidar.end());
        int closest_point_idx = std::distance(std::begin(filtered_lidar), v);
        float closest_dist      = *v;

        // Eliminate all points inside 'bubble' (set them to zero) 
        int min_sfty_idx = std::max(0, closest_point_idx-m_safety_bubble_rad);
        int max_sfty_idx = std::min(static_cast<int>(filtered_lidar.size()), closest_point_idx+m_safety_bubble_rad);
        std::fill_n(filtered_lidar.begin() + min_sfty_idx, max_sfty_idx - min_sfty_idx, 0.0);

        // Find max length gap 
        std::vector<int> gap_idxs = find_max_gap(filtered_lidar);

        RCLCPP_INFO(get_logger(), "YES"); 

        // Find the best point in the gap 
        int best_idx = find_best_point(filtered_lidar, gap_idxs, min_sfty_idx, max_sfty_idx, closest_dist);       

        // Get some information about the particular laser scan
        float angle_min  = scan_msg->angle_min;
        // float angle_max  = scan_msg->angle_max;
        float angle_incr = scan_msg->angle_increment;
        // float max_range  = scan_msg->range_max;

        // Figure out what to make the steering angle
        double steering_angle = angle_min + best_idx * angle_incr;

        // Based on the velocity, calculate speed of the car
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        double velocity = 0.0;
        steering_angle = std::abs(steering_angle);
        if (steering_angle >= 0 && steering_angle < 10)
            velocity = 1.5;
        else if (steering_angle >= 10 && steering_angle < 20)
            velocity = 1.0;
        else
            velocity = 0.5;

        // std::cout << "Steering Angle: " << steering_angle << std::endl;
        // std::cout << "Car Speed: " <<velocity << std::endl;
        
        // Put the velocity in the message
        drive_msg.drive.speed = velocity;

        // Publish the message
        drive_msg.drive.steering_angle = steering_angle;
        m_pub_drive->publish(drive_msg);
    }

};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}