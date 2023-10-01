#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include <sstream>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

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
        m_safety_bubble_rad = 100;         // Width of the safety bubble (number of beams)
        m_panic_dist = 0.5;//0.1;              // WHen to greedily go towards furthest point
        m_lidar_smoothing_kern_size = 2; // For preprocessing lidar
        m_far_thresh = 4.0f;//4.0f;                // Threshold for meters when something wont be considered
        m_fov = 90*M_PI/180;//90 * M_PI/180; // degrees of the field of view
    }

private:
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";

    // Parameters
    float m_gap_thresh;
    float m_consecutive_hits;
    double m_fov;

    // For safety
    int m_safety_bubble_rad;
    float m_panic_dist;

    // For filtering lidar
    unsigned int m_lidar_smoothing_kern_size;
    float m_far_thresh;

    // Publisher and Subscriber
    // We want to publish the control (steering angle)
    // We want to read the sensor measurements from the lidar
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_sub_scan;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr m_pub_drive;

    double get_minimum_fov_angle()
    {
        return -m_fov/2;
    }

    std::vector<float> preprocess_lidar(std::vector<float> ranges, unsigned int min_scan_idx, unsigned int max_scan_idx)
    {   
        // Preprocess the LiDAR scan array. Expert implementation includes:
        // 1.Setting each value to the mean over some window
        // 2.Rejecting high values (eg. > 3m)

        std::vector<float> preproc_ranges;
        for(unsigned int i = min_scan_idx; i <= max_scan_idx; ++i)
        {
            unsigned int min_kern = std::max((unsigned int)0, i - m_lidar_smoothing_kern_size);
            unsigned int max_kern = std::min(i + m_lidar_smoothing_kern_size, static_cast<unsigned int>(ranges.size()-1));
            min_kern = i-m_lidar_smoothing_kern_size;
            max_kern = i+m_lidar_smoothing_kern_size;

            // Average
            float filtered_val = ranges[i];

            // Filter out values above threshold then average
            auto v = std::vector<float>(ranges.begin() + min_kern, ranges.begin() + max_kern);
            v.erase(std::remove_if(
            v.begin(), v.end(),
            [&](const float& x) { 
                return x > m_far_thresh; // put your condition here
            }), v.end());
            float sum = std::accumulate(v.begin(), v.end(), 0);
            if (sum > 0.0)
            {
                filtered_val = sum / (v.size());
            }

            preproc_ranges.push_back(filtered_val);
        }

      return preproc_ranges;      
    }


    std::vector<int> find_max_gap(std::vector<float> ranges)
    {   
        // Return the start index & end index of the max gap in free_space_ranges
        int length = -1;
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

        // 
        RCLCPP_INFO(get_logger(), std::to_string(start_idx));
        RCLCPP_INFO(get_logger(), std::to_string(end_idx));


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

        // TODO: REMOVE TESTING THE MIDDLE OF THE GAP

        // See which side of the buble we are closest to
        float dist_to_min = furthest_idx - min_sfty_idx;
        float dist_to_max = furthest_idx - max_sfty_idx;

        // Set the second reference point
        int second_pt = min_sfty_idx;
        if(std::abs(dist_to_min) > std::abs(dist_to_max))
        {
            second_pt = max_sfty_idx;
        } 

        // Aim somewhere in the middle of the two points
        float panic = std::max(0.0f, closest_dist - m_panic_dist);
        float weight = panic / furthest_dist;

        int best_idx = (1 - weight) * furthest_idx + weight * second_pt;


        // Not sure how good this is
        // If we are really panicking and going to hit something
        // if (panic == 0.0f)
        // {
        //     // If sign is positive, obstacle is on right
        //     // If sign is negaitve
        //     RCLCPP_INFO(get_logger(), "CURRENTLY PANICKING!");
        //     int overCorrect = furthest_idx - second_pt;
        //     best_idx = best_idx + overCorrect/2;
        // }

        return best_idx;
    }


    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {   
        // Get some information about the particular laser scan
        float angle_min  = scan_msg->angle_min;
        // float angle_max  = scan_msg->angle_max;
        float angle_incr = scan_msg->angle_increment;
        // float max_range  = scan_msg->range_max;

        // Figure out the min and max idxs for the FOV
        unsigned int min_scan_idx = (-m_fov/2 - angle_min)/angle_incr;   // radians to idx
        unsigned int max_scan_idx = (m_fov/2  - angle_min) / angle_incr; // radians to idx    

        // For logging
        std::ostringstream oss;

        oss << min_scan_idx << " " << max_scan_idx << std::endl;

        // Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        std::vector<float> filtered_lidar = preprocess_lidar(scan_msg->ranges, min_scan_idx, max_scan_idx);

        // Debug filtered lidar
        // oss << "Filtered LiDAR Right Most: " << (float)filtered_lidar.at(160) << std::endl;
        // RCLCPP_INFO(get_logger(), oss.str());
        // oss << "Filtered LiDAR Left Most: " << (float)filtered_lidar.at(filtered_lidar.size()-160) << std::endl;
        // RCLCPP_INFO(get_logger(), oss.str());

        // Find closest point to LiDAR
        auto v = std::min_element(filtered_lidar.begin(), filtered_lidar.end());
        int closest_point_idx = std::distance(std::begin(filtered_lidar), v);
        float closest_dist      = *v;

        // Debug closest LiDAR angle and distance
        oss << "Angle of closest lidar beam: " << (get_minimum_fov_angle() + angle_incr*closest_point_idx) * 180/M_PI << " degrees" << std::endl;
        RCLCPP_INFO(get_logger(), oss.str());
        oss << "Closest Lidar Distance: " << closest_dist << "m" <<std::endl;
        RCLCPP_INFO(get_logger(), oss.str());

        // Adaptive safety bubble radius
        float gain = 50.0f;
        float bias = 20.0f;
        m_safety_bubble_rad = gain * std::max(0.0f, (m_far_thresh -closest_dist)) + bias

        // Eliminate all points inside 'bubble' (set them to zero) 
        int min_sfty_idx = std::max(0, closest_point_idx - m_safety_bubble_rad);
        int max_sfty_idx = std::min(static_cast<int>(filtered_lidar.size()), closest_point_idx + m_safety_bubble_rad);

        // Debug safety angles
        oss << "Minimum Safety Bubble Angle" << (get_minimum_fov_angle() + angle_incr*min_sfty_idx) * 180/M_PI << std::endl;
        RCLCPP_INFO(get_logger(), oss.str());
        oss << "Maximum Safety Bubble Angle" << (get_minimum_fov_angle() + angle_incr*max_sfty_idx) * 180/M_PI << std::endl;
        RCLCPP_INFO(get_logger(), oss.str());

        std::fill_n(filtered_lidar.begin() + min_sfty_idx, max_sfty_idx - min_sfty_idx, 0.0);

        // Find max length gap 
        std::vector<int> gap_idxs = find_max_gap(filtered_lidar);

        // Find the best point in the gap 
        int best_idx = find_best_point(filtered_lidar, gap_idxs, min_sfty_idx, max_sfty_idx, closest_dist);       
        oss << "Furthest Index: " << best_idx << std::endl;
        RCLCPP_INFO(get_logger(), oss.str());

        // Figure out what to make the steering angle
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        double steering_angle = get_minimum_fov_angle() + best_idx * angle_incr;
        drive_msg.drive.steering_angle = steering_angle;

        oss << "Steering Angle: " << steering_angle * (180/M_PI) << std::endl;
        RCLCPP_INFO(get_logger(), oss.str());

        // Based on the velocity, calculate speed of the car
        double velocity = 0.0;
        steering_angle = std::abs(steering_angle);
        if (steering_angle >= 0 && steering_angle < 10)
            velocity = 1.0;
        else if (steering_angle >= 10 && steering_angle < 20)
            velocity = 0.5;
        else
            velocity = 0.25;

        // Put the velocity in the message
        drive_msg.drive.speed = velocity;
        // std::cout << "Car Speed: " <<velocity << std::endl;

        // Publish message
        m_pub_drive->publish(drive_msg);
    }

};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}