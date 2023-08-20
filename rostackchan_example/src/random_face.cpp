// Copyright 2023 Ar-Ray-code.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <random>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

enum class Expression
{
    Angry = 0,
    Sleepy,
    Happy,
    Sad,
    Doubt,
    Neutral
};

namespace rostackchan_example
{
    class RandomFace : public rclcpp::Node
    {
    public:
        explicit RandomFace(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
            : Node("random_move", options)
        {
            using namespace std::chrono_literals;
            this->face_common_pub_ = this->create_publisher<std_msgs::msg::Int32>("face_command", 10);
            this->face_feedback_sub_ = this->create_subscription<std_msgs::msg::Int32>(
                "face_feedback", 10,
                [this](const std_msgs::msg::Int32::SharedPtr msg) {
                    this->count_++;
                    if (this->count_ < 100)
                    {
                        return;
                    }
                    this->count_ = 0;
                    switch (msg->data)
                    {
                    case static_cast<int>(Expression::Angry):
                        RCLCPP_INFO(this->get_logger(), "Angry");
                        break;
                    case static_cast<int>(Expression::Sleepy):
                        RCLCPP_INFO(this->get_logger(), "Sleepy");
                        break;
                    case static_cast<int>(Expression::Happy):
                        RCLCPP_INFO(this->get_logger(), "Happy");
                        break;
                    case static_cast<int>(Expression::Sad):
                        RCLCPP_INFO(this->get_logger(), "Sad");
                        break;
                    case static_cast<int>(Expression::Doubt):
                        RCLCPP_INFO(this->get_logger(), "Doubt");
                        break;
                    case static_cast<int>(Expression::Neutral):
                        RCLCPP_INFO(this->get_logger(), "Neutral");
                        break;
                    default:
                        RCLCPP_INFO(this->get_logger(), "Unknown");
                        break;
                    }
                });
            this->timer_ = this->create_wall_timer(
                10s, std::bind(&RandomFace::timer_callback, this));
        }

    private:
        void timer_callback()
        {
            using namespace std::placeholders;
            std_msgs::msg::Int32 msg;
            msg.data = get_random(0, 5);
            this->face_common_pub_->publish(msg);
        }

        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr face_common_pub_;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr face_feedback_sub_;
        rclcpp::TimerBase::SharedPtr timer_;

        int count_ = 0;

        // random
        std::random_device seed_gen;
        std::mt19937 engine;
        static double get_random(double min, double max)
        {
            std::random_device seed_gen;
            std::mt19937 engine(seed_gen());
            std::uniform_real_distribution<> dist(min, max);
            return dist(engine);
        }
    };
} // namespace rostackchan_example

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rostackchan_example::RandomFace>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
