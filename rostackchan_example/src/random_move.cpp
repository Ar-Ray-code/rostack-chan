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
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

namespace rostackchan_motions
{
    class RandomMove : public rclcpp::Node
    {
    public:
        using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
        using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

        explicit RandomMove(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
            : Node("random_move", options)
        {
            this->client_ptr_ = rclcpp_action::create_client<FollowJointTrajectory>(
                this->get_node_base_interface(),
                this->get_node_graph_interface(),
                this->get_node_logging_interface(),
                this->get_node_waitables_interface(),
                "/joint_trajectory_controller/follow_joint_trajectory");

            this->timer_ = this->create_wall_timer(
                std::chrono::milliseconds(2000),
                std::bind(&RandomMove::send_goal, this));
        }

    private:
        void send_goal()
        {
            using namespace std::placeholders;
            using namespace std::chrono_literals;

            if (!this->client_ptr_->wait_for_action_server(10s))
            {
                RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
                return;
            }

            auto goal_msg = FollowJointTrajectory::Goal();
            goal_msg.trajectory.joint_names = {"joint1", "joint2"};
            goal_msg.trajectory.points.resize(1);
            double joint1 = get_random(0.0,3.14);
            double joint2 = get_random(1.4,2.0);
            RCLCPP_INFO(this->get_logger(), "joint1: %f, joint2: %f", joint1, joint2);
            goal_msg.trajectory.points[0].positions = {joint1, joint2};
            goal_msg.trajectory.points[0].time_from_start = rclcpp::Duration(1s);

            RCLCPP_INFO(this->get_logger(), "Sending goal");
            auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
            send_goal_options.goal_response_callback =
                std::bind(&RandomMove::goal_response_callback, this, _1);
            send_goal_options.feedback_callback =
                std::bind(&RandomMove::feedback_callback, this, _1, _2);
            send_goal_options.result_callback =
                std::bind(&RandomMove::result_callback, this, _1);
            this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
        }

        void goal_response_callback(const GoalHandleFollowJointTrajectory::SharedPtr & goal_handle)
        {
            if (!goal_handle)
            {
                RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
            }
        }

        void feedback_callback(
            GoalHandleFollowJointTrajectory::SharedPtr,
            const std::shared_ptr<const FollowJointTrajectory::Feedback> feedback)
        {
            RCLCPP_DEBUG(
                this->get_logger(), "Next waypoint: %f %f",
                feedback->joint_names[0].c_str(),
                feedback->actual.positions[0]);
        }

        void result_callback(const GoalHandleFollowJointTrajectory::WrappedResult &result)
        {
            switch (result.code)
            {
            case rclcpp_action::ResultCode::SUCCEEDED:
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                return;
            }

            RCLCPP_INFO(this->get_logger(), "Result received");
        }

        rclcpp_action::Client<FollowJointTrajectory>::SharedPtr client_ptr_;
        rclcpp::TimerBase::SharedPtr timer_;

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
} // namespace rostackchan_motions

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rostackchan_motions::RandomMove>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
