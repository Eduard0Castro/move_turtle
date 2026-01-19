#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/kill.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "move_turtle_interfaces/action/move_turtle.hpp"

using MoveTurtle = move_turtle_interfaces::action::MoveTurtle;
using LifecycleCallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;  

namespace TurtleFinal{

    class TurtleControl : public rclcpp_lifecycle::LifecycleNode {

        public:

            TurtleControl(const rclcpp::NodeOptions &options);

        private:

            rclcpp::CallbackGroup::SharedPtr callback_group;
            std::string turtle_name, killed_name;

            std::shared_ptr<rclcpp::Client<turtlesim::srv::Spawn>> client_spawner;
            std::shared_ptr<rclcpp::Client<turtlesim::srv::Kill>> client_killer;

            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
            rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscriber;

            rclcpp_action::Server<MoveTurtle>::SharedPtr move_server;
            std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveTurtle>> goal_handle;
            
            std::mutex mutex;

            bool activation = false;
            
            float linear_velocity, angular_velocity;

            void turtle_pose_callback(turtlesim::msg::Pose::SharedPtr msg);

            /**
             * @brief Sends a spawn request to turtlesim.
             */
            void spawn_turtle(float x, float y, float theta);
            void kill_turtle(std::string name);
            void callback_spawner(rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture future);

            rclcpp_action::GoalResponse goal_callback(const rclcpp_action::GoalUUID &uuid,
                                                    std::shared_ptr<const MoveTurtle::Goal> goal);
            rclcpp_action::CancelResponse cancel_callback(const std::shared_ptr<rclcpp_action::
                                                        ServerGoalHandle<MoveTurtle>> goal_handle);
            void handle_goal_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveTurtle>> goal_handle);

            LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state);
            LifecycleCallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state);
            LifecycleCallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state);
            LifecycleCallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state);
            LifecycleCallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state);
                                                        
    };
}