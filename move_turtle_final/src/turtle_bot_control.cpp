#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/twist.hpp"

#include "move_turtle_interfaces/action/move_turtle.hpp"

using MoveTurtle = move_turtle_interfaces::action::MoveTurtle;
using LifecycleCallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using namespace std::placeholders;  


namespace TurtleBotFinal{

    class TurtleBotControl : public rclcpp_lifecycle::LifecycleNode {

        public:

        TurtleBotControl(const rclcpp::NodeOptions &options): rclcpp_lifecycle::LifecycleNode("turtle_bot_control", options){

            this->callback_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);


            RCLCPP_INFO(this->get_logger(), "Turtle bot control node has been started");
        }

        private:

            rclcpp::CallbackGroup::SharedPtr callback_group;
            std::string turtle_name, killed_name;

            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;

            rclcpp_action::Server<MoveTurtle>::SharedPtr move_server;
            std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveTurtle>> goal_handle;
            
            std::mutex mutex;

            bool activation = false;
            

            rclcpp_action::GoalResponse goal_callback(const rclcpp_action::GoalUUID &uuid,
                                                    std::shared_ptr<const MoveTurtle::Goal> goal){

                (void) uuid;

                RCLCPP_INFO(this->get_logger(), "Goal received");

                if(goal->duration_sec < 0 || abs(goal->linear_vel_x) > 3.0 || abs(goal->angular_vel_z) > 3.0){

                    RCLCPP_WARN(this->get_logger(), "Invalid goal parameters. Rejecting current request");
                    return rclcpp_action::GoalResponse::REJECT;
                }

                {
                    std::lock_guard<std::mutex> lock(this->mutex);
                    if(this->goal_handle && this->goal_handle->is_active()) {
                        RCLCPP_WARN(this->get_logger(), "Another goal is active. Rejecting current request");
                        return rclcpp_action::GoalResponse::REJECT;
                    }
                }
                
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            
            }


            rclcpp_action::CancelResponse cancel_callback(const std::shared_ptr<rclcpp_action::
                                                        ServerGoalHandle<MoveTurtle>> goal_handle){

                RCLCPP_INFO(this->get_logger(), "Cancel request received");
                (void) goal_handle;

                return rclcpp_action::CancelResponse::ACCEPT;
            }




            void handle_goal_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveTurtle>> goal_handle){

                RCLCPP_INFO(this->get_logger(), "Executing the goal");
                this->goal_handle = goal_handle;

                std::shared_ptr<const MoveTurtle::Goal> goal = goal_handle->get_goal();

                rclcpp::Clock steady_clock(RCL_STEADY_TIME);
                rclcpp::Time start_time = steady_clock.now();
                rclcpp::Duration duration = rclcpp::Duration::from_seconds(goal->duration_sec);

                geometry_msgs::msg::Twist msg;
                msg.linear.x = goal->linear_vel_x;
                msg.angular.z = goal->angular_vel_z;


                // Two ways, same result:
                auto feedback = std::make_shared<MoveTurtle::Feedback>();
                MoveTurtle::Result::SharedPtr result = std::make_shared<MoveTurtle::Result>();

                
                
                while (rclcpp::ok() && (steady_clock.now() - start_time < duration)){

                    RCLCPP_INFO(this->get_logger(), "Linear velocity: %f; Angular velocity: %f", 
                                                    goal->linear_vel_x, 
                                                    goal->angular_vel_z);

                    this->vel_pub->publish(msg);
                    {
                        std::lock_guard<std::mutex> lock(this->mutex);
                        feedback->linear_velocity = goal->linear_vel_x;
                        feedback->angular_velocity = goal->angular_vel_z;
                    }
                    goal_handle->publish_feedback(feedback);
                    rclcpp::sleep_for(std::chrono::milliseconds(10));

                }

                msg.linear.x = 0.0;
                msg.angular.z = 0.0;

                this->vel_pub->publish(msg);

                result->success = true;
                result->message = "Goal complete";

                RCLCPP_INFO(this->get_logger(), "Goal complete!");

                goal_handle->succeed(result);
                
            };

            LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state){


                (void) previous_state;
                RCLCPP_INFO(this->get_logger(), "Configuring lifecycle node");

                return LifecycleCallbackReturn::SUCCESS;


            }


            LifecycleCallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state){

                (void) previous_state;

                RCLCPP_INFO(this->get_logger(), "Destroying settings");
                
                return LifecycleCallbackReturn::SUCCESS;
            }
            
            LifecycleCallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state){

                RCLCPP_INFO(this->get_logger(), "Creating move turtle action server");

                rclcpp_lifecycle::LifecycleNode::on_activate(previous_state);
                rclcpp::QoS qos_profile(10);
                this -> vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", qos_profile);
                this->move_server = rclcpp_action::create_server<MoveTurtle>(this, 
                                    "move_turtle_bot",
                                    std::bind(&TurtleBotControl::goal_callback, this, _1, _2),
                                    std::bind(&TurtleBotControl::cancel_callback, this, _1),
                                    [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveTurtle>> goal_handle){
                                        this->handle_goal_callback(goal_handle);}, 
                                    rcl_action_server_get_default_options(), 
                                    this->callback_group);

                return LifecycleCallbackReturn::SUCCESS;

            }
            LifecycleCallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state){

                RCLCPP_INFO(this->get_logger(), "Destroying move turtle action server");

                rclcpp_lifecycle::LifecycleNode::on_deactivate(previous_state);
                this->move_server.reset();
                return LifecycleCallbackReturn::SUCCESS;
            }

                                                        
    };
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(TurtleBotFinal::TurtleBotControl);