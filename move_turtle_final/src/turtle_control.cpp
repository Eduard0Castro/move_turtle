#include "move_turtle_final/turtle_control.hpp"

using namespace std::placeholders;


namespace TurtleFinal{


    TurtleControl::TurtleControl(const rclcpp::NodeOptions &options) : rclcpp_lifecycle::LifecycleNode("turtle_control", options){

        this->callback_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        this->declare_parameter<std::string>("turtle_name", "turtle_thor");
        this->turtle_name = this->get_parameter("turtle_name").as_string();

        this->pose_subscriber = this->create_subscription<turtlesim::msg::Pose>(this->turtle_name + "/pose", 10,
                                                                std::bind(&TurtleControl::turtle_pose_callback, 
                                                                this, _1));
        
        RCLCPP_INFO(this->get_logger(), "Killing turtle1");

        this->kill_turtle("turtle1");

        RCLCPP_INFO(this->get_logger(), "Turtle control node has been started");
    }


    void TurtleControl::turtle_pose_callback(turtlesim::msg::Pose::SharedPtr msg){
        {
            std::lock_guard<std::mutex> lock(this->mutex);
            this->linear_velocity  = msg->linear_velocity;
            this->angular_velocity = msg->angular_velocity;
        }
    }

    void TurtleControl::spawn_turtle(float x, float y, float theta){

        this->client_spawner = this->create_client<turtlesim::srv::Spawn>("spawn", rmw_qos_profile_services_default,
                                                                this->callback_group);
        auto request = std::make_shared<turtlesim::srv::Spawn_Request>();

        while (!client_spawner->wait_for_service(std::chrono::seconds(1)))
            RCLCPP_WARN(this->get_logger(), "Waiting for spawn service");

        request->name = this->turtle_name;
        request->x = x;
        request->y = y;
        request->theta = theta;
        

        // Function passed with std::bind
        auto future = client_spawner->async_send_request(request, std::bind(&TurtleControl::callback_spawner, this, _1));

    }

    
    void TurtleControl::kill_turtle(std::string name){

        this->client_killer = this->create_client<turtlesim::srv::Kill>("kill", rmw_qos_profile_services_default,
                                                                    this->callback_group);
        auto request = std::make_shared<turtlesim::srv::Kill::Request>();

        request->name = name;
        this->killed_name = name;

        while (!this->client_killer->wait_for_service(std::chrono::seconds(1)))
            RCLCPP_WARN(this->get_logger(), "Waiting for turtle kill service");


        // Lambda callback example:
        this->client_killer->async_send_request(request, 
                [this](rclcpp::Client<turtlesim::srv::Kill>::SharedFuture future){
                        try{
                            const std::shared_ptr<turtlesim::srv::Kill_Response> response = future.get();
                            RCLCPP_INFO(this->get_logger(), "Turtle named %s has been killed", 
                                                            this->killed_name.c_str());

                        }

                        catch(std::exception &ex) {
                            RCLCPP_ERROR(this->get_logger(), 
                                        "An error has occured while receiving response from Kill server: %s",
                                        ex.what());}

                });
    }

    void TurtleControl::callback_spawner(rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture future){


        try{
            const std::shared_ptr<turtlesim::srv::Spawn_Response> response = future.get();
            RCLCPP_INFO(this->get_logger(), "New turtle named %s is in the map", response->name.c_str());

        }

        catch(std::exception &ex){
            RCLCPP_ERROR(this->get_logger(), 
                        "An error has occured: %s", ex.what());
        }
    }


    rclcpp_action::GoalResponse TurtleControl::goal_callback(const rclcpp_action::GoalUUID &uuid,
                                            std::shared_ptr<const MoveTurtle::Goal> goal){

        (void) uuid;

        RCLCPP_INFO(this->get_logger(), "Goal received");

        if(goal->duration_sec < 0 || abs(goal->linear_vel_x) > 10.0 || abs(goal->angular_vel_z) > 10.0){

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


    rclcpp_action::CancelResponse TurtleControl::cancel_callback(const std::shared_ptr<rclcpp_action::
                                                ServerGoalHandle<MoveTurtle>> goal_handle){

        RCLCPP_INFO(this->get_logger(), "Cancel request received");
        (void) goal_handle;

        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void TurtleControl::handle_goal_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveTurtle>> goal_handle){

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
                feedback->linear_velocity = this->linear_velocity;
                feedback->angular_velocity = this->angular_velocity;
            }
            goal_handle->publish_feedback(feedback);
            rclcpp::sleep_for(std::chrono::milliseconds(10));

        }


        result->success = true;
        result->message = "Goal complete";

        RCLCPP_INFO(this->get_logger(), "Goal complete!");

        goal_handle->succeed(result);
        

    }

    LifecycleCallbackReturn TurtleControl::on_configure(const rclcpp_lifecycle::State &previous_state){


        (void) previous_state;
        RCLCPP_INFO(this->get_logger(), "Spawing a new turtle in the map");

        double x = rand()%100/10.0;
        double y = rand()%100/10.0;

        this->spawn_turtle(x, y, 0.0);

        return LifecycleCallbackReturn::SUCCESS;


    }


    LifecycleCallbackReturn TurtleControl::on_cleanup(const rclcpp_lifecycle::State &previous_state){

        (void) previous_state;

        RCLCPP_INFO(this->get_logger(), "Killing new turtle");
        this->kill_turtle(this->turtle_name);
        
        return LifecycleCallbackReturn::SUCCESS;
    }


    LifecycleCallbackReturn TurtleControl::on_activate(const rclcpp_lifecycle::State &previous_state){

        RCLCPP_INFO(this->get_logger(), "Creating move turtle action server");

        rclcpp_lifecycle::LifecycleNode::on_activate(previous_state);
        rclcpp::QoS qos_profile(10);
        this -> vel_pub = this->create_publisher<geometry_msgs::msg::Twist>(this->turtle_name + "/cmd_vel",
                                                                            qos_profile);
        this->move_server = rclcpp_action::create_server<MoveTurtle>(this, 
                            "move_turtle/" + this->turtle_name, 
                            std::bind(&TurtleControl::goal_callback, this, _1, _2),
                            std::bind(&TurtleControl::cancel_callback, this, _1),
                            [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveTurtle>> goal_handle){
                                this->handle_goal_callback(goal_handle);}, 
                            rcl_action_server_get_default_options(), 
                            this->callback_group);

        return LifecycleCallbackReturn::SUCCESS;

    }


    LifecycleCallbackReturn TurtleControl::on_deactivate(const rclcpp_lifecycle::State &previous_state){

        RCLCPP_INFO(this->get_logger(), "Destroying move turtle action server");

        rclcpp_lifecycle::LifecycleNode::on_deactivate(previous_state);
        this->move_server.reset();
        return LifecycleCallbackReturn::SUCCESS;
    }

    // Without using namespace:
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn TurtleControl::on_shutdown(
                                                const rclcpp_lifecycle::State &previous_state){

        (void) previous_state;
        RCLCPP_INFO(this->get_logger(), "Shutting down lifecycle node");

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;

    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(TurtleFinal::TurtleControl);
