#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "custom_interfaces/action/count_until.hpp"

using CountUntil = custom_interfaces::action::CountUntil;
using CountUntilGoalHandle = rclcpp_action::ClientGoalHandle<CountUntil>;
using namespace std::placeholders;

class CountUntilClient : public rclcpp::Node 
{
public:
    CountUntilClient() : Node("count_until_client") 
    {
        count_until_client_ = rclcpp_action::create_client<CountUntil>(this, "count_until");
    }

    void send_goal(int target_number, double period){
        count_until_client_->wait_for_action_server();

        auto goal = CountUntil::Goal();

        goal.target_number = target_number;
        goal.period = period;

        auto options = rclcpp_action::Client<CountUntil>::SendGoalOptions();
        options.result_callback = std::bind(&CountUntilClient::goal_result_callback, this, _1);
        options.goal_response_callback = std::bind(&CountUntilClient::goal_response_callback, this, _1);
        options.feedback_callback = std::bind(&CountUntilClient::goal_feedback_callback, this, _1, _2);

        RCLCPP_INFO(this->get_logger(), "Sending a goal...");
        count_until_client_->async_send_goal(goal, options);

        //timer_ = this->create_wall_timer(std::chrono::seconds(2), std::bind(&CountUntilClient::timer_callback, this));

    }
     
private:

    void timer_callback(){
        RCLCPP_INFO(this->get_logger(), "Cancel the goal");

        timer_->cancel();
    }

    void goal_response_callback(const CountUntilGoalHandle::SharedPtr &goal_handle){
        if(!goal_handle){
            RCLCPP_INFO(this->get_logger(), "Goal got rejected");
        }
        else{
            this->goal_handle_ = goal_handle;
            count_until_client_->async_cancel_goal(goal_handle);
            RCLCPP_INFO(this->get_logger(), "Goal got accepted");
        }
    }

    void goal_result_callback(const CountUntilGoalHandle::WrappedResult &result){
        auto status = result.code;
        if (status == rclcpp_action::ResultCode::SUCCEEDED){
            RCLCPP_INFO(this->get_logger(), "Succeeded");
        }
        else if (status == rclcpp_action::ResultCode::ABORTED){
            RCLCPP_ERROR(this->get_logger(), "Aborted");
        }
        else if (status == rclcpp_action::ResultCode::CANCELED){
            RCLCPP_WARN(this->get_logger(), "Cancelled");
        }
        int reached_number = result.result->reached_number;
        RCLCPP_INFO(this->get_logger(), "Result: %d", reached_number);

    }

    void goal_feedback_callback(const CountUntilGoalHandle::SharedPtr &goal_handle, const std::shared_ptr<const CountUntil::Feedback> feedback){
        int number = feedback->current_number;
        RCLCPP_INFO(this->get_logger(), "Got feedback: %d", number);
    }

    rclcpp_action::Client<CountUntil>::SharedPtr count_until_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    CountUntilGoalHandle::SharedPtr goal_handle_;

};
     
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CountUntilClient>();
    node->send_goal(6, 0.8);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}