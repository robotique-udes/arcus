class MasterNode : public rclcpp::Node
{
  public:
    MasterNode();

  private:
    void watchdog();
    void errorCodeCallback(const arcus_msgs::msg::ErrorCode::SharedPtr msg);
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<arcus_msgs::msg::ErrorCode>::SharedPtr error_listener_;
    rclcpp::Publisher<arcus_msgs::msg::ErrorCode>::SharedPtr error_publisher_;
    uint32_t heartbeats[7];
};