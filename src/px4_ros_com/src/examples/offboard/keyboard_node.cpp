#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8.hpp>
#include <termios.h>
#include <unistd.h>

class KeyboardNode : public rclcpp::Node
{
public:
    KeyboardNode() : Node("keyboard_node")
    {
        pub_ = create_publisher<std_msgs::msg::Int8>(
            "/mission_control", 10);

        set_terminal_raw();
        timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&KeyboardNode::read_key, this));
    }

    ~KeyboardNode()
    {
        restore_terminal();
    }

private:
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    struct termios old_tio_;

    void set_terminal_raw()
    {
        struct termios new_tio;
        tcgetattr(STDIN_FILENO, &old_tio_);
        new_tio = old_tio_;
        new_tio.c_lflag &= (~ICANON & ~ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);
    }

    void restore_terminal()
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &old_tio_);
    }

    void read_key()
    {
        char c;
        if (read(STDIN_FILENO, &c, 1) < 0) return;

        std_msgs::msg::Int8 msg;

        switch (c)
        {
        case 's':
            msg.data = 1;  // start
            break;
        case 'p':
            msg.data = 2;  // pause
            break;
        case 'a':
            msg.data = 3;  // abort
            break;
        case 'l':
            msg.data = 4;  // land
            break;
        default:
            return;
        }

        pub_->publish(msg);
        RCLCPP_INFO(get_logger(), "Key pressed: %c", c);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KeyboardNode>());
    rclcpp::shutdown();
    return 0;
}
