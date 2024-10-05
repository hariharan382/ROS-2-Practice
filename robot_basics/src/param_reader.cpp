#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>

class LocalizationNode : public rclcpp::Node
{
    public:
        LocalizationNode()
        : Node("localizationNode")
        {
            declare_parameter("number_particles", 200);
            declare_parameter("topics", std::vector<std::string>());
            declare_parameter("topic_types", std::vector<std::string>());

            get_parameter("number_particles", num_particles_);
            RCLCPP_INFO_STREAM(get_logger(), "Number of Particles" <<num_particles_);

            get_parameter("topics", topics_);
            get_parameter("topic_types", topic_types_);

            if (topics_.size() != topic_types_.size())
            {
                RCLCPP_ERROR(
                    get_logger(), "Number of topics (%zu) != number of types (%zu)",
                    topics_.size(), topics_.size());
            }else {
                RCLCPP_INFO_STREAM(get_logger(), "Number of topics: " << topics_.size());
                for(size_t i = 0; i < topics_.size(); i++) 
                {
                    RCLCPP_INFO_STREAM(get_logger(), "\t" << topics_[i] << "\t - " << topic_types_[i]);
                }
            }



        }
    private:
        int num_particles_;
        std::vector<std::string> topics_;
        std::vector<std::string> topic_types_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node =  std::make_shared<LocalizationNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
