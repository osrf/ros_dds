namespace rclcpp
{
    namespace publisher
    {
        template <typename T>
        class Publisher
        {
        public:
            Publisher();
            ~Publisher();

            void publish(T msg);
        };
    }
}
