#include <gazebo/gazebo.hh>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

namespace gazebo
{
class MyPlugin : public ModelPlugin
{
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        // Inicie o nó ROS
        ros::NodeHandle nh;

        // Crie um publisher para publicar a velocidade do vento
        m_wind_speed_pub = nh.advertise<std_msgs::Float64>("/wind", 1);

        // Conecte o callback de atualização
        m_update_connection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&MyPlugin::OnUpdate, this, _1));

        // Log de informações
        ROS_INFO("MyPlugin carregado com sucesso!");
    }

    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
        // Obtenha a velocidade do vento do Gazebo
        double wind_speed = this->m_model->GetWorld()->GetWindSpeed().Length();

        // Crie a mensagem ROS
        std_msgs::Float64 wind_speed_msg;
        wind_speed_msg.data = wind_speed;

        // Publicar a mensagem ROS a cada 1 segundo
        ros::Rate rate(1);
        rate.sleep();
        m_wind_speed_pub.publish(wind_speed_msg);
    }

    private: physics::ModelPtr m_model;
    private: ros::Publisher m_wind_speed_pub;
    private: event::ConnectionPtr m_update_connection;
};

GZ_REGISTER_MODEL_PLUGIN(MyPlugin)
}