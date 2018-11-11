#include "gmm_obs/ContactPlugin.hh"

using namespace gazebo;

GZ_REGISTER_SENSOR_PLUGIN(ContactPlugin)

ContactPlugin::ContactPlugin() : SensorPlugin()
{
}

ContactPlugin::~ContactPlugin()
{
}

void ContactPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
{

  this->parentSensor =
    std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);
  if (!this->parentSensor)
  {
    gzerr << "ContactPlugin required a ContactSensor. \n";
    return;

  }

  this->updateConnection = this->parentSensor->ConnectUpdated(std::bind(&ContactPlugin::OnUpdate, this));
  this->parentSensor->SetActive(true);

}

void ContactPlugin::OnUpdate()
{
  //std::cout << "in ContactPlugin::OnUpdate()" << std::endl;
  msgs::Contacts contacts;
  contacts = this->parentSensor->Contacts();
  //std::cout<<contacts.contact_size()<<std::endl;
  
  for (int i = 0; i < contacts.contact_size(); ++i) { 
   /* std::cout << "Collision between [" << contacts.contact(i).collision1()
              << "] and [" <<contacts.contact(i).collision2() << "]"
              << std::endl;*/
    for (int j = 0; j < contacts.contact(i).position_size(); ++j) {
     /* std::cout <<j << "Position:"
                << contacts.contact(i).position(j).x() << " "
                << contacts.contact(i).position(j).y() << " "
                << contacts.contact(i).position(j).z() << std::endl;*/
      std::cout << " Forces:"
                << contacts.contact(i).wrench(j).body_1_wrench().force().x() << " "
                << contacts.contact(i).wrench(j).body_1_wrench().force().y() << " "
                << contacts.contact(i).wrench(j).body_1_wrench().force().z() << std::endl;
    //  std::cout << " Depth:" << contacts.contact(i).depth(j) << std::endl;
    }
  }
  
}
