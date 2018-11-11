#ifndef _GAZEBO_CONTACT_PLUGIN_H_
#define _GAZEBO_CONTACT_PLUGIN_H_

#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>

namespace gazebo
{
  class ContactPlugin : public SensorPlugin
  {
    public:
      ContactPlugin();
      virtual ~ContactPlugin();
      virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);
    private:
      virtual void OnUpdate();
      sensors::ContactSensorPtr parentSensor;
      event::ConnectionPtr updateConnection;
  };
}
#endif
