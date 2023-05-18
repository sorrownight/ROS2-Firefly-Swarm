#ifndef SYSTEM_PLUGIN_FIREFLYLED_HH_
#define SYSTEM_PLUGIN_FIREFLYLED_HH_

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/transport/Node.hh>
#include <gz/transport/SubscribeOptions.hh>
#include <gz/msgs/empty.pb.h>
#include <gz/msgs/color.pb.h>
#include <gz/rendering/Scene.hh>
namespace firefly_led
{
  using namespace gz;
  using namespace sim;
  using namespace systems;

  class FireflyLED:
    public System,
    public ISystemConfigure,
    public ISystemPreUpdate
  {
    public: 
    
    virtual void Configure(
                  const Entity &_entity,
                  const std::shared_ptr<const sdf::Element> &_sdf,
                  EntityComponentManager &_ecm,
                  EventManager &_eventMgr) override;
    virtual void PreUpdate(const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;

    // The LED will switch to on/off
    void switchMode(const gz::msgs::Empty& _msg); 

    // Change the Color as designated
    void switchColor(const gz::msgs::Color& _msg);


    private: 
      Entity visualEntity;
      Entity linkEntity;
      Entity modelEntity;
      
      std::string visualName;
      std::string visualRenderName;

      gz::common::ConnectionPtr connection;
      gz::rendering::ScenePtr scene{nullptr};

      gz::transport::Node node; // Our plugin acts sort of like a ROS Subscriber Node
      gz::msgs::Empty flashCmd;
      bool isOn = false;
      bool stateChanged = false;
      std::string modeTopic = "/LED_mode"; // TODO: Check to see if we need namespace
      std::string colorTopic = "/LED_color"; // TODO: Check to see if we need namespace
      std::string matName = "tmp_mat_led"; // TODO: Check to see if we need namespace

      gz::math::Color ledColor {gz::math::Color::Green};
      gz::math::Color OG_MAT_EMISSIVE {gz::math::Color::Blue};
      
      void FindScene();
      void PerformRenderingOperations();
  };
}
#endif