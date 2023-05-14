#ifndef SYSTEM_PLUGIN_FIREFLYLED_HH_
#define SYSTEM_PLUGIN_FIREFLYLED_HH_

// The only required include in the header is this one.
// All others will depend on what your plugin does.
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
// It's good practice to use a custom namespace for your project.
namespace firefly_led
{
  using namespace gz;
  using namespace sim;
  using namespace systems;
  // This is the main plugin's class. It must inherit from System and at least
  // one other interface.
  // Here we use `ISystemPostUpdate`, which is used to get results after
  // physics runs. The opposite of that, `ISystemPreUpdate`, would be used by
  // plugins that want to send commands.
  class FireflyLED:
    public System,
    public ISystemConfigure,
    public ISystemPostUpdate
  {
    // Plugins inheriting ISystemPostUpdate must implement the PostUpdate
    // callback. This is called at every simulation iteration after the physics
    // updates the world. The _info variable provides information such as time,
    // while the _ecm provides an interface to all entities and components in
    // simulation.
    public: 
    
    virtual void Configure(
                  const Entity &_entity,
                  const std::shared_ptr<const sdf::Element> &_sdf,
                  EntityComponentManager &_ecm,
                  EventManager &_eventMgr) override;

    virtual void PostUpdate(const gz::sim::UpdateInfo &_info,
                const gz::sim::EntityComponentManager &_ecm) override;


    private: 
      Entity linkEntity;
  };
}
#endif