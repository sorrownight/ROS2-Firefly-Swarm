// We'll use a string and the gzmsg command below for a brief example.
// Remove these includes if your plugin doesn't need them.
#include <string>
#include <gz/common/Console.hh>
#include <gz/sim/components/Material.hh>
#include <gz/sim/components/Light.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
// This header is required to register plugins. It's good practice to place it
// in the cc file, like it's done here.
#include <gz/plugin/Register.hh>

// Don't forget to include the plugin's header.
#include "FireflyLED.hh"

// This is required to register the plugin. Make sure the interfaces match
// what's in the header.
GZ_ADD_PLUGIN(
    firefly_led::FireflyLED,
    gz::sim::System,
    firefly_led::FireflyLED::ISystemConfigure,
    firefly_led::FireflyLED::ISystemPostUpdate)

using namespace firefly_led;

// Here we implement the PostUpdate function, which is called at every
// iteration.
void FireflyLED::PostUpdate(const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager & _ecm)
{
  // Get Material tag: <material>...
  auto material = _ecm.Component<components::Material>(this->linkEntity)->Data();
  material.SetEmissive(isOn ? ledColor : OG_MAT_EMISSIVE);
}

void FireflyLED::Configure(const Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         gz::sim::EntityComponentManager &_ecm,
                         EventManager &/*_eventMgr*/) 
{
  // Read property from SDF
  auto linkName = _sdf->Get<std::string>("link_name");

  // Get link entity by querying for an entity that has a specific set of
  // components
  this->linkEntity = _ecm.EntityByComponents(
      components::ParentEntity(_entity),
      components::Name(linkName), components::Link());

  // Get Material tag: <material>...
  auto material = _ecm.Component<components::Material>(this->linkEntity)->Data();

  this->OG_MAT_EMISSIVE = material.Emissive();

  this->node.Subscribe<FireflyLED, gz::msgs::Empty>(this->modeTopic, &FireflyLED::switchMode, 
                                          this, gz::transport::SubscribeOptions());
}

void FireflyLED::switchMode(const gz::msgs::Empty& _msg)
{
  isOn = !isOn; // We will switchMode on the next iteration
}

void FireflyLED::switchColor(const gz::msgs::Color& _msg)
{
  this->ledColor.Set(_msg.r(), _msg.g(), _msg.b(), _msg.a());
}