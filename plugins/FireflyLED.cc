// We'll use a string and the gzmsg command below for a brief example.
// Remove these includes if your plugin doesn't need them.
#include <string>
#include <gz/common/Console.hh>
#include <gz/sim/components/Material.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/sim/components/Light.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/VisualCmd.hh>

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
    firefly_led::FireflyLED::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(firefly_led::FireflyLED,"FireflyLED")
using namespace firefly_led;


void FireflyLED::PreUpdate(const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager & _ecm)
{
  if (stateChanged) {
    // Get Material tag: <material>...  
    auto material = _ecm.Component<components::Material>(this->visualEntity);
    material->Data().SetEmissive(isOn ? ledColor : OG_MAT_EMISSIVE); // by reference
    material->Data().SetDiffuse(isOn ? ledColor : OG_MAT_EMISSIVE); // by reference

    // GZ is bad at detecting state changes
    _ecm.SetChanged(this->visualEntity, gz::sim::components::Material::typeId,
          gz::sim::ComponentState::OneTimeChange);

    

    gz::sim::components::VisualCmd visualUpdate;
    auto* newMat = new gz::msgs::Material;
    visualUpdate.Data().set_allocated_material(newMat);

    _ecm.CreateComponent(this->visualEntity, visualUpdate);

    _ecm.RemoveComponent<components::Material>(this->visualEntity);
    _ecm.CreateComponent(this->visualEntity, components::Material());

    _ecm.SetChanged(this->visualEntity, gz::sim::components::Material::typeId,
          gz::sim::ComponentState::OneTimeChange);

    _ecm.SetChanged(this->visualEntity, gz::sim::components::VisualCmd::typeId,
          gz::sim::ComponentState::OneTimeChange);

    _ecm.SetChanged(linkEntity, gz::sim::components::Visual::typeId,
          gz::sim::ComponentState::OneTimeChange);

    
    stateChanged = false;
  }
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
  linkEntity = _ecm.EntityByComponents(
      components::ParentEntity(_entity),
      components::Name(linkName), components::Link());
  if (linkEntity == kNullEntity) {
    std::cout << "Failed to find link" << std::endl;
    return;
  }

  // Inside the link, we expect a visual tag
  this->visualEntity = _ecm.EntityByComponents(
      components::ParentEntity(linkEntity), components::Visual());
  
  if (visualEntity == kNullEntity) {
    std::cout << "Failed to find visual" << std::endl;
    return;
  }

  // Get Material tag: <material>...
  auto material = _ecm.Component<components::Material>(this->visualEntity)->Data();

  this->OG_MAT_EMISSIVE = material.Emissive();

  this->node.Subscribe<FireflyLED, gz::msgs::Empty>(this->modeTopic, &FireflyLED::switchMode, 
                                          this, gz::transport::SubscribeOptions());

  this->node.Subscribe<FireflyLED, gz::msgs::Color>(this->modeTopic, &FireflyLED::switchColor, 
                                          this, gz::transport::SubscribeOptions());
}

void FireflyLED::switchMode(const gz::msgs::Empty& _msg)
{
  std::cout << "MODE" << std::endl;

  isOn = !isOn; // We will switchMode on the next iteration
  stateChanged = true;
}

void FireflyLED::switchColor(const gz::msgs::Color& _msg)
{
  this->ledColor.Set(_msg.r(), _msg.g(), _msg.b(), _msg.a());
  stateChanged = true;
}