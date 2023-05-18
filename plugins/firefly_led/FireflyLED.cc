#include <string>
#include <gz/common/Console.hh>
#include <gz/sim/components/Material.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/sim/components/Light.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/VisualCmd.hh>
#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/sim/rendering/Events.hh>
#include <gz/math/Rand.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Util.hh>

#include <gz/math/Rand.hh>
#include "FireflyLED.hh"
#include "FireflyLED.hh"


GZ_ADD_PLUGIN(
    firefly_led::FireflyLED,
    gz::sim::System,
    firefly_led::FireflyLED::ISystemConfigure,
    firefly_led::FireflyLED::ISystemPreUpdate
    )

GZ_ADD_PLUGIN_ALIAS(firefly_led::FireflyLED,"FireflyLED")
using namespace firefly_led;

void FireflyLED::PreUpdate(const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager & _ecm)
{
  auto material = _ecm.Component<components::Material>(this->visualEntity);
  material->Data().SetEmissive(isOn ? ledColor : OG_MAT_EMISSIVE); // by reference
  material->Data().SetDiffuse(isOn ? ledColor : OG_MAT_EMISSIVE); // by reference
  _ecm.SetChanged(this->visualEntity, gz::sim::components::Material::typeId,
        gz::sim::ComponentState::OneTimeChange);
  if (stateChanged) {   
    gz::sim::EventManager mgr;
    mgr.Emit<gz::sim::events::ForceRender>();
    stateChanged = false;
  }
}

void FireflyLED::Configure(const Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         gz::sim::EntityComponentManager &_ecm,
                         EventManager& _eventMgr) 
{
  // configure topics based on absolute scoped name
  modeTopic = gz::sim::scopedName(_entity, _ecm) + modeTopic;
  colorTopic = gz::sim::scopedName(_entity, _ecm) + colorTopic;

  modelEntity = _entity;
  // Read property from SDF
  auto modelName = _ecm.Component<components::Name>(_entity)->Data();
  auto linkName = _sdf->Get<std::string>("link_name");
  this->visualName = _sdf->Get<std::string>("visual_name");
  this->matName = modelName + this->matName;

  this->visualRenderName = modelName + "::" + linkName;

  // Get link entity by querying for an entity that has a specific set of
  // components
  linkEntity = _ecm.EntityByComponents(
      components::ParentEntity(_entity),
      components::Name(linkName), components::Link());
  if (linkEntity == kNullEntity) {
    gzerr << "Failed to find link" << std::endl;
    return;
  }

  // Inside the link, we expect a visual tag
  this->visualEntity = _ecm.EntityByComponents(
      components::ParentEntity(linkEntity), components::Visual());
  
  if (visualEntity == kNullEntity) {
    gzerr << "Failed to find visual" << std::endl;
    return;
  }

  // Get Material tag: <material>...
  auto material = _ecm.Component<components::Material>(this->visualEntity)->Data();
  
  this->connection = _eventMgr.Connect<gz::sim::events::PreRender>(
    std::bind(&FireflyLED::PerformRenderingOperations, this));

  this->node.Subscribe<FireflyLED, gz::msgs::Empty>(this->modeTopic, &FireflyLED::switchMode, 
                                          this, gz::transport::SubscribeOptions());

  this->node.Subscribe<FireflyLED, gz::msgs::Color>(this->modeTopic, &FireflyLED::switchColor, 
                                          this, gz::transport::SubscribeOptions());
  gzdbg<< "[LED]: Subscribed on: " << modeTopic << std::endl;
  gzdbg<< "[LED]: Subscribed on: " << colorTopic << std::endl;

}

void FireflyLED::switchMode(const gz::msgs::Empty& _msg)
{
  gzdbg<< "[LED]: Switched Mode" << std::endl;

  isOn = !isOn; // We will switchMode on the next iteration
  stateChanged = true;
}

void FireflyLED::switchColor(const gz::msgs::Color& _msg)
{
  this->ledColor.Set(_msg.r(), _msg.g(), _msg.b(), _msg.a());
  stateChanged = true;
}

void FireflyLED::PerformRenderingOperations()
{
  if (nullptr == this->scene)
  {
    this->FindScene();
  }

  if (nullptr == this->scene)
    gzdbg << "[LED]: Unable to locate Scene." << std::endl;

  auto visual = this->scene->VisualByName(visualRenderName);
  if (visual == nullptr) {
    gzdbg << "Can't find visual by Name: " << visual->Name() << std::endl;
    return;
  }
  gz::rendering::MaterialPtr material = visual->Material();

  if (material != nullptr) {
    this->scene->UnregisterMaterial(matName);
    this->scene->DestroyMaterial(material);
  }
  // ORGE 1.x wants explictly swapping of materials. Yikes.
  common::Material tmpMat;
  tmpMat.SetEmissive(isOn ? ledColor : OG_MAT_EMISSIVE);
  tmpMat.SetDiffuse(isOn ? ledColor : OG_MAT_EMISSIVE);

  auto tmpMatPtr = scene->CreateMaterial(tmpMat);
  scene->RegisterMaterial(matName, tmpMatPtr);
  visual->SetMaterial(tmpMatPtr);
  
}

void FireflyLED::FindScene()
{
  auto loadedEngNames = gz::rendering::loadedEngines();
  if (loadedEngNames.empty())
  {
    gzdbg << "No rendering engine is loaded yet" << std::endl;
    return;
  }

  // assume there is only one engine loaded
  auto engineName = loadedEngNames[0];
  for (std::string name : loadedEngNames)  
    gzdbg << "[LED] Engine: " << name << std::endl;
    
  if (loadedEngNames.size() > 1)
  {
    gzdbg << "More than one engine is available. "
      << "Using engine [" << engineName << "]" << std::endl;
  }
  auto engine = gz::rendering::engine(engineName);
  if (!engine)
  {
    gzerr << "Internal error: failed to load engine [" << engineName
      << "]. Grid plugin won't work." << std::endl;
    return;
  }

  if (engine->SceneCount() == 0)
  {
    gzdbg << "No scene has been created yet" << std::endl;
    return;
  }

  // Get first scene
  auto scenePtr = engine->SceneByIndex(0);
  if (nullptr == scenePtr)
  {
    gzerr << "Internal error: scene is null." << std::endl;
    return;
  }

  if (engine->SceneCount() > 1)
  {
    gzdbg << "More than one scene is available. "
      << "Using scene [" << scene->Name() << "]" << std::endl;
  }

  if (!scenePtr->IsInitialized() || nullptr == scenePtr->RootVisual())
  {
    return;
  }

  this->scene = scenePtr;
}