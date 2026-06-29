#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <ignition/math/Color.hh>

namespace gazebo
{
class TrafficLightCyclePlugin : public ModelPlugin
{
  public: void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    this->model = std::move(model);
    this->redLinkName = this->ReadString(sdf, "red_link", "light_source");
    this->redVisualName = this->ReadString(sdf, "red_visual", "red_lamp");
    this->greenLinkName = this->ReadString(sdf, "green_link", "light_source2");
    this->greenVisualName = this->ReadString(sdf, "green_visual", "green_lamp");
    this->redDuration = this->ReadDouble(sdf, "red_duration", 35.0);
    this->greenDuration = this->ReadDouble(sdf, "green_duration", 35.0);

    this->redLink = this->model->GetLink(this->redLinkName);
    this->greenLink = this->model->GetLink(this->greenLinkName);
    if (!this->redLink || !this->greenLink)
    {
      gzerr << "[TrafficLightCyclePlugin] Link bulunamadi, model="
            << this->model->GetName() << "\n";
      return;
    }

    this->node.reset(new transport::Node());
    this->node->Init(this->model->GetWorld()->Name());
    this->visualPublisher =
        this->node->Advertise<msgs::Visual>("~/visual", 10);

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&TrafficLightCyclePlugin::OnUpdate, this));

    gzmsg << "[TrafficLightCyclePlugin] " << this->model->GetName()
          << " aktif: " << this->redDuration << "s kirmizi, "
          << this->greenDuration << "s yesil\n";
  }

  private: void OnUpdate()
  {
    if (!this->visualPublisher)
      return;

    const double cycleDuration = this->redDuration + this->greenDuration;
    if (cycleDuration <= 0.0)
      return;

    const double simTime = this->model->GetWorld()->SimTime().Double();
    const bool redActive =
        std::fmod(std::max(0.0, simTime), cycleDuration) < this->redDuration;
    const bool stateChanged =
        !this->initialized || redActive != this->lastRedActive;

    if (
        !stateChanged
        && simTime - this->lastPublishTime < 0.5
    )
    {
      return;
    }

    this->PublishVisual(
        this->redLink,
        this->redVisualName,
        redActive ? ignition::math::Color(1.0, 0.0, 0.0, 1.0)
                  : ignition::math::Color(0.03, 0.0, 0.0, 1.0));
    this->PublishVisual(
        this->greenLink,
        this->greenVisualName,
        redActive ? ignition::math::Color(0.0, 0.03, 0.0, 1.0)
                  : ignition::math::Color(0.0, 1.0, 0.0, 1.0));

    if (stateChanged)
    {
      gzmsg << "[TrafficLightCyclePlugin] " << this->model->GetName()
            << " durum=" << (redActive ? "KIRMIZI" : "YESIL") << "\n";
    }

    this->initialized = true;
    this->lastRedActive = redActive;
    this->lastPublishTime = simTime;
  }

  private: void PublishVisual(
      const physics::LinkPtr &link,
      const std::string &visualName,
      const ignition::math::Color &color)
  {
    const std::string scopedVisualName =
        link->GetScopedName() + "::" + visualName;
    msgs::Visual visual = link->GetVisualMessage(scopedVisualName);
    if (!visual.has_name())
    {
      gzerr << "[TrafficLightCyclePlugin] Visual bulunamadi: "
            << this->model->GetName() << "::" << link->GetName()
            << "::" << visualName << "\n";
      return;
    }

    auto *material = visual.mutable_material();
    msgs::Set(material->mutable_ambient(), color);
    msgs::Set(material->mutable_diffuse(), color);
    msgs::Set(material->mutable_emissive(), color);
    msgs::Set(
        material->mutable_specular(),
        ignition::math::Color(
            color.R() * 0.4,
            color.G() * 0.4,
            color.B() * 0.4,
            1.0));
    this->visualPublisher->Publish(visual);
  }

  private: static std::string ReadString(
      const sdf::ElementPtr &sdf,
      const std::string &name,
      const std::string &fallback)
  {
    return sdf->HasElement(name) ? sdf->Get<std::string>(name) : fallback;
  }

  private: static double ReadDouble(
      const sdf::ElementPtr &sdf,
      const std::string &name,
      double fallback)
  {
    return sdf->HasElement(name) ? sdf->Get<double>(name) : fallback;
  }

  private: physics::ModelPtr model;
  private: physics::LinkPtr redLink;
  private: physics::LinkPtr greenLink;
  private: transport::NodePtr node;
  private: transport::PublisherPtr visualPublisher;
  private: event::ConnectionPtr updateConnection;
  private: std::string redLinkName;
  private: std::string redVisualName;
  private: std::string greenLinkName;
  private: std::string greenVisualName;
  private: double redDuration = 35.0;
  private: double greenDuration = 35.0;
  private: double lastPublishTime = -1.0;
  private: bool initialized = false;
  private: bool lastRedActive = false;
};

GZ_REGISTER_MODEL_PLUGIN(TrafficLightCyclePlugin)
}
