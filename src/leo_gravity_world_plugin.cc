#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Events.hh>
#include <ignition/math/Vector3.hh>

#include "gazebo_leo_gravity/ggm_model.hpp"

#include <string>

namespace gazebo_leo_gravity
{

class LeoGravityWorldPlugin : public gazebo::WorldPlugin
{
public:
    LeoGravityWorldPlugin() {}
    virtual ~LeoGravityWorldPlugin() {}

    // Called when the plugin is loaded
    void Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf) override
    {
        world_ = _world;

        // Get GGM05C file path from SDF parameter
        std::string ggm_file = "data/GGM05C.gfc";
        if (_sdf->HasElement("ggm_file"))
            ggm_file = _sdf->Get<std::string>("ggm_file");

        // Load GGM05C model (nmax = 50 for speed)
        if (!ggm_model_.load(ggm_file, 50))
        {
            gzerr << "[LeoGravityWorldPlugin] Failed to load GGM05C file.\n";
            return;
        }

        // Connect update event
        update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
            std::bind(&LeoGravityWorldPlugin::OnUpdate, this));
    }

    // Called every simulation iteration
    void OnUpdate()
    {
        // Iterate over all models in the world
        for (auto model : world_->Models())
        {
            // Skip static models
            if (model->IsStatic()) continue;

            // Get model position
            ignition::math::Vector3d pos = model->WorldPose().Pos();

            // Compute gravity acceleration using GGM05C
            ignition::math::Vector3d grav_acc = ggm_model_.acceleration(pos);

            // Apply Force: F = m * g
            double mass = model->GetLink()->GetInertial()->Mass();
            ignition::math::Vector3d force = grav_acc * mass;

            // Apply force to the first link
            model->GetLink()->AddForce(force);
        }
    }

private:
    gazebo::physics::WorldPtr world_;
    gazebo::event::ConnectionPtr update_connection_;
    GGMModel ggm_model_;
};

// Register plugin
GZ_REGISTER_WORLD_PLUGIN(LeoGravityWorldPlugin)

} // namespace gazebo_leo_gravity
