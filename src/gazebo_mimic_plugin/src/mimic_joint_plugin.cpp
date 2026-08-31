// mimic_joint_plugin.cpp — Gazebo Classic model plugin.
//
// Faz uma junta "mimic" seguir uma junta mestre:  q_mimic = mult * q_master + off.
//   - com  <hasPID>  : um PID gera FORÇA (limitada a <maxEffort>) para rastrear
//                      o alvo — transmite torque, então a preensão segura o objeto.
//   - sem  <hasPID>  : SetPosition cinemático (barato, sem torque).
//
// SDF:
//   <plugin name="mimic_X" filename="libgazebo_mimic_joint_plugin.so">
//     <joint>MASTER</joint>
//     <mimicJoint>X</mimicJoint>
//     <multiplier>1.0</multiplier>
//     <offset>0.0</offset>
//     <sensitiveness>0.0</sensitiveness>   <!-- zona morta do erro (rad) -->
//     <hasPID><p>40</p><i>0</i><d>0.2</d></hasPID>
//     <maxEffort>1.0</maxEffort>
//   </plugin>
//
// Reescrita ROS-free de roboticsgroup/roboticsgroup_gazebo_plugins
// (BSD-3, Konstantinos Chatzilygeroudis): gazebo::common::PID no lugar de
// control_toolbox, ganhos vindos do SDF, sem roscpp.

#include <cmath>
#include <functional>
#include <string>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/PID.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

namespace gazebo
{
class GazeboMimicJointPlugin : public ModelPlugin
{
public:
  GazeboMimicJointPlugin() = default;
  ~GazeboMimicJointPlugin() override = default;

  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
  {
    this->model_ = _model;
    this->world_ = _model->GetWorld();

    if (!_sdf->HasElement("joint") || !_sdf->HasElement("mimicJoint"))
    {
      gzerr << "[mimic_joint] faltam <joint> e/ou <mimicJoint>; plugin inerte.\n";
      return;
    }
    const std::string master = _sdf->Get<std::string>("joint");
    const std::string mimic  = _sdf->Get<std::string>("mimicJoint");

    this->multiplier_    = _sdf->Get<double>("multiplier", 1.0).first;
    this->offset_        = _sdf->Get<double>("offset", 0.0).first;
    this->sensitiveness_ = _sdf->Get<double>("sensitiveness", 0.0).first;
    this->max_effort_    = _sdf->Get<double>("maxEffort", 1.0).first;

    this->master_joint_ = _model->GetJoint(master);
    this->mimic_joint_  = _model->GetJoint(mimic);
    if (!this->master_joint_)
    {
      gzerr << "[mimic_joint] junta mestre '" << master << "' nao encontrada.\n";
      return;
    }
    if (!this->mimic_joint_)
    {
      gzerr << "[mimic_joint] junta mimic '" << mimic << "' nao encontrada.\n";
      return;
    }

    this->has_pid_ = _sdf->HasElement("hasPID");
    if (this->has_pid_)
    {
      sdf::ElementPtr p = _sdf->GetElement("hasPID");
      const double kp = p->Get<double>("p", 40.0).first;
      const double ki = p->Get<double>("i", 0.0).first;
      const double kd = p->Get<double>("d", 0.2).first;
      // imax/imin = teto do termo integral; cmdMax/cmdMin = saturacao da saida.
      this->pid_ = common::PID(kp, ki, kd,
                               this->max_effort_, -this->max_effort_,
                               this->max_effort_, -this->max_effort_);
    }

    this->last_update_ = this->world_->SimTime();
    this->update_conn_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&GazeboMimicJointPlugin::OnUpdate, this));

    gzmsg << "[mimic_joint] " << mimic << " <- " << master
          << "  x" << this->multiplier_ << " +" << this->offset_
          << (this->has_pid_ ? "  (PID, maxEffort="
                               + std::to_string(this->max_effort_) + ")"
                             : "  (cinematico)")
          << "\n";
  }

private:
  void OnUpdate()
  {
    if (!this->master_joint_ || !this->mimic_joint_)
      return;

    const double target =
      this->master_joint_->Position(0) * this->multiplier_ + this->offset_;

    if (this->has_pid_)
    {
      const common::Time now = this->world_->SimTime();
      common::Time dt = now - this->last_update_;
      this->last_update_ = now;
      if (dt <= common::Time(0, 0))
        return;

      double error = this->mimic_joint_->Position(0) - target;
      if (std::fabs(error) < this->sensitiveness_)
        error = 0.0;

      double effort = this->pid_.Update(error, dt);
      if (effort > this->max_effort_)  effort = this->max_effort_;
      if (effort < -this->max_effort_) effort = -this->max_effort_;
      this->mimic_joint_->SetForce(0, effort);
    }
    else
    {
      this->mimic_joint_->SetPosition(0, target, false);
    }
  }

  physics::ModelPtr model_;
  physics::WorldPtr world_;
  physics::JointPtr master_joint_;
  physics::JointPtr mimic_joint_;
  event::ConnectionPtr update_conn_;
  common::PID pid_;
  common::Time last_update_;

  double multiplier_{1.0};
  double offset_{0.0};
  double sensitiveness_{0.0};
  double max_effort_{1.0};
  bool has_pid_{false};
};

GZ_REGISTER_MODEL_PLUGIN(GazeboMimicJointPlugin)
}  // namespace gazebo
