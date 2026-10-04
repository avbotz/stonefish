/*
    This file is a part of Stonefish.

    Stonefish is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Stonefish is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

//
//  Thruster.cpp
//  Stonefish
//
//  Created by Roger Pi on 03/06/2024
//  Modified by Patryk Cieslak on 06/07/2026
//  Copyright (c) 2024-2026 Roger Pi and Patryk Cieslak. All rights reserved.
//

#include "actuators/Thruster.h"

#include "core/SimulationApp.h"
#include "core/SimulationManager.h"
#include "core/DeviceFactory.h"
#include "graphics/GLSLShader.h"
#include "graphics/OpenGLContent.h"
#include "entities/SolidEntity.h"
#include "entities/solids/Polyhedron.h"
#include "utils/SystemUtil.hpp"
#include <sstream>

namespace sf
{

Thruster::Thruster(const std::string& uniqueName, std::unique_ptr<SolidEntity> propeller,
                                            std::unique_ptr<RotorDynamics> rotorDynamics,
                                            std::unique_ptr<ThrustModel> thrustModel,
                                            Scalar diameter, bool rightHand, Scalar maxSetpoint,
                                            bool invertedSetpoint, bool normalizedSetpoint)
    : LinkActuator(uniqueName), propeller_(std::move(propeller)), RH_(rightHand), D_(diameter),
      theta_(Scalar(0)), omega_(Scalar(0)), thrust_(Scalar(0)), torque_(Scalar(0)),
      setpoint_(Scalar(0)), setpointLimit_(maxSetpoint), inv_(invertedSetpoint), normalized_(normalizedSetpoint),
      rotorModel_(std::move(rotorDynamics)), thrustModel_(std::move(thrustModel))
{
    setSetpointLimit(maxSetpoint);
    propeller_->BuildGraphicalObject();
}

LinkActuatorType Thruster::getLinkActuatorType() const
{
    return LinkActuatorType::THRUSTER;
}

void Thruster::setSetpoint(Scalar s)
{
    if (normalized_)
        setpoint_ = btClamped(s, Scalar(-1), Scalar(1)) * setpointLimit_;
    else
        setpoint_ = btClamped(s, -setpointLimit_, setpointLimit_);
    if (inv_) setpoint_ *= Scalar(-1);
    ResetWatchdog();
}

void Thruster::setSetpointLimit(Scalar limit)
{
    setpointLimit_ = btFabs(limit);

    switch (rotorModel_->getType())
    {
        case RotorDynamicsType::ZERO_ORDER:
        case RotorDynamicsType::FIRST_ORDER:
        case RotorDynamicsType::MECHANICAL_PI: // Setpoint is the angular velocity of the rotor
            rotorModel_->setOutputLimit(setpointLimit_*2); // Protect against uncontrolled behavior
            break;

        default: // Setpoint is the motor torque or voltage -> no relation to the angular velocity
            rotorModel_->setOutputLimit(Scalar(-1)); // No limit
            break;
    }
}

Scalar Thruster::getSetpointLimit()
{
    return setpointLimit_;
}

Scalar Thruster::getSetpoint() const
{
    return inv_ ? -setpoint_ : setpoint_;
}

Scalar Thruster::getAngle() const
{
    return theta_;
}

Scalar Thruster::getOmega() const
{
    return omega_;
}

Scalar Thruster::getThrust() const
{
    return thrust_;
}

Scalar Thruster::getTorque() const
{
    return torque_;
}

bool Thruster::isPropellerRight() const
{
    return RH_;
}

Scalar Thruster::getPropellerDiameter() const
{
    return D_;
}

Scalar Thruster::ImmersionFactor(Scalar depth, Scalar diameter)
{
    Scalar R = diameter/Scalar(2);
    if (depth >= Scalar(2) * R) // Deep enough to avoid ventilation
        return Scalar(1);
    if (depth <= -R) // Propeller out of water
        return Scalar(0);

    // Submerged fraction of the propeller disk area (disk minus the circular segment above the surface)
    Scalar x = btMin(depth/R, Scalar(1));
    Scalar submerged = Scalar(1) - (btAcos(x) - x * btSqrt(Scalar(1) - x*x))/SIMD_PI;

    // Ventilation (air drawn from the surface) within one diameter from the surface
    Scalar ventilation = Scalar(0.5) + Scalar(0.5) * btClamped((depth - R)/R, Scalar(0), Scalar(1));
    return submerged * ventilation;
}

void Thruster::Update(Scalar dt)
{
    Actuator::Update(dt);

    if (attach_ == nullptr)
        return; // No attachment, no action

    // Update rotation & angular velocity
    if (rotorModel_->getType() == RotorDynamicsType::MECHANICAL_PI)
        static_cast<MechanicalPI*>(rotorModel_.get())->setDampingTorque(btFabs(torque_));
    omega_ = rotorModel_->Update(dt, setpoint_);
    theta_ += omega_ * dt; // Just for animation

    // Check if thruster is sumberged and compute thrust model
    Transform solidTrans = attach_->getCGTransform();
    Transform thrustTrans = attach_->getOTransform() * o2a_;
    Ocean *ocn = SimulationApp::getApp()->getSimulationManager()->getOcean();
    Scalar immersion = ocn != nullptr ? ImmersionFactor(ocn->GetDepth(thrustTrans.getOrigin()), D_) : Scalar(0);

    if (immersion > Scalar(0))
    {
        // Update Thrust
        if (thrustModel_->getType() == ThrustModelType::FD)
        {
            Vector3 relPos = thrustTrans.getOrigin() - solidTrans.getOrigin();
            Vector3 velocity = attach_->getLinearVelocityInLocalPoint(relPos);
            Scalar u = -thrustTrans.getBasis().getColumn(0).dot(ocn->GetFluidVelocity(thrustTrans.getOrigin()) - velocity);
            static_cast<FDThrust*>(thrustModel_.get())->setIncomingFluidVelocity(u);
            static_cast<FDThrust*>(thrustModel_.get())->setLiquidDensity(ocn->getLiquid().density);
            std::pair<Scalar, Scalar> out = thrustModel_->Update(omega_); // Handedness accounted for in the model
            thrust_ = out.first;
            torque_ = out.second;
        }
        else
        {
            // Account for handedness of the propeller: a left-hand propeller is a mirror image of a right-hand one,
            // so it behaves like a right-hand propeller rotating in the opposite direction, T_LH(w) = T(-w),
            // while the reaction torque is mirrored, Q_LH(w) = -Q(-w) (always opposing the rotation)
            std::pair<Scalar, Scalar> out = thrustModel_->Update(RH_ ? omega_ : -omega_);
            thrust_ = out.first;
            torque_ = RH_ ? out.second : -out.second;
        }

        // Reduce thrust and torque when the propeller is close to the surface
        thrust_ *= immersion;
        torque_ *= immersion;

        // Apply forces and torques
        Vector3 thrustV(thrust_, 0, 0);
        Vector3 torqueV(torque_, 0, 0);
        attach_->ApplyCentralForce(thrustTrans.getBasis() * thrustV);
        attach_->ApplyTorque((thrustTrans.getOrigin() - solidTrans.getOrigin()).cross(thrustTrans.getBasis() * thrustV));
        attach_->ApplyTorque(thrustTrans.getBasis() * torqueV);
    }
    else
    {
        thrust_ = Scalar(0);
        torque_ = Scalar(0);
    }
}

std::vector<Renderable> Thruster::Render()
{
    Transform thrustTrans = Transform::getIdentity();
    if (attach_ != nullptr)
        thrustTrans = attach_->getOTransform() * o2a_;
    else
        LinkActuator::Render();

    // Rotate propeller
    thrustTrans *= Transform(Quaternion(0, 0, theta_), Vector3(0, 0, 0));

    // Add renderable
    std::vector<Renderable> items(0);
    Renderable item;
    item.type = RenderableType::SOLID;
    item.materialName = propeller_->getMaterial().name;
    item.objectId = propeller_->getGraphicalObject();
    item.lookId = dm_ == DisplayMode::GRAPHICAL ? propeller_->getLook() : -1;
    item.model = glMatrixFromTransform(thrustTrans);
    items.push_back(item);

    item.type = RenderableType::ACTUATOR_LINES;
    item.data = std::make_shared<std::vector<glm::vec3>>();
    auto points = item.getDataAsPoints();
    points->push_back(glm::vec3(0, 0, 0));
    points->push_back(glm::vec3(0.1f * thrust_, 0, 0));
    items.push_back(item);

    return items;
}

void Thruster::WatchdogTimeout()
{
    setSetpoint(Scalar(0));
}

// Statics

ConstructInfo Thruster::getConstructInfo()
{
    ConstructInfo info;
    ConstructInfoNode node;
    
    // Specs
    node.optional = false;
    node.attributes.insert({"max_setpoint", {ConstructInfoValueType::SCALAR, false}});
    node.attributes.insert({"inverted_setpoint", {ConstructInfoValueType::BOOL, true}});
    node.attributes.insert({"normalized_setpoint", {ConstructInfoValueType::BOOL, true}});
    info.nodes.insert({"specs", node});

    // Propeller mesh
    node.attributes.clear();
    node.optional = false;
    node.attributes.insert({"diameter", {ConstructInfoValueType::SCALAR, false}});
    node.attributes.insert({"right", {ConstructInfoValueType::BOOL, false}});

    ConstructInfoNode childNode;
    childNode.optional = false;
    childNode.attributes.insert({"filename", {ConstructInfoValueType::STRING, false}});
    childNode.attributes.insert({"scale", {ConstructInfoValueType::SCALAR, true}});
    node.childNodes.insert({"mesh", childNode});

    childNode.attributes.clear();
    childNode.optional = false;
    childNode.attributes.insert({"name", {ConstructInfoValueType::STRING, false}});
    node.childNodes.insert({"material", childNode});
    node.childNodes.insert({"look", childNode});

    info.nodes.insert({"propeller", node});

    // Rotor dynamics
    node.attributes.clear();
    node.childNodes.clear();
    childNode.attributes.clear();
    node.optional = false;
    node.attributes.insert({"type", {ConstructInfoValueType::STRING, false}});

    childNode.optional = true;
    childNode.attributes.insert({"value", {ConstructInfoValueType::SCALAR, false}});
    // zero order - no parameters
    // first_order
    node.childNodes.insert({"time_constant", childNode});
    // yoerger
    node.childNodes.insert({"alpha", childNode});
    node.childNodes.insert({"beta", childNode});
    // bessa
    node.childNodes.insert({"jmsp", childNode});
    node.childNodes.insert({"kv1", childNode});
    node.childNodes.insert({"kv2", childNode});
    node.childNodes.insert({"kt", childNode});
    node.childNodes.insert({"rm", childNode});
    // mechanical_pi
    node.childNodes.insert({"rotor_inertia", childNode});
    node.childNodes.insert({"propeller_inertia", childNode}); // Alternative name
    node.childNodes.insert({"max_torque", childNode});
    node.childNodes.insert({"kp", childNode});
    node.childNodes.insert({"ki", childNode});
    node.childNodes.insert({"ilimit", childNode});

    info.nodes.insert({"rotor_dynamics", node});

    // Thrust model
    node.attributes.clear();
    node.childNodes.clear();
    childNode.attributes.clear();
    node.optional = false;
    node.attributes.insert({"type", {ConstructInfoValueType::STRING, false}});
    node.attributes.insert({"torque_ratio", {ConstructInfoValueType::SCALAR, true}}); // Ratio of the reaction torque to the thrust (not used by the fluid dynamics model)
    
    childNode.optional = true;
    
    // thrust_coeff
    childNode.attributes.insert({"forward", {ConstructInfoValueType::SCALAR, false}});
    childNode.attributes.insert({"reverse", {ConstructInfoValueType::SCALAR, true}});
    node.childNodes.insert({"thrust_coeff", childNode});

    // deadband
    childNode.attributes.clear();
    childNode.attributes.insert({"lower", {ConstructInfoValueType::SCALAR, false}});
    childNode.attributes.insert({"upper", {ConstructInfoValueType::SCALAR, false}});
    node.childNodes.insert({"deadband", childNode});

    // torque_coeff
    childNode.attributes.clear();
    childNode.attributes.insert({"value", {ConstructInfoValueType::SCALAR, false}});
    node.childNodes.insert({"torque_coeff", childNode});

    // linear_interpolation
    childNode.attributes.clear();
    childNode.attributes.insert({"value", {ConstructInfoValueType::STRING, false}});
    node.childNodes.insert({"input", childNode});
    node.childNodes.insert({"output", childNode});

    info.nodes.insert({"thrust_model", node});

    return info;
}

std::unique_ptr<Thruster> Thruster::Construct(const std::string& uniqueName, ConstructInfo& info)
{
    // Specs
    Scalar maxSetpoint = std::get<Scalar>(info.nodes.at("specs").attributes.at("max_setpoint").value);
    bool invertedSetpoint = false;
    bool normalizedSetpoint = true;
    ConstructInfoValue& value = info.nodes.at("specs").attributes.at("inverted_setpoint");
    if (value.valid)
        invertedSetpoint = std::get<bool>(value.value);
    value = info.nodes.at("specs").attributes.at("normalized_setpoint");
    if (value.valid)
        normalizedSetpoint = std::get<bool>(value.value);
    
    // Propeller mesh (required)
    Scalar diameter = std::get<Scalar>(info.nodes.at("propeller").attributes.at("diameter").value);
    bool right = std::get<bool>(info.nodes.at("propeller").attributes.at("right").value);
    std::string meshFilename = std::get<std::string>(info.nodes.at("propeller").childNodes.at("mesh").attributes.at("filename").value);
    value = info.nodes.at("propeller").childNodes.at("mesh").attributes.at("scale");
    Scalar meshScale(1.);
    if (value.valid)
        meshScale = std::get<Scalar>(value.value);
    std::string materialName = std::get<std::string>(info.nodes.at("propeller").childNodes.at("material").attributes.at("name").value);
    std::string lookName = std::get<std::string>(info.nodes.at("propeller").childNodes.at("look").attributes.at("name").value);

    PhysicsSettings phy;
    phy.collisions = false;
    phy.buoyancy = false;
    phy.mode = PhysicsMode::SUBMERGED;
    std::unique_ptr<Polyhedron> propeller = std::make_unique<Polyhedron>(uniqueName + "/Propeller", phy, GetFullPath(meshFilename), 
        meshScale, I4(), materialName, lookName, Scalar(-1.), GeometryApproxType::CYLINDER);

    // Rotor dynamics
    std::unique_ptr<RotorDynamics> rotorDynamics;

    std::string rotorDynamicsType = std::get<std::string>(info.nodes.at("rotor_dynamics").attributes.at("type").value);
    if (rotorDynamicsType == "zero_order")
    {
        rotorDynamics = std::make_unique<ZeroOrder>();
    }
    else if (rotorDynamicsType == "first_order")
    {
        value = info.nodes.at("rotor_dynamics").childNodes.at("time_constant").attributes.at("value");
        if (!value.valid)
            return nullptr;
        
        rotorDynamics = std::make_unique<FirstOrder>(std::get<Scalar>(value.value));
    }
    else if (rotorDynamicsType == "yoerger")
    {
        value = info.nodes.at("rotor_dynamics").childNodes.at("alpha").attributes.at("value");
        if (!value.valid)
            return nullptr;
        Scalar alpha = std::get<Scalar>(value.value);
        
        value = info.nodes.at("rotor_dynamics").childNodes.at("beta").attributes.at("value");
        if (!value.valid)
            return nullptr;
        Scalar beta = std::get<Scalar>(value.value);

        rotorDynamics = std::make_unique<Yoerger>(alpha, beta);
    }
    else if (rotorDynamicsType == "bessa")
    {
        value = info.nodes.at("rotor_dynamics").childNodes.at("jmsp").attributes.at("value");
        if (!value.valid)
            return nullptr;
        Scalar jmsp = std::get<Scalar>(value.value);

        value = info.nodes.at("rotor_dynamics").childNodes.at("kv1").attributes.at("value");
        if (!value.valid)
            return nullptr;
        Scalar kv1 = std::get<Scalar>(value.value);

        value = info.nodes.at("rotor_dynamics").childNodes.at("kv2").attributes.at("value");
        if (!value.valid)
            return nullptr;
        Scalar kv2 = std::get<Scalar>(value.value);

        value = info.nodes.at("rotor_dynamics").childNodes.at("kt").attributes.at("value");
        if (!value.valid)
            return nullptr;
        Scalar kt = std::get<Scalar>(value.value);

        value = info.nodes.at("rotor_dynamics").childNodes.at("rm").attributes.at("value");
        if (!value.valid)
            return nullptr;
        Scalar rm = std::get<Scalar>(value.value);

        rotorDynamics = std::make_unique<Bessa>(jmsp, kv1, kv2, kt, rm);
    }
    else if (rotorDynamicsType == "mechanical_pi")
    {
        Scalar J = propeller->getInertia().getX() + propeller->getAddedInertia().getX();
        value = info.nodes.at("rotor_dynamics").childNodes.at("rotor_inertia").attributes.at("value");
        if (!value.valid)
            value = info.nodes.at("rotor_dynamics").childNodes.at("propeller_inertia").attributes.at("value");
        if (value.valid)
            J = std::get<Scalar>(value.value);
        if (J <= Scalar(0))
        {
            cError("Rotor inertia of actuator '%s' has to be positive!", uniqueName.c_str());
            return nullptr;
        }

        Scalar maxTorque(-1); // No limit
        value = info.nodes.at("rotor_dynamics").childNodes.at("max_torque").attributes.at("value");
        if (value.valid)
            maxTorque = std::get<Scalar>(value.value);
            
        value = info.nodes.at("rotor_dynamics").childNodes.at("kp").attributes.at("value");
        if (!value.valid)
            return nullptr;
        Scalar kp = std::get<Scalar>(value.value);

        value = info.nodes.at("rotor_dynamics").childNodes.at("ki").attributes.at("value");
        if (!value.valid)
            return nullptr;
        Scalar ki = std::get<Scalar>(value.value);

        value = info.nodes.at("rotor_dynamics").childNodes.at("ilimit").attributes.at("value");
        if (!value.valid)
            return nullptr;
        Scalar iLimit = std::get<Scalar>(value.value);

        rotorDynamics = std::make_unique<MechanicalPI>(J, kp, ki, iLimit, maxTorque);
    }
    else
    {
        return nullptr;
    }

    // Thrust model
    std::unique_ptr<ThrustModel> thrustModel;

    std::string thrustModelType = std::get<std::string>(info.nodes.at("thrust_model").attributes.at("type").value);
    if (thrustModelType == "quadratic")
    {
        Scalar forward = std::get<Scalar>(info.nodes.at("thrust_model").childNodes.at("thrust_coeff").attributes.at("forward").value);
        Scalar reverse = forward;
        value = info.nodes.at("thrust_model").childNodes.at("thrust_coeff").attributes.at("reverse");
        if (value.valid)
            reverse = std::get<Scalar>(value.value);
        
        thrustModel = std::make_unique<QuadraticThrust>(forward, reverse);
    }   
    else if (thrustModelType == "deadband")
    {
        Scalar forward = std::get<Scalar>(info.nodes.at("thrust_model").childNodes.at("thrust_coeff").attributes.at("forward").value);
        Scalar reverse = forward;
        value = info.nodes.at("thrust_model").childNodes.at("thrust_coeff").attributes.at("reverse");
        if (value.valid)
            reverse = std::get<Scalar>(value.value);

        Scalar lower = std::get<Scalar>(info.nodes.at("thrust_model").childNodes.at("deadband").attributes.at("lower").value);
        Scalar upper = std::get<Scalar>(info.nodes.at("thrust_model").childNodes.at("deadband").attributes.at("upper").value);

        thrustModel = std::make_unique<DeadbandThrust>(forward, reverse, lower, upper);
    } 
    else if (thrustModelType == "linear_interpolation")
    {
        std::string inputString = std::get<std::string>(info.nodes.at("thrust_model").childNodes.at("input").attributes.at("value").value);
        std::string outputString = std::get<std::string>(info.nodes.at("thrust_model").childNodes.at("output").attributes.at("value").value);

        auto stringToVector = [](const std::string& str) -> std::vector<Scalar> 
        {
            std::vector<Scalar> result;
            std::stringstream ss(str);
            Scalar temp;
            while (ss >> temp) result.push_back(temp);
            return result;
        };

        auto input = stringToVector(inputString);
        auto output = stringToVector(outputString);

        try
        {
            thrustModel = std::make_unique<InterpolatedThrust>(input, output);
        }
        catch (const std::exception& e)
        {
            cError("Linear interpolation of actuator '%s' incorrect: %s", uniqueName.c_str(), e.what());
            return nullptr;
        }
    }
    else if (thrustModelType == "fluid_dynamics")
    {
        Scalar forward = std::get<Scalar>(info.nodes.at("thrust_model").childNodes.at("thrust_coeff").attributes.at("forward").value);
        Scalar reverse = forward;
        value = info.nodes.at("thrust_model").childNodes.at("thrust_coeff").attributes.at("reverse");
        if (value.valid)
            reverse = std::get<Scalar>(value.value);
        Scalar torqueCoeff = std::get<Scalar>(info.nodes.at("thrust_model").childNodes.at("torque_coeff").attributes.at("value").value);
        
        thrustModel = std::make_unique<FDThrust>(forward, reverse, torqueCoeff, diameter, right);
    }
    else
    {
        return nullptr;
    }

    value = info.nodes.at("thrust_model").attributes.at("torque_ratio");
    if (value.valid)
    {
        Scalar torqueRatio = std::get<Scalar>(value.value);
        if (thrustModel->getType() != ThrustModelType::FD)
            thrustModel->setTorqueRatio(torqueRatio);
        else if (torqueRatio != Scalar(0))
            cWarning("Torque ratio of actuator '%s' ignored - fluid dynamics thrust model uses torque coefficient.", uniqueName.c_str());
    }

    return std::make_unique<Thruster>(uniqueName, std::move(propeller), std::move(rotorDynamics), std::move(thrustModel), 
        diameter, right, maxSetpoint, invertedSetpoint, normalizedSetpoint);
}

REGISTER_ACTUATOR("thruster", Thruster)

} 
