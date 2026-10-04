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
//  SolidEntity.cpp
//  Stonefish
//
//  Created by Patryk Cieslak on 29/12/12.
//  Copyright (c) 2012-2026 Patryk Cieslak. All rights reserved.
//

#include "entities/SolidEntity.h"

#include "core/GraphicalSimulationApp.h"
#include "core/SimulationManager.h"
#include "graphics/OpenGLPipeline.h"
#include "graphics/OpenGLContent.h"
#include "utils/SystemUtil.hpp"
#include "entities/forcefields/Ocean.h"
#include "entities/forcefields/Atmosphere.h"
#include <iostream>
#include <algorithm>

namespace sf
{

SolidEntity::SolidEntity(const std::string& uniqueName, PhysicsSettings phy, const std::string& material, const std::string& look, Scalar thickness) 
    : MovingEntity(uniqueName, material, look), thick_(thickness), phy_(phy)
{
    //Check if ocean is enabled and change physics mode accordingly
    if((phy.mode == PhysicsMode::SUBMERGED || phy.mode == PhysicsMode::FLOATING) && !SimulationApp::getApp()->getSimulationManager()->isOceanEnabled())
        this->phy_.mode = PhysicsMode::SURFACE;
    
    //Get material
    mat_ = SimulationApp::getApp()->getSimulationManager()->getMaterialManager()->getMaterial(material);
    
    //Get Look
    if(SimulationApp::getApp()->hasGraphics())
        lookId_ = static_cast<GraphicalSimulationApp*>(SimulationApp::getApp())->getGLPipeline()->getContent()->getLookId(look);
    else
        lookId_ = -1;
    
    //Set transformations to identity
    T_O2G_ = I4();
    T_O2C_ = I4();
    T_O2H_ = I4();
    T_CG2C_ = I4();
    T_CG2G_ = I4();
    T_CG2O_ = I4();
    P_CB_.setZero();
    
    //Set properties
    mass_ = Scalar(0);
    aMass_.setZero();
    aMassT_ = Matrix3(0,0,0,0,0,0,0,0,0);
    surface_ = Scalar(0);
    aI_.setZero();
    Ipri_.setZero();
    contactK_ = Scalar(-1);
    contactD_ = Scalar(0);
    volume_ = Scalar(0);
    fdApproxType_ =  GeometryApproxType::AUTO;
    fdApproxParams_ = std::vector<Scalar>(0);
    fdCd_ = Vector3(-1.0, -1.0, -1.0);
    fdCf_ = Vector3(-1.0, -1.0, -1.0);
    T_CG2H_ = Transform::getIdentity();
    
    //Set vectors to zero
    Fb_.setZero();
    Tb_.setZero();
    Fdq_.setZero();
    Tdq_.setZero();
    Fdf_.setZero();
    Tdf_.setZero();
    Fda_.setZero();
    Tda_.setZero();
    Swet_ = Scalar(0);
    Vsub_ = Scalar(0);
    lastV_.setZero();
    lastOmega_.setZero();
    linearAcc_.setZero();
    angularAcc_.setZero();
    
    //Set pointers
    graObjectId_ = -1;
    phyObjectId_ = -1;
    dm_ = DisplayMode::GRAPHICAL;
    submerged_.type = RenderableType::HYDRO_LINES;
    submerged_.model = glm::mat4(1.f);
    submerged_.data = std::make_shared<std::vector<glm::vec3>>();
}

SolidEntity::~SolidEntity()
{
    btCompoundShape* cs;
    if ((cs = dynamic_cast<btCompoundShape*>(collisionShape_.get())) != nullptr)
    {
        for (int i = 0; i < cs->getNumChildShapes(); ++i)
            delete cs->getChildShape(i);
    }
}

EntityType SolidEntity::getType() const
{
    return EntityType::SOLID;
}

btMultiBodyLinkCollider* SolidEntity::getMultiBodyLinkCollider() const
{
    return multibodyCollider_.get();
}

void SolidEntity::ScalePhysicalPropertiesToArbitraryMass(Scalar mass)
{
    if(rigidBody_ != nullptr || multibodyCollider_ != nullptr)
    {
        cWarning("Physical properties of bodies cannot be changed after adding to simulation!");
        return;
    }
    
    Scalar oldMass = this->mass_;
    this->mass_ = mass;
    Ipri_ *= this->mass_/oldMass;
}

void SolidEntity::SetArbitraryPhysicalProperties(Scalar mass, const Vector3& inertia, const Transform& CG)
{
    if(rigidBody_ != nullptr || multibodyCollider_ != nullptr)
    {
        cWarning("Physical properties of bodies cannot be changed after adding to simulation!");
        return;
    }
    
    this->mass_ = mass;
    Ipri_ = inertia;
    Transform T_CG_old_new = T_CG2O_ * CG; //Transform from old CG to new CG
    T_CG2O_ = CG.inverse(); //Set new CG in body origin frame
    T_CG2C_ = T_CG2O_ * T_O2C_;
    T_CG2G_ = T_CG2O_ * T_O2G_;
    T_CG2H_ = T_CG_old_new.inverse() * T_CG2H_;
    P_CB_ = T_CG_old_new.inverse() * P_CB_;
}

void SolidEntity::SetContactProperties(bool soft, Scalar stiffness, Scalar damping)
{
    if(soft)
    {
        contactK_ = stiffness > Scalar(0) ? stiffness : Scalar(1000);
        contactD_ = damping >= Scalar(0) ? damping : Scalar(0); 
    }
    else
    {
        contactK_ = Scalar(-1);
        contactD_ = Scalar(0);
    }
    
    if(rigidBody_ != nullptr)
    {
        if(soft)
            rigidBody_->setContactStiffnessAndDamping(contactK_, contactD_);
        else
        {
            int cflags = rigidBody_->getCollisionFlags();
            cflags &= ~(btCollisionObject::CollisionFlags::CF_HAS_CONTACT_STIFFNESS_DAMPING);
            rigidBody_->setCollisionFlags(cflags);
        }
    }
    else if(multibodyCollider_ != nullptr)
    {
        if(soft)
            multibodyCollider_->setContactStiffnessAndDamping(contactK_, contactD_);
        else
        {
            int cflags = multibodyCollider_->getCollisionFlags();
            cflags &= ~(btCollisionObject::CollisionFlags::CF_HAS_CONTACT_STIFFNESS_DAMPING);
            multibodyCollider_->setCollisionFlags(cflags);
        }
    }
}

void SolidEntity::SetAddedMass(const Vector3& addedMass, const Vector3& addedInertia)
{
    //Values defined in the origin frame of the body -> CG frame (principal axes)
    Matrix3 R = T_CG2O_.getBasis();
    Matrix3 A = R * Matrix3(btMax(addedMass.x(), Scalar(0)), 0, 0, 0, btMax(addedMass.y(), Scalar(0)), 0, 0, 0, btMax(addedMass.z(), Scalar(0))) * R.transpose();
    Matrix3 I = R * Matrix3(btMax(addedInertia.x(), Scalar(0)), 0, 0, 0, btMax(addedInertia.y(), Scalar(0)), 0, 0, 0, btMax(addedInertia.z(), Scalar(0))) * R.transpose();
    if(addedMass.x() >= Scalar(0) && addedMass.y() >= Scalar(0) && addedMass.z() >= Scalar(0))
    {
        aMassT_ = A;
        aMass_ = Vector3(A[0][0], A[1][1], A[2][2]);
    }
    if(addedInertia.x() >= Scalar(0) && addedInertia.y() >= Scalar(0) && addedInertia.z() >= Scalar(0))
        aI_ = Vector3(I[0][0], I[1][1], I[2][2]);
}

void SolidEntity::SetHydrodynamicCoefficients(const Vector3& Cd, const Vector3& Cf)
{
    if(Cd.getX() >= Scalar(0) && Cd.getY() >= Scalar(0) && Cd.getZ() >= Scalar(0))
        fdCd_ = Cd;
    if(Cf.getX() >= Scalar(0) && Cf.getY() >= Scalar(0) && Cf.getZ() >= Scalar(0))
        fdCf_ = Cf;
}

int SolidEntity::getPhysicalObject() const
{
    return phyObjectId_;
}

bool SolidEntity::isBuoyant() const
{
    return (phy_.mode == PhysicsMode::SUBMERGED || phy_.mode == PhysicsMode::FLOATING) && phy_.buoyancy;
}
    
PhysicsMode SolidEntity::getPhysicsMode() const
{
    return phy_.mode;
}

void SolidEntity::getAABB(Vector3& min, Vector3& max)
{
    if(rigidBody_ != nullptr)
        rigidBody_->getAabb(min, max);
    else if(multibodyCollider_ != nullptr)
        multibodyCollider_->getCollisionShape()->getAabb(getCGTransform(), min, max);
    else
    {
        min.setValue(BT_LARGE_FLOAT, BT_LARGE_FLOAT, BT_LARGE_FLOAT);
        max.setValue(-BT_LARGE_FLOAT, -BT_LARGE_FLOAT, -BT_LARGE_FLOAT);
    }
}

std::vector<Renderable> SolidEntity::Render()
{
    std::vector<Renderable> items(0);
    
    if( (rigidBody_ != nullptr || multibodyCollider_ != nullptr)  && isRenderable() )
    {
        //Mesh
        Renderable item1;
        item1.type = RenderableType::SOLID;
        item1.materialName = mat_.name;
        
        if(dm_ == DisplayMode::GRAPHICAL && graObjectId_ >= 0)
        {
            item1.objectId = graObjectId_;
            item1.lookId = lookId_;
            item1.model = glMatrixFromTransform(getGTransform());
            item1.cor = glVectorFromVector(getCGTransform().getOrigin());
            item1.vel = glVectorFromVector(getLinearVelocity());
            item1.avel = glVectorFromVector(getAngularVelocity());
            items.push_back(item1);
        }
        else if(dm_ == DisplayMode::PHYSICAL && phyObjectId_ >= 0)
        {
            item1.objectId = phyObjectId_;
            item1.lookId = -1;
            item1.model = glMatrixFromTransform(getCTransform());
            item1.cor = glVectorFromVector(getCGTransform().getOrigin());
            item1.vel = glVectorFromVector(getLinearVelocity());
            item1.avel = glVectorFromVector(getAngularVelocity());
            items.push_back(item1);
        }
        
        //Coordinate system
        Renderable item2;
        item2.type = RenderableType::SOLID_CS;
        item2.model = glMatrixFromTransform(getCGTransform());
        items.push_back(item2);
        
        //Hydrodynamics
        Vector3 cbWorld = getCGTransform() * P_CB_;
        Renderable item3;
        item3.type = RenderableType::HYDRO_CS;
        item3.model = glMatrixFromTransform(Transform(Quaternion::getIdentity(), cbWorld));
        items.push_back(item3);

        //Forces
        Vector3 cg = getCGTransform().getOrigin();
        glm::vec3 cgv((GLfloat)cg.x(), (GLfloat)cg.y(), (GLfloat)cg.z());

        //---Buoyancy
        Renderable item4;
        item4.type = RenderableType::FORCE_BUOYANCY;
        item4.model = glm::mat4(1.f);
        item4.data = std::make_shared<std::vector<glm::vec3>>();
        auto points = item4.getDataAsPoints();
        points->push_back(cgv);
        points->push_back(cgv + glm::vec3((GLfloat)Fb_.x(), (GLfloat)Fb_.y(), (GLfloat)Fb_.z())/1000.f);
        items.push_back(item4);
        
        //---Linear drag
        Renderable item5;
        item5.type = RenderableType::FORCE_LINEAR_DRAG;
        item5.model = glm::mat4(1.f);
        item5.data = std::make_shared<std::vector<glm::vec3>>();
        points = item5.getDataAsPoints();
        points->push_back(cgv);
        points->push_back(cgv + glm::vec3((GLfloat)Fdf_.x(), (GLfloat)Fdf_.y(), (GLfloat)Fdf_.z()));
        items.push_back(item5);

        //---Quadratic drag
        Renderable item6;
        item6.type = RenderableType::FORCE_QUADRATIC_DRAG;
        item6.model = glm::mat4(1.f);
        item6.data = std::make_shared<std::vector<glm::vec3>>();
        points = item6.getDataAsPoints();
        points->push_back(cgv);
        points->push_back(cgv + glm::vec3((GLfloat)Fdq_.x(), (GLfloat)Fdq_.y(), (GLfloat)Fdq_.z()));
        items.push_back(item6);

        //Surface crossing debug
#ifdef DEBUG_HYDRO
        items.push_back(submerged);

        //points = submerged.getDataAsPoints();
        Renderable debugItem;
        debugItem.type = RenderableType::HYDRO_LINES;
        debugItem.model = glm::mat4(1.f);
        debugItem.data = std::make_shared<std::vector<glm::vec3>>();
        points = debugItem.getDataAsPoints();

        Vector3 min, max;
        getAABB(min, max);

        points->push_back(glm::vec3((GLfloat)min.x(), (GLfloat)min.y(), (GLfloat)min.z()));
        points->push_back(glm::vec3((GLfloat)max.x(), (GLfloat)min.y(), (GLfloat)min.z()));

        points->push_back(glm::vec3((GLfloat)min.x(), (GLfloat)min.y(), (GLfloat)min.z()));
        points->push_back(glm::vec3((GLfloat)min.x(), (GLfloat)max.y(), (GLfloat)min.z()));

        points->push_back(glm::vec3((GLfloat)min.x(), (GLfloat)min.y(), (GLfloat)min.z()));
        points->push_back(glm::vec3((GLfloat)min.x(), (GLfloat)min.y(), (GLfloat)max.z()));

        points->push_back(glm::vec3((GLfloat)min.x(), (GLfloat)max.y(), (GLfloat)min.z()));
        points->push_back(glm::vec3((GLfloat)min.x(), (GLfloat)max.y(), (GLfloat)max.z()));

        points->push_back(glm::vec3((GLfloat)min.x(), (GLfloat)min.y(), (GLfloat)max.z()));
        points->push_back(glm::vec3((GLfloat)min.x(), (GLfloat)max.y(), (GLfloat)max.z()));

        points->push_back(glm::vec3((GLfloat)min.x(), (GLfloat)min.y(), (GLfloat)max.z()));
        points->push_back(glm::vec3((GLfloat)max.x(), (GLfloat)min.y(), (GLfloat)max.z()));

        points->push_back(glm::vec3((GLfloat)max.x(), (GLfloat)min.y(), (GLfloat)min.z()));
        points->push_back(glm::vec3((GLfloat)max.x(), (GLfloat)min.y(), (GLfloat)max.z()));

        points->push_back(glm::vec3((GLfloat)max.x(), (GLfloat)min.y(), (GLfloat)min.z()));
        points->push_back(glm::vec3((GLfloat)max.x(), (GLfloat)max.y(), (GLfloat)min.z()));

        points->push_back(glm::vec3((GLfloat)min.x(), (GLfloat)max.y(), (GLfloat)min.z()));
        points->push_back(glm::vec3((GLfloat)max.x(), (GLfloat)max.y(), (GLfloat)min.z()));

        points->push_back(glm::vec3((GLfloat)max.x(), (GLfloat)max.y(), (GLfloat)min.z()));
        points->push_back(glm::vec3((GLfloat)max.x(), (GLfloat)max.y(), (GLfloat)max.z()));

        points->push_back(glm::vec3((GLfloat)max.x(), (GLfloat)min.y(), (GLfloat)max.z()));
        points->push_back(glm::vec3((GLfloat)max.x(), (GLfloat)max.y(), (GLfloat)max.z()));

        points->push_back(glm::vec3((GLfloat)min.x(), (GLfloat)max.y(), (GLfloat)max.z()));
        points->push_back(glm::vec3((GLfloat)max.x(), (GLfloat)max.y(), (GLfloat)max.z()));

        items.push_back(debugItem);
#else
        //Geometry approximation
        Renderable item7;
        item7.model = glMatrixFromTransform(getHTransform());
        item7.data = std::make_shared<std::vector<glm::vec3>>();
        points = item7.getDataAsPoints();

        switch(fdApproxType_)
        {    
            case GeometryApproxType::SPHERE:
                item7.type = RenderableType::HYDRO_ELLIPSOID;
                points->push_back(glm::vec3((GLfloat)fdApproxParams_[0], (GLfloat)fdApproxParams_[0], (GLfloat)fdApproxParams_[0]));
                break;
                
            case GeometryApproxType::CYLINDER:
                item7.type = RenderableType::HYDRO_CYLINDER;
                points->push_back(glm::vec3((GLfloat)fdApproxParams_[0], (GLfloat)fdApproxParams_[0], (GLfloat)fdApproxParams_[1]));
                break;

            case GeometryApproxType::AUTO:       
            case GeometryApproxType::ELLIPSOID:
                item7.type = RenderableType::HYDRO_ELLIPSOID;
                points->push_back(glm::vec3((GLfloat)fdApproxParams_[0], (GLfloat)fdApproxParams_[1], (GLfloat)fdApproxParams_[2]));
                break;
        }
        items.push_back(item7);
#endif
    }
    
    return items;
}
    
Transform SolidEntity::getCG2GTransform() const
{
    return T_CG2G_;
}
    
Transform SolidEntity::getCG2CTransform() const
{
    return T_CG2C_;
}
    
Transform SolidEntity::getCG2OTransform() const
{
    return T_CG2O_;
}
    
Vector3 SolidEntity::getCB() const
{
    return P_CB_;
}

Transform SolidEntity::getCGTransform() const
{
    if(rigidBody_ != nullptr)
    {
        //Motion state holds an interpolated transform, updated only after the full simulation step -> use current state
        return rigidBody_->getCenterOfMassTransform();
    }
    else if(multibodyCollider_ != nullptr)
    {
        return multibodyCollider_->getWorldTransform();
    }
    else
        return Transform::getIdentity();
}

Transform SolidEntity::getO2CTransform() const
{
    return T_O2C_;
}
    
Transform SolidEntity::getO2GTransform() const
{
    return T_O2G_;
}

Transform SolidEntity::getO2HTransform() const
{
    return T_O2H_;
}
    
Transform SolidEntity::getGTransform() const
{
    return getCGTransform() * T_CG2G_;
}
    
Transform SolidEntity::getCTransform() const
{
    return getCGTransform() * T_CG2C_;
}
    
Transform SolidEntity::getHTransform() const
{
    return getCGTransform() * T_CG2H_;
}
    
Transform SolidEntity::getOTransform() const
{
    return getCGTransform() * T_CG2O_;
}

void SolidEntity::setCGTransform(const Transform& trans)
{
    if(rigidBody_ != nullptr)
    {
        rigidBody_->setCenterOfMassTransform(trans);
        rigidBody_->getMotionState()->setWorldTransform(trans);
    }
    else if(multibodyCollider_ != nullptr)
    {
        multibodyCollider_->setWorldTransform(trans);
    }
}

Vector3 SolidEntity::getLinearVelocity() const
{
    if(rigidBody_ != nullptr)
    {
        return rigidBody_->getLinearVelocity();
    }
    else if(multibodyCollider_ != nullptr)
    {
        //Get multibody and link id
        btMultiBody* multiBody = multibodyCollider_->m_multiBody;
        int index = multibodyCollider_->m_link;
        if(index < 0) //Base
            return multiBody->getBaseVel(); //Global
        
        //Velocities of all links propagated along the kinematic tree (in local frames of links)
        btAlignedObjectArray<Vector3> omega, vel;
        omega.resize(multiBody->getNumLinks() + 1);
        vel.resize(multiBody->getNumLinks() + 1);
        multiBody->compTreeLinkVelocities(&omega[0], &vel[0]);
        return multiBody->localDirToWorld(index, vel[index + 1]); //Global
    }
    else
        return Vector3(0,0,0);
}

Vector3 SolidEntity::getAngularVelocity() const
{
    if(rigidBody_ != nullptr)
    {
        return rigidBody_->getAngularVelocity();
    }
    else if(multibodyCollider_ != nullptr)
    {
        //Get multibody and link id
        btMultiBody* multiBody = multibodyCollider_->m_multiBody;
        int index = multibodyCollider_->m_link;
        if(index < 0) //Base
            return multiBody->getBaseOmega(); //Global
        
        //Velocities of all links propagated along the kinematic tree (in local frames of links)
        btAlignedObjectArray<Vector3> omega, vel;
        omega.resize(multiBody->getNumLinks() + 1);
        vel.resize(multiBody->getNumLinks() + 1);
        multiBody->compTreeLinkVelocities(&omega[0], &vel[0]);
        return multiBody->localDirToWorld(index, omega[index + 1]); //Global
    }
    else
        return Vector3(0,0,0);
}

Vector3 SolidEntity::getLinearVelocityInLocalPoint(const Vector3& relPos) const
{
    if(rigidBody_ != nullptr)
    {
        return rigidBody_->getVelocityInLocalPoint(relPos);
    }
    else if(multibodyCollider_ != nullptr)
    {
        return getLinearVelocity() + getAngularVelocity().cross(relPos);
    }
    else
        return Vector3(0,0,0);
}

Vector3 SolidEntity::getAppliedForce()
{
    if(rigidBody_ != nullptr)
    {
        return rigidBody_->getTotalForce();
    }
    else if(multibodyCollider_ != nullptr)
    {
        btMultiBody* multiBody = multibodyCollider_->m_multiBody;
        int index = multibodyCollider_->m_link;

        if(index >= 0)
            return multiBody->getLinkForce(index);
        else
            return multiBody->getBaseForce();
    }
    else
        return V0();
}

void SolidEntity::getHydrodynamicForces(Vector3& Fb, Vector3& Tb, Vector3& Fd, Vector3& Td, Vector3& Ff, Vector3& Tf)
{
    Fb = this->Fb_;
    Tb = this->Tb_;
    Fd = this->Fdq_;
    Td = this->Tdq_;
    Ff = this->Fdf_;
    Tf = this->Tdf_;
}

void SolidEntity::getHydrodynamicCoefficients(Vector3& Cd, Vector3& Cf) const
{
    Cd = fdCd_;
    Cf = fdCf_;
}

Scalar SolidEntity::getWettedSurface() const
{
    return Swet_;
}

Scalar SolidEntity::getSubmergedVolume() const
{
    return Vsub_;
}

Vector3 SolidEntity::getLinearAcceleration() const
{
    return linearAcc_;
}

Vector3 SolidEntity::getAngularAcceleration() const
{
    return angularAcc_;
}

Scalar SolidEntity::getVolume() const
{
    return volume_;
}
    
Vector3 SolidEntity::getInertia() const
{
    return Ipri_;
}

Scalar SolidEntity::getMass() const
{
    return mass_;
}

Scalar SolidEntity::getSurface() const
{
    return surface_;
}

Vector3 SolidEntity::getAddedMass() const
{
    return aMass_;
}

Matrix3 SolidEntity::getAddedMassTensor() const
{
    return aMassT_;
}

Vector3 SolidEntity::getAddedInertia() const
{
    return aI_;
}

Scalar SolidEntity::getAugmentedMass() const
{
    if(phy_.mode == PhysicsMode::SUBMERGED)
        return mass_ + (aMass_.x() + aMass_.y() + aMass_.z())/Scalar(3);
    else
        return mass_;
}

Vector3 SolidEntity::getAugmentedInertia() const
{
    if(phy_.mode == PhysicsMode::SUBMERGED)
        return Ipri_ + aI_;
    else
        return Ipri_;
}

void SolidEntity::getGeometryApprox(GeometryApproxType& type, std::vector<Scalar>& params) const
{
    type = fdApproxType_;
    params = fdApproxParams_;
}

const Mesh* SolidEntity::getPhysicsMesh()
{
    return phyMesh_.get();
}

std::vector<Vector3> SolidEntity::getMeshVertices() const
{
    std::vector<Vector3> vertices;
    if(phyMesh_ != nullptr)
    {
        for(size_t i=0; i<phyMesh_->getNumOfVertices(); ++i)
        {
            glm::vec3 pos = phyMesh_->getVertexPos(i);
            vertices.push_back(Vector3(pos.x, pos.y, pos.z));
        }
    }
    return vertices;
}

void SolidEntity::ComputeFluidDynamicsApprox(GeometryApproxType t)
{
    switch(t)
    {
        case  GeometryApproxType::SPHERE:
            ComputeSphericalApprox();
            break;

        case  GeometryApproxType::CYLINDER:
            ComputeCylindricalApprox();
            break;
            
        case  GeometryApproxType::AUTO:    
        case  GeometryApproxType::ELLIPSOID:
            ComputeEllipsoidalApprox();
            break;
    }
#ifdef DEBUG
    cInfo("Added mass: %lf, %lf, %lf, %lf, %lf, %lf", aMass_.x(), aMass_.y(), aMass_.z(), aI_.x(), aI_.y(), aI_.z());
#endif
}

void SolidEntity::ComputeSphericalApprox()
{
    std::vector<Vector3> x = getMeshVertices();
    if(x.size() < 2)
        return;
    for(size_t i=0; i<x.size(); ++i)
        x[i] = T_CG2C_ * x[i] - P_CB_;
    
    Scalar r(0);
    for(size_t i=0; i<x.size(); ++i)
    {
        Scalar rc = x[i].length2();
        if(rc > r)
            r = rc;
    }
    
    fdApproxType_ =  GeometryApproxType::SPHERE;
    fdApproxParams_.resize(1);
    fdApproxParams_[0] = btSqrt(r);
    
    Scalar rho = Scalar(1000);
    Ocean* ocn;
    if((ocn = SimulationApp::getApp()->getSimulationManager()->getOcean()) != nullptr)
        rho = ocn->getLiquid().density;

    Scalar R = fdApproxParams_[0];
    Scalar m = Scalar(2)*M_PI*rho*R*R*R/Scalar(3); //Half of the displaced mass
    aMass_ = Vector3(m,m,m);
    aMassT_ = Matrix3(m, 0, 0, 0, m, 0, 0, 0, m);
    aI_ = V0();
    
    //Set transform with respect to geometry
    Transform sphereTransform = I4();
    sphereTransform.setOrigin(P_CB_);
    T_CG2H_ = sphereTransform;

    Vector3 Cd(0.47,0.47,0.47); //Sphere in subcritical flow
    SetHydrodynamicCoefficients(Cd, Vector3(-1,-1,-1)); //Skin friction estimated from the Reynolds number
}

void SolidEntity::ComputeCylindricalApprox()
{
    std::vector<Vector3> x = getMeshVertices();
    if(x.size() < 2)
        return;
    for(size_t i=0; i<x.size(); ++i)
        x[i] = T_CG2C_ * x[i] - P_CB_;
        
    //Radius
    Scalar r[3] = {0,0,0};
    for(size_t i=0; i<x.size(); ++i)
    {
        Scalar d;
        
        //X
        d = btSqrt(x[i].y()*x[i].y() + x[i].z()*x[i].z());
        r[0] = d > r[0] ? d : r[0];
        
        //Y
        d = btSqrt(x[i].x()*x[i].x() + x[i].z()*x[i].z());
        r[1] = d > r[1] ? d : r[1];
        
        //Z
        d = btSqrt(x[i].x()*x[i].x() + x[i].y()*x[i].y());
        r[2] = d > r[2] ? d : r[2];
    }
    
    unsigned int axis = 0;
    
    if(r[0] <= r[1] && r[0] <= r[2]) //X cylinder
        axis = 0;
    else if(r[1] <= r[0] && r[1] <= r[2]) //Y cylinder
        axis = 1;
    else //Z cylinder
        axis = 2;
    
    Scalar l_2 = 0;
    for(size_t i=0; i<x.size(); ++i)
    {
        Scalar d = btFabs(x[i].m_floats[axis]);
        l_2 = d > l_2 ? d : l_2;
    }
    
    fdApproxType_ =  GeometryApproxType::CYLINDER;
    fdApproxParams_.resize(2);
    
    if(axis == 0) //X axis
    {
        fdApproxParams_[0] = r[0]; 
        fdApproxParams_[1] = l_2*Scalar(2);
        T_CG2H_ = Transform(Quaternion(0,M_PI_2,0), P_CB_);
    }
    else if(axis == 1) //Y axis
    {
        fdApproxParams_[0] = r[1];
        fdApproxParams_[1] = l_2*Scalar(2);
        T_CG2H_ = Transform(Quaternion(0,0,M_PI_2), P_CB_);
    }
    else
    {
        fdApproxParams_[0] = r[2];
        fdApproxParams_[1] = l_2*Scalar(2);
        T_CG2H_ = Transform(IQ(), P_CB_);
    }
    
    //Added mass and inertia
    Scalar rho = Scalar(1000);
    Ocean* ocn;
    if((ocn = SimulationApp::getApp()->getSimulationManager()->getOcean()) != nullptr)
        rho = ocn->getLiquid().density;

    //Spheroid with the same second moments of volume as the cylinder (along the cylinder axis z)
    Vector3 m, I;
    EllipsoidAddedMass(fdApproxParams_[0]*btSqrt(Scalar(5)/Scalar(4)), fdApproxParams_[0]*btSqrt(Scalar(5)/Scalar(4)), 
                       fdApproxParams_[1]/Scalar(2)*btSqrt(Scalar(5)/Scalar(3)), rho, m, I);
    
    aMassT_ = T_CG2H_.getBasis() * Matrix3(m.x(), 0, 0, 0, m.y(), 0, 0, 0, m.z()) * T_CG2H_.getBasis().transpose();
    aMass_ = Vector3(aMassT_[0][0], aMassT_[1][1], aMassT_[2][2]);
    aI_ = T_CG2H_.getBasis() * I;
    aI_ = Vector3(btFabs(aI_.getX()), btFabs(aI_.getY()), btFabs(aI_.getZ()));

    Vector3 Cd(1.1, 1.1, 0.9); //Cross-flow and axial flow in subcritical regime
    Cd = T_CG2O_.getBasis().inverse() * T_CG2H_.getBasis() * Cd; // To origin frame
    Cd = Vector3(btFabs(Cd.getX()), btFabs(Cd.getY()), btFabs(Cd.getZ()));
    SetHydrodynamicCoefficients(Cd, Vector3(-1,-1,-1)); //Skin friction estimated from the Reynolds number
}

void SolidEntity::ComputeEllipsoidalApprox()
{
#ifdef DEBUG
    cInfo("---- Computing ellipsoidal approximation of geometry for %s ----", getName().c_str());
#endif
    std::vector<Vector3> x = getMeshVertices();
    if(x.size() < 2)
        return;
    for(size_t i=0; i<x.size(); ++i)
        x[i] = T_CG2C_ * x[i] - P_CB_; //Points in CG frame around center of buoyancy
    
    //Ellipsoid with the same second moments of volume as the body (volume integrals over the closed mesh)
    Vector3 c(0,0,0), d(0,0,0);
    bool fitted = false;
    const Mesh* mesh = getPhysicsMesh();
    if(mesh != nullptr)
    {
        Scalar V6 = Scalar(0); //Six times the volume
        Vector3 C6(0,0,0); //Volume-weighted centroid (times 24)
        Vector3 S(0,0,0);  //Second moments of volume (times 120)
        for(size_t i=0; i<mesh->faces.size(); ++i)
        {
            glm::vec3 p1gl = mesh->getVertexPos(i, 0);
            glm::vec3 p2gl = mesh->getVertexPos(i, 1);
            glm::vec3 p3gl = mesh->getVertexPos(i, 2);
            Vector3 v1 = T_CG2C_ * Vector3(p1gl.x, p1gl.y, p1gl.z) - P_CB_;
            Vector3 v2 = T_CG2C_ * Vector3(p2gl.x, p2gl.y, p2gl.z) - P_CB_;
            Vector3 v3 = T_CG2C_ * Vector3(p3gl.x, p3gl.y, p3gl.z) - P_CB_;
            Scalar tetraV6 = v1.dot(v2.cross(v3));
            V6 += tetraV6;
            C6 += (v1 + v2 + v3) * tetraV6;
            for(int j=0; j<3; ++j)
                S.m_floats[j] += tetraV6 * Scalar(2) * (v1[j]*v1[j] + v2[j]*v2[j] + v3[j]*v3[j] + v1[j]*v2[j] + v1[j]*v3[j] + v2[j]*v3[j]);
        }
        
        if(btFabs(V6) > Scalar(1e-12))
        {
            Scalar V = V6/Scalar(6);
            c = C6/(Scalar(4)*V6);
            S /= Scalar(120);
            fitted = true;
            for(int j=0; j<3; ++j)
            {
                Scalar Sjj = S[j]/V - c[j]*c[j]; //Per unit volume, about the centroid (sign of V cancels)
                if(Sjj <= Scalar(0))
                    fitted = false;
                else
                    d.m_floats[j] = btSqrt(Scalar(5) * Sjj); //For an ellipsoid: integral of x^2 dV = V a^2/5
            }
        }
    }
    
    if(!fitted) //Mesh not closed -> minimum-volume enclosing ellipsoid
    {
        //P. Kumar, E.A. Yıldırım, Computing Minimum-Volume Enclosing Axis-Aligned Ellipsoids
        //J Optim Theory Appl (2008) 136: 211–228

        //Initial volume approximation algorithm
        std::vector<Vector3> x0;
        for(size_t k=0; k<3; ++k) //3 dimensions
        {
            //Construct vector for current dimension
            std::vector<Scalar> x_k(x.size());
            for(size_t i=0; i<x.size(); ++i)
                x_k[i] = x[i].m_floats[k];

            //Find range of values
            auto result = std::minmax_element(x_k.begin(), x_k.end());
        
            //Add limits to the set x0
            x0.push_back(x[result.first - x_k.begin()]);
            x0.push_back(x[result.second - x_k.begin()]);
        }

        //Initial sigma
        std::vector<Scalar> sigma(x.size());
        for(size_t i=0; i<x.size(); ++i)
        {
            std::vector<Vector3>::iterator it;
            it = std::find(x0.begin(), x0.end(), x[i]);
            if(it != x0.end())
                sigma[i] = Scalar(1)/Scalar(6);
            else
                sigma[i] = Scalar(0);
        }
    
        //Utility functions
        auto u = [](auto j, auto& x, auto& sigma)
        {
            auto sum = Scalar(0);
            for(size_t i=0; i<x.size(); ++i)
                sum += sigma[i] * x[i].m_floats[j] * x[i].m_floats[j]; 
            return sum;
        };
    
        auto v = [](auto j, auto& x, auto& sigma)
        {
            auto sum = Scalar(0);
            for(size_t i=0; i<x.size(); ++i)
                sum += sigma[i] * x[i].m_floats[j]; 
            return sum;	
        };
    
        auto lambda = [&x, &sigma, &u, &v](int i)
        {
            auto sum = Scalar(0);
            for(int j=0; j<3; ++j)
            {
                auto vj = v(j, x, sigma);
                auto uj = u(j, x, sigma);
                sum += (x[i].m_floats[j] - vj)*(x[i].m_floats[j] - vj)/(3*(uj - vj*vj));
            }
            return sum;
        };

        //Initialize i* and epsilon
        size_t iStar;
        std::vector<Scalar> I(x.size());
        for(size_t i=0; i<x.size(); ++i) I[i] = lambda(i);
        auto IStar = std::max_element(I.begin(), I.end());
        iStar = IStar - I.begin();
        Scalar epsilon = lambda(iStar) - Scalar(1);

        //Run optimization
        Scalar errorTol(0.2);
        Scalar epsilonTol = btPow(Scalar(1) + errorTol, Scalar(2)/Scalar(3)) - Scalar(1);
        size_t maxIter = 10;

        size_t k=0;
#ifdef DEBUG
        cInfo("%s MVAE iteration %ld --> %lf", getName().c_str(), k, epsilon);
#endif
        while(epsilon > epsilonTol && k < maxIter)
        {
            x0.push_back(x[iStar]);
        
            Scalar beta = epsilon/(Scalar(3+1)*(Scalar(1)+epsilon));

            //Update sigma
            for(size_t i=0; i<sigma.size(); ++i)
                sigma[i] = (Scalar(1)-beta)*sigma[i];
            sigma[iStar] += beta;

            //Update i* and epsilon
            for(size_t i=0; i<x.size(); ++i) I[i] = lambda(i);
            IStar = std::max_element(I.begin(), I.end());
            iStar = IStar - I.begin();
            epsilon = lambda(iStar) - Scalar(1);

            ++k;
#ifdef DEBUG
            cInfo("%s MVAE iteration %ld --> %lf\n", getName().c_str(), k, epsilon);
#endif
        }
    
        if(k == 0)
        {
            c.setX((x0[0].x() + x0[1].x())/Scalar(2)); 
            c.setY((x0[2].y() + x0[3].y())/Scalar(2)); 
            c.setZ((x0[4].z() + x0[5].z())/Scalar(2));
            d.setX(btFabs(x0[0].x()-c.x()));
            d.setY(btFabs(x0[2].y()-c.y()));
            d.setZ(btFabs(x0[4].z()-c.z()));
        }
        else
        {
            c.setX(v(0,x,sigma));
            c.setY(v(1,x,sigma));
            c.setZ(v(2,x,sigma));
            for(int j=0; j<3; ++j)
            {
                d.m_floats[j] = Scalar(1)/(Scalar(3)*(u(j,x,sigma) - v(j,x,sigma)*v(j,x,sigma)));
                d.m_floats[j] = Scalar(1)/btSqrt(d.m_floats[j]);
            }
        }
#ifdef DEBUG
        cInfo("Ellipsoid core points: %d", x0.size());
#endif
    }
#ifdef DEBUG
    cInfo("Ellipsoid center: %1.3lf %1.3lf %1.3lf", c.x(), c.y(), c.z());
    cInfo("Ellipsoid axis: %1.3lf %1.3lf %1.3lf", d.x(), d.y(), d.z());
#endif
    
    fdApproxType_ =  GeometryApproxType::ELLIPSOID;
    fdApproxParams_.resize(3);
    fdApproxParams_[0] = d.getX();
    fdApproxParams_[1] = d.getY();
    fdApproxParams_[2] = d.getZ();
    
    //Compute added mass
    Scalar rho = Scalar(1000);
    Ocean* ocn;
    if((ocn = SimulationApp::getApp()->getSimulationManager()->getOcean()) != nullptr)
        rho = ocn->getLiquid().density;

    EllipsoidAddedMass(fdApproxParams_[0], fdApproxParams_[1], fdApproxParams_[2], rho, aMass_, aI_);
    aMassT_ = Matrix3(aMass_.x(), 0, 0, 0, aMass_.y(), 0, 0, 0, aMass_.z()); //Ellipsoid aligned with CG frame
    
    //Set transform with respect to geometry
    Transform ellipsoidTransform;
    ellipsoidTransform.getBasis().setIdentity(); //Aligned with CG frame (for now)
    ellipsoidTransform.setOrigin(P_CB_ + c);
    T_CG2H_ = ellipsoidTransform;

    Vector3 Cd(Scalar(1)/fdApproxParams_[0] , Scalar(1)/fdApproxParams_[1], Scalar(1)/fdApproxParams_[2]);
    Scalar maxCd = btMax(btMax(Cd.x(), Cd.y()), Cd.z());
    Cd /= maxCd;
    Cd = T_CG2O_.getBasis().inverse() * Cd; // To origin frame
    Cd = Vector3(btFabs(Cd.getX()), btFabs(Cd.getY()), btFabs(Cd.getZ()));
    SetHydrodynamicCoefficients(Cd, Vector3(-1,-1,-1)); //Skin friction estimated from the Reynolds number

#ifdef DEBUG
    cInfo("--------------------------------------------------------------------");
#endif
}

//Carlson's symmetric elliptic integral of the second kind R_D(x,y,z), computed with the duplication algorithm
//(B.C. Carlson, "Numerical computation of real or complex elliptic integrals", Numerical Algorithms 10, 1995)
static Scalar CarlsonRD(Scalar x, Scalar y, Scalar z)
{
    const Scalar C1 = Scalar(3)/Scalar(14), C2 = Scalar(1)/Scalar(6), C3 = Scalar(9)/Scalar(22), C4 = Scalar(3)/Scalar(26);
    const Scalar C5 = Scalar(0.25)*C3, C6 = Scalar(1.5)*C4;
    Scalar sum = Scalar(0), fac = Scalar(1);
    Scalar ave(1), delx(0), dely(0), delz(0);
    for(int i=0; i<100; ++i)
    {
        Scalar sx = btSqrt(x), sy = btSqrt(y), sz = btSqrt(z);
        Scalar lambda = sx*(sy + sz) + sy*sz;
        sum += fac/(sz*(z + lambda));
        fac *= Scalar(0.25);
        x = Scalar(0.25)*(x + lambda);
        y = Scalar(0.25)*(y + lambda);
        z = Scalar(0.25)*(z + lambda);
        ave = Scalar(0.2)*(x + y + Scalar(3)*z);
        delx = (ave - x)/ave;
        dely = (ave - y)/ave;
        delz = (ave - z)/ave;
        if(btMax(btMax(btFabs(delx), btFabs(dely)), btFabs(delz)) < Scalar(0.0015))
            break;
    }
    Scalar ea = delx*dely, eb = delz*delz, ec = ea - eb, ed = ea - Scalar(6)*eb, ee = ed + ec + ec;
    return Scalar(3)*sum + fac*(Scalar(1) + ed*(-C1 + C5*ed - C6*delz*ee) + delz*(C2*ee + delz*(-C3*ec + delz*C4*ea)))/(ave*btSqrt(ave));
}

void SolidEntity::EllipsoidLambCoefficients(Scalar a, Scalar b, Scalar c, Scalar& alpha0, Scalar& beta0, Scalar& gamma0)
{
    //alpha0 = abc * int_0^inf dl/((a^2+l) sqrt((a^2+l)(b^2+l)(c^2+l))) = 2/3 abc R_D(b^2,c^2,a^2), beta0 and gamma0 analogous
    //(H. Lamb, Hydrodynamics, 1932, Sec. 114)
    Scalar k = Scalar(2)/Scalar(3)*a*b*c;
    alpha0 = k * CarlsonRD(b*b, c*c, a*a);
    beta0 = k * CarlsonRD(c*c, a*a, b*b);
    gamma0 = k * CarlsonRD(a*a, b*b, c*c);
}

void SolidEntity::EllipsoidAddedMass(Scalar a, Scalar b, Scalar c, Scalar rho, Vector3& addedMass, Vector3& addedInertia)
{
    //Added mass and added moments of inertia of an ellipsoid with semi-axes a, b, c along x, y, z, in unbounded ideal fluid
    //(H. Lamb, Hydrodynamics, 1932, Sec. 373; A.I. Korotkin, Added Masses of Ship Structures, 2009)
    Scalar alpha0, beta0, gamma0;
    EllipsoidLambCoefficients(a, b, c, alpha0, beta0, gamma0);
    Scalar md = Scalar(4)/Scalar(3)*M_PI*rho*a*b*c; //Displaced mass
    addedMass = Vector3(alpha0/(Scalar(2)-alpha0), beta0/(Scalar(2)-beta0), gamma0/(Scalar(2)-gamma0)) * md;

    //Rotation about an axis, with (p,q) the two other semi-axes and (P,Q) the corresponding coefficients
    auto rotational = [md](Scalar p, Scalar q, Scalar P, Scalar Q)
    {
        Scalar p2 = p*p, q2 = q*q;
        Scalar dpq = p2 - q2;
        if(btFabs(dpq) < Scalar(1e-6)*(p2 + q2)) //Body of revolution around this axis
            return Scalar(0);
        return md/Scalar(5) * dpq*dpq * (Q - P)/(Scalar(2)*dpq + (p2 + q2)*(P - Q));
    };
    addedInertia = Vector3(rotational(b, c, beta0, gamma0), rotational(c, a, gamma0, alpha0), rotational(a, b, alpha0, beta0));
}

void SolidEntity::BuildGraphicalObject()
{
    if(phyMesh_ == nullptr || !SimulationApp::getApp()->hasGraphics())
        return;

    if (graObjectId_ > -1) // Object already built
        return;
        
    graObjectId_ = static_cast<GraphicalSimulationApp*>(SimulationApp::getApp())->getGLPipeline()->getContent()->BuildObject(phyMesh_.get());
    phyObjectId_ = graObjectId_;
}

void SolidEntity::BuildRigidBody(btDynamicsWorld* world)
{
    if(rigidBody_ == nullptr)
    {
        motionState_ = std::make_unique<btDefaultMotionState>();
        
        //Generate collision shape
        std::unique_ptr<btCollisionShape> shape0 = BuildCollisionShape();
        std::unique_ptr<btCompoundShape> shape;
        
        if(shape0->getShapeType() == COMPOUND_SHAPE_PROXYTYPE) //For a compound shape just move the children to avoid additional level
        {
            shape.reset(static_cast<btCompoundShape*>(shape0.release()));
            for(int i=0; i < shape->getNumChildShapes(); ++i)
                shape->updateChildTransform(i, T_CG2C_ * shape->getChildTransform(i), true);
        }
        else //For other shapes, create compound shape which allow for the shift against gravity centre
        {
            shape = std::make_unique<btCompoundShape>();
            shape->addChildShape(T_CG2C_, shape0.release());
        }
        collisionShape_ = std::move(shape);
        
        //Construct Bullet rigid body
        Scalar M = getAugmentedMass();
        Vector3 I = getAugmentedInertia();
        
        btRigidBody::btRigidBodyConstructionInfo rigidBodyCI(M, motionState_.get(), collisionShape_.get(), I);
        rigidBodyCI.m_friction = rigidBodyCI.m_rollingFriction = rigidBodyCI.m_restitution = Scalar(0.); //not used
        rigidBodyCI.m_linearDamping = rigidBodyCI.m_angularDamping = SimulationApp::getApp()->getSimulationManager()->getBodyDamping();
        rigidBodyCI.m_additionalDamping = false;

        rigidBody_ = std::make_unique<btRigidBody>(rigidBodyCI);
        rigidBody_->setUserPointer(this);
        rigidBody_->setFlags(rigidBody_->getFlags() | BT_ENABLE_GYROSCOPIC_FORCE_IMPLICIT_BODY);
        rigidBody_->setCollisionFlags(rigidBody_->getCollisionFlags() | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
        //rigidBody->setContactProcessingThreshold(0.002);
        //rigidBody->setCcdMotionThreshold(0.01);
        //rigidBody->setCcdSweptSphereRadius(0.9);
        
        // Soft contact
        if(contactK_ > Scalar(0))
            rigidBody_->setContactStiffnessAndDamping(contactK_, contactD_);

        cInfo("Built rigid body %s [mass: %1.3lf; inertia: %1.3lf, %1.3lf, %1.3lf; volume: %1.1lf]", getName().c_str(), mass_, Ipri_.x(), Ipri_.y(), Ipri_.z(), volume_*1e6);
    }
}

void SolidEntity::BuildMultibodyLinkCollider(btMultiBody *mb, unsigned int child, btSoftMultiBodyDynamicsWorld* world)
{
    if(multibodyCollider_ == nullptr)
    {
        //Generate collision shape
        std::unique_ptr<btCollisionShape> shape0 = BuildCollisionShape();
        std::unique_ptr<btCompoundShape> shape;
        if(shape0->getShapeType() == COMPOUND_SHAPE_PROXYTYPE) //For a compound shape just move the children to avoid additional level
        {
            shape.reset(static_cast<btCompoundShape*>(shape0.release()));
            for(int i=0; i < shape->getNumChildShapes(); ++i)
            {
                shape->getChildShape(i)->setMargin(Scalar(0));
                shape->updateChildTransform(i, T_CG2C_ * shape->getChildTransform(i), true);
            }
        }
        else //For other shapes, create compound shape which allow for the shift against gravity centre
        {
            shape = std::make_unique<btCompoundShape>();
            shape0->setMargin(Scalar(0));
            shape->addChildShape(T_CG2C_, shape0.release()); 
        }
        shape->setMargin(Scalar(0));
        collisionShape_ = std::move(shape);
        
        //Construct Bullet multi-body link
        multibodyCollider_ = std::make_unique<btMultiBodyLinkCollider>(mb, child - 1);
        multibodyCollider_->setCollisionShape(collisionShape_.get());
        multibodyCollider_->setUserPointer(this); //HAS TO BE AFTER SETTING COLLISION SHAPE TO PROPAGATE TO ALL OF COMPOUND SUBSHAPES!!!!!
        multibodyCollider_->setFriction(Scalar(0));
        multibodyCollider_->setRestitution(Scalar(0));
        multibodyCollider_->setRollingFriction(Scalar(0));
        multibodyCollider_->setSpinningFriction(Scalar(0));
        multibodyCollider_->setCollisionFlags(multibodyCollider_->getCollisionFlags() | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
        multibodyCollider_->setActivationState(DISABLE_DEACTIVATION);
        
        if(child > 0)
            mb->getLink(child - 1).m_collider = multibodyCollider_.get();
        else
            mb->setBaseCollider(multibodyCollider_.get());
        
        world->addCollisionObject(multibodyCollider_.get(), MASK_DYNAMIC, MASK_GHOST | MASK_STATIC | MASK_DYNAMIC | MASK_ANIMATED_COLLIDING);
        
        if(contactK_ > Scalar(0))
            multibodyCollider_->setContactStiffnessAndDamping(contactK_, contactD_);
        
        //Graphics
        BuildGraphicalObject();
        
        cInfo("Built multibody link %s (mass[kg]: %1.3lf; inertia[kgm2]: %1.3lf, %1.3lf, %1.3lf; volume[cm3]: %1.1lf)", getName().c_str(), mass_, Ipri_.x(), Ipri_.y(), Ipri_.z(), volume_*1e6);
    }
}

void SolidEntity::AddToSimulation(SimulationManager* sm)
{
    AddToSimulation(sm, Transform::getIdentity());
}

void SolidEntity::AddToSimulation(SimulationManager* sm, const Transform& origin)
{
    if(rigidBody_ == nullptr)
    {
        // Build
        BuildRigidBody(sm->getDynamicsWorld());
        BuildGraphicalObject();
        
        // Setup sleeping
        Scalar linSleep, angSleep;
        sm->getSleepingThresholds(linSleep, angSleep);
        if(linSleep <= Scalar(0) || angSleep <= Scalar(0))        
            rigidBody_->setActivationState(DISABLE_DEACTIVATION);
        else
        {
            rigidBody_->setSleepingThresholds(linSleep, angSleep);
        }

        // Add to world
        Transform Tcg = origin * T_CG2O_.inverse();
        motionState_ = std::make_unique<btDefaultMotionState>(Tcg);
        rigidBody_->setMotionState(motionState_.get());
        sm->getDynamicsWorld()->addRigidBody(rigidBody_.get(), MASK_DYNAMIC, MASK_GHOST | MASK_STATIC | MASK_DYNAMIC | MASK_ANIMATED_COLLIDING);
    }
}

void SolidEntity::RemoveFromSimulation(SimulationManager* sm)
{
    sm->getDynamicsWorld()->removeRigidBody(rigidBody_.get());
    rigidBody_.reset();
}

void SolidEntity::UpdateAcceleration(Scalar dt)
{
    Vector3 currentV = getLinearVelocity();
    Vector3 currentOmega = getAngularVelocity();
    linearAcc_ = (currentV - lastV_)/dt;
    angularAcc_ = (currentOmega - lastOmega_)/dt;
    lastV_ = currentV;
    lastOmega_ = currentOmega;
}

void SolidEntity::ApplyGravity(const Vector3& g)
{
    if(rigidBody_ != nullptr)
    {
        rigidBody_->applyCentralForce(g * mass_);
    }
}

void SolidEntity::ApplyCentralForce(const Vector3& force)
{
    if(rigidBody_ != nullptr)
        rigidBody_->applyCentralForce(force);
    else if(multibodyCollider_ != nullptr)
    {
        btMultiBody* multiBody = multibodyCollider_->m_multiBody;
        int index = multibodyCollider_->m_link;
        
        if(index == -1) //base
            multiBody->addBaseForce(force);
        else
            multiBody->addLinkForce(index, force);
    }
}

void SolidEntity::ApplyTorque(const Vector3& torque)
{
    if(rigidBody_ != nullptr)
        rigidBody_->applyTorque(torque);
    else if(multibodyCollider_ != nullptr)
    {
        btMultiBody* multiBody = multibodyCollider_->m_multiBody;
        int index = multibodyCollider_->m_link;
        
        if(index == -1) //base
            multiBody->addBaseTorque(torque);
        else
            multiBody->addLinkTorque(index, torque);
    }
}

BodyFluidPosition SolidEntity::CheckBodyFluidPosition(Ocean* ocn)
{
    Vector3 aabbMin, aabbMax;
    getAABB(aabbMin, aabbMax);
    Vector3 d = aabbMax-aabbMin;
    
    unsigned int underwater = 0;
    if(ocn->GetDepth(aabbMin) > Scalar(0)) ++underwater;
    if(ocn->GetDepth(aabbMax) > Scalar(0)) ++underwater;
    if(ocn->GetDepth(aabbMin + Vector3(d.x(), 0, 0)) > Scalar(0)) ++underwater;
    if(ocn->GetDepth(aabbMin + Vector3(0, d.y(), 0)) > Scalar(0)) ++underwater;
    if(ocn->GetDepth(aabbMin + Vector3(d.x(), d.y(), 0)) > Scalar(0)) ++underwater;
    if(ocn->GetDepth(aabbMin + Vector3(0, 0, d.z())) > Scalar(0)) ++underwater;
    if(ocn->GetDepth(aabbMin + Vector3(d.x(), 0, d.z())) > Scalar(0)) ++underwater;
    if(ocn->GetDepth(aabbMin + Vector3(0, d.y(), d.z())) > Scalar(0)) ++underwater;
    
    if(underwater == 0)
        return BodyFluidPosition::OUTSIDE;
    else if(underwater == 8)
        return BodyFluidPosition::INSIDE;
    else
        return BodyFluidPosition::CROSSING_SURFACE;
}

Scalar SolidEntity::SkinFrictionCoefficient(Ocean* ocn, Scalar speed, Scalar length)
{
    //Reynolds number of the flow along the body
    Fluid liquid = ocn->getLiquid();
    Scalar nu = liquid.density > Scalar(0) && liquid.viscosity > Scalar(0) ? liquid.viscosity/liquid.density : Scalar(1e-6); //Kinematic viscosity [m^2/s]
    Scalar Re = btMax(speed * length / nu, Scalar(1e4)); //Below the limit the friction forces are negligible
    
    //Friction coefficient of a turbulent boundary layer (ITTC-1957 model-ship correlation line)
    Scalar logRe = btLog(Re)/btLog(Scalar(10));
    return Scalar(0.075)/((logRe - Scalar(2)) * (logRe - Scalar(2)));
}

Scalar SolidEntity::getShellBuoyancyRatio() const
{
    //Shell bodies are considered flooded (only the wall displaces liquid), consistently with the fully submerged case.
    //Buoyancy of a partially submerged body is computed by integrating over the outer surface, so it has to be scaled.
    if(thick_ <= Scalar(0) || phyMesh_ == nullptr)
        return Scalar(1);
    
    Scalar V6 = Scalar(0);
    for(size_t i=0; i<phyMesh_->faces.size(); ++i)
    {
        glm::vec3 v1 = phyMesh_->getVertexPos(i, 0);
        glm::vec3 v2 = phyMesh_->getVertexPos(i, 1);
        glm::vec3 v3 = phyMesh_->getVertexPos(i, 2);
        V6 += (Scalar)glm::dot(v1, glm::cross(v2, v3));
    }
    Scalar enclosedVolume = btFabs(V6)/Scalar(6);
    return enclosedVolume > volume_ ? volume_/enclosedVolume : Scalar(1);
}

Scalar SolidEntity::getCharacteristicLength() const
{
    return btSqrt(surface_/Scalar(M_PI)); //Diameter of a sphere with the same surface area
}

//Weight of an anisotropic coefficient in a given direction (squared components of a unit vector sum to one)
static inline Scalar DirectionalCoefficient(const Vector3& dir, const Vector3& coeff)
{
    return dir.getX() * dir.getX() * coeff.getX() + dir.getY() * dir.getY() * coeff.getY() + dir.getZ() * dir.getZ() * coeff.getZ();
}

void SolidEntity::CorrectHydrodynamicForces(Ocean* ocn, Vector3& _Fdq, Vector3& _Tdq, Vector3& _Fdf, Vector3& _Tdf, 
    const Vector3& fdCd, const Vector3& fdCf, const Transform& T_O, Scalar Cf0)
{
    Matrix3 toOrigin = T_O.getBasis().inverse();
    Scalar rho = ocn->getLiquid().density;

    //Skin friction coefficients not defined by the user (negative) are estimated for turbulent flow
    Vector3 Cf(fdCf.getX() < Scalar(0) ? Cf0 : fdCf.getX(),
               fdCf.getY() < Scalar(0) ? Cf0 : fdCf.getY(),
               fdCf.getZ() < Scalar(0) ? Cf0 : fdCf.getZ());

    Vector3 Fdq = (toOrigin * _Fdq).safeNormalize();
    _Fdq = Scalar(0.5) * rho * DirectionalCoefficient(Fdq, fdCd) * _Fdq; //0.5*rho*Cd*S*v^2 from drag equation    

    Vector3 Tdq = (toOrigin * _Tdq).safeNormalize();
    _Tdq = Scalar(0.5) * rho * DirectionalCoefficient(Tdq, fdCd) * _Tdq; //0.5*rho*Cd*S*v^2 from drag equation

    Vector3 Fdf = (toOrigin * _Fdf).safeNormalize();
    _Fdf = Scalar(0.5) * rho * DirectionalCoefficient(Fdf, Cf) * _Fdf; //0.5*rho*Cf*S*v^2 from skin friction equation
    
    Vector3 Tdf = (toOrigin * _Tdf).safeNormalize();
    _Tdf = Scalar(0.5) * rho * DirectionalCoefficient(Tdf, Cf) * _Tdf; //0.5*rho*Cf*S*v^2 from skin friction equation
}

void SolidEntity::ComputeHydrodynamicForcesSurface(const HydrodynamicsSettings& settings, const Mesh* mesh, Ocean* ocn, const Transform& T_CG, const Transform& T_C,
                                            const Vector3& _v, const Vector3& _omega, Vector3& _Fb, Vector3& _Tb, Vector3& _Fdq, Vector3& _Tdq, Vector3& _Fdf, Vector3& _Tdf, 
                                            Scalar& _Swet, Scalar& _Vsub, Renderable& debug)
{
    if(mesh == nullptr)
    {
        if(settings.reallisticBuoyancy)
        {
            _Fb.setZero();
            _Tb.setZero();
        }

        if(settings.dampingForces)
        {
            _Fdq.setZero();
            _Tdq.setZero();
            _Fdf.setZero();
            _Tdf.setZero();
        }

        _Swet = Scalar(0);
        _Vsub = Scalar(0);
        return;
    }

    //Clear outputs (some of them are only written when the body is submerged or when a computation is enabled)
    _Fb.setZero();
    _Tb.setZero();
    _Fdq.setZero();
    _Tdq.setZero();
    _Fdf.setZero();
    _Tdf.setZero();
    _Swet = Scalar(0);
    _Vsub = Scalar(0);
    
    auto debugPoints = debug.getDataAsPoints();

    //Computation with floats (geometry has float precision)
    glm::vec3 Fb(0.f);
    glm::vec3 Tb(0.f);
    glm::vec3 Fdq(0.f);
    glm::vec3 Tdq(0.f);
    glm::vec3 Fdf(0.f);
    glm::vec3 Tdf(0.f);
    GLfloat Swet(0.f);
    GLfloat Vsub(0.f);
    glm::mat4 TCG = glMatrixFromTransform(T_CG);
    glm::mat4 TC = glMatrixFromTransform(T_C);
    glm::vec3 v = glVectorFromVector(_v);
    glm::vec3 omega = glVectorFromVector(_omega);
    glm::vec3 CBsub(0.f);
   
    //Calculate fluid dynamics forces and torques
    glm::vec3 p = glm::vec3(TCG[3]);
    glm::vec3 p0 = p; //Point used as a center of mesh for volume calculation.
    p0.z = 0.f;       //When the robot is far from the world origin numerical erros would explode without translating the mesh data!
    
    //Loop through all faces...
    const VertexPosAccessor vertexPos = mesh->getVertexPosAccessor();
    const size_t numFaces = mesh->faces.size();

    for(size_t i=0; i<numFaces; ++i)
    {
        //Global coordinates
        const Face& face = mesh->faces[i];
        const glm::vec3& p1gl = vertexPos(face, 0);
        const glm::vec3& p2gl = vertexPos(face, 1);
        const glm::vec3& p3gl = vertexPos(face, 2);
        glm::vec3 p1 = glm::vec3(TC * glm::vec4(p1gl, 1.f));
        glm::vec3 p2 = glm::vec3(TC * glm::vec4(p2gl, 1.f));
        glm::vec3 p3 = glm::vec3(TC * glm::vec4(p3gl, 1.f));
        
        //Check if face underwater
        GLfloat depth[3];
        depth[0] = ocn->GetDepth(p1);
        depth[1] = ocn->GetDepth(p2);
        depth[2] = ocn->GetDepth(p3);
        
        if(depth[0] < 0.f && depth[1] < 0.f && depth[2] < 0.f)
            continue;
        
        //Calculate face properties
        glm::vec3 fc;
        glm::vec3 fn;
        glm::vec3 fn1;
        GLfloat A;
        
        if(depth[0] < 0.f) //Vertex 1 above water
        {
            if(depth[1] < 0.f) //Two vertices above water (triangle)
            {
                p1 = p3 + (p1-p3) * (depth[2]/(fabsf(depth[0]) + depth[2]));
                p2 = p3 + (p2-p3) * (depth[2]/(fabsf(depth[1]) + depth[2]));
                //p3 without change
                
                //Volume properties
                glm::vec3 p01 = p1-p0;
                glm::vec3 p02 = p2-p0;
                glm::vec3 p03 = p3-p0;
                glm::vec3 tetraCG = (p01+p02+p03)/4.f;
                GLfloat tetraV6 = glm::dot(p01, glm::cross(p02, p03));
                CBsub += tetraCG * tetraV6;
                Vsub += tetraV6;
                
                //Face properties
                glm::vec3 fv1 = p2-p1; //One side of the face (triangle)
                glm::vec3 fv2 = p3-p1; //Another side of the face (triangle)
                fc = (p1+p2+p3)/3.f; //Face centroid
        
                fn = glm::cross(fv1, fv2); //Normal of the face (length != 1)
                GLfloat len = glm::length2(fn); //Double area
                if(len < 1e-12f) continue;
                len = glm::sqrt(len);
                fn1 = fn/len; //Normalised normal (length = 1)
                A = len/2.f; //Area of the face (triangle)         
#ifdef DEBUG_HYDRO
                debugPoints->push_back(p1);
                debugPoints->push_back(p2);
                debugPoints->push_back(p2);
                debugPoints->push_back(p3);
                debugPoints->push_back(p3);
                debugPoints->push_back(p1);
#endif
            }
            else if(depth[2] < 0.f) //Two vertices above water (triangle)
            {
                p1 = p2 + (p1-p2) * (depth[1]/(fabsf(depth[0]) + depth[1]));
                //p2 without change
                p3 = p2 + (p3-p2) * (depth[1]/(fabsf(depth[2]) + depth[1]));
                
                //Volume properties
                glm::vec3 p01 = p1-p0;
                glm::vec3 p02 = p2-p0;
                glm::vec3 p03 = p3-p0;
                glm::vec3 tetraCG = (p01+p02+p03)/4.f;
                GLfloat tetraV6 = glm::dot(p01, glm::cross(p02, p03));
                CBsub += tetraCG * tetraV6;
                Vsub += tetraV6;
                
                //Face properties
                glm::vec3 fv1 = p2-p1; //One side of the face (triangle)
                glm::vec3 fv2 = p3-p1; //Another side of the face (triangle)
                fc = (p1+p2+p3)/3.f; //Face centroid
        
                fn = glm::cross(fv1, fv2); //Normal of the face (length != 1)
                GLfloat len = glm::length2(fn);
                if(len < 1e-12f) continue;
                len = glm::sqrt(len);
                fn1 = fn/len; //Normalised normal (length = 1)
                A = len/2.f; //Area of the face (triangle)         
#ifdef DEBUG_HYDRO
                debugPoints->push_back(p1);
                debugPoints->push_back(p2);
                debugPoints->push_back(p2);
                debugPoints->push_back(p3);
                debugPoints->push_back(p3);
                debugPoints->push_back(p1);
#endif
            }
            else //depth[1] >= 0 && depth[2] >= 0 --> Two vertices under water (quad = two triangles)
            {
                //Quad!!!!
                glm::vec3 p4 = p3 + (p1-p3) * (depth[2]/(fabsf(depth[0]) + depth[2]));
                p1 = p2 + (p1-p2) * (depth[1]/(fabsf(depth[0]) + depth[1]));
                //p2 without change
                //p3 without change
                
                //Volume properties
                //Tetra 1
                glm::vec3 p01 = p1-p0;
                glm::vec3 p02 = p2-p0;
                glm::vec3 p03 = p3-p0;
                glm::vec3 tetraCG = (p01+p02+p03)/4.f;
                GLfloat tetraV6 = glm::dot(p01, glm::cross(p02, p03));
                CBsub += tetraCG * tetraV6;
                Vsub += tetraV6;
                //Tetra 2
                glm::vec3 p04 = p4-p0;
                tetraCG = (p01+p03+p04)/4.f;
                tetraV6 = glm::dot(p01, glm::cross(p03, p04));
                CBsub += tetraCG * tetraV6;
                Vsub += tetraV6;
                
                //Face properties
                glm::vec3 fv1 = p2-p1;
                glm::vec3 fv2 = p4-p1;
                glm::vec3 fv3 = p2-p3;
                glm::vec3 fv4 = p4-p3;
                fc = (p1 + p2 + p3 + p4)/4.f;
                
                fn = glm::cross(fv1, fv2);
                GLfloat len = glm::length2(fn);
                if(len < 1e-12f) continue;
                len = glm::sqrt(len);
                fn1 = fn/len;
                A = (len + glm::length(glm::cross(fv3, fv4)))/2.f; //Quad
                fn = fn1 * A;
#ifdef DEBUG_HYDRO
                debugPoints->push_back(p1);
                debugPoints->push_back(p2);
                debugPoints->push_back(p2);
                debugPoints->push_back(p3);
                debugPoints->push_back(p3);
                debugPoints->push_back(p4);
                debugPoints->push_back(p4);
                debugPoints->push_back(p1);
#endif  
            }
        }
        else if(depth[1] < 0.f)
        {
            if(depth[2] < 0.f)
            {
                //p1 without change
                p2 = p1 + (p2-p1) * (depth[0]/(fabsf(depth[1]) + depth[0]));
                p3 = p1 + (p3-p1) * (depth[0]/(fabsf(depth[2]) + depth[0]));
                
                //Volume properties
                glm::vec3 p01 = p1-p0;
                glm::vec3 p02 = p2-p0;
                glm::vec3 p03 = p3-p0;
                glm::vec3 tetraCG = (p01+p02+p03)/4.f;
                GLfloat tetraV6 = glm::dot(p01, glm::cross(p02, p03));
                CBsub += tetraCG * tetraV6;
                Vsub += tetraV6;

                //Face properties
                glm::vec3 fv1 = p2-p1; //One side of the face (triangle)
                glm::vec3 fv2 = p3-p1; //Another side of the face (triangle)
                fc = (p1+p2+p3)/3.f; //Face centroid
        
                fn = glm::cross(fv1, fv2); //Normal of the face (length != 1)
                GLfloat len = glm::length2(fn);
                if(len < 1e-12f) continue;
                len = glm::sqrt(len);
                fn1 = fn/len; //Normalised normal (length = 1)
                A = len/2.f; //Area of the face (triangle)
#ifdef DEBUG_HYDRO
                debugPoints->push_back(p1);
                debugPoints->push_back(p2);
                debugPoints->push_back(p2);
                debugPoints->push_back(p3);
                debugPoints->push_back(p3);
                debugPoints->push_back(p1);
#endif                
            }
            else
            {
                //Quad!!!!
                glm::vec3 p4 = p3 + (p2-p3) * (depth[2]/(fabsf(depth[1]) + depth[2]));
                //p1 without change
                p2 = p1 + (p2-p1) * (depth[0]/(fabsf(depth[1]) + depth[0]));
                //p3 without change
                
                //Volume properties
                //Tetra 1
                glm::vec3 p01 = p1-p0;
                glm::vec3 p02 = p2-p0;
                glm::vec3 p03 = p3-p0;
                glm::vec3 tetraCG = (p01+p02+p03)/4.f;
                GLfloat tetraV6 = glm::dot(p01, glm::cross(p02, p03));
                CBsub += tetraCG * tetraV6;
                Vsub += tetraV6;
                //Tetra 2
                glm::vec3 p04 = p4-p0;
                tetraCG = (p02+p04+p03)/4.f;
                tetraV6 = glm::dot(p02, glm::cross(p04, p03));
                CBsub += tetraCG * tetraV6;
                Vsub += tetraV6;              

                //Face properties
                glm::vec3 fv1 = p2-p1;
                glm::vec3 fv2 = p3-p1;
                glm::vec3 fv3 = p2-p3;
                glm::vec3 fv4 = p4-p3;
                fc = (p1 + p2 + p3 + p4)/4.f;
                fn = glm::cross(fv1, fv2); //Triangle 1
                GLfloat len = glm::length2(fn);
                if(len < 1e-12f) continue;    
                len = glm::sqrt(len);
                fn1 = fn/len;
                A = (len + glm::length(glm::cross(fv3, fv4)))/2.f; //Quad
                fn = fn1 * A;
#ifdef DEBUG_HYDRO
                debugPoints->push_back(p1);
                debugPoints->push_back(p2);
                debugPoints->push_back(p2);
                debugPoints->push_back(p4);
                debugPoints->push_back(p4);
                debugPoints->push_back(p3);
                debugPoints->push_back(p3);
                debugPoints->push_back(p1);
#endif                 
            }
        }
        else if(depth[2] < 0.f)
        {
            //Quad!!!!
            glm::vec3 p4 = p1 + (p3-p1) * (depth[0]/(fabsf(depth[2]) + depth[0]));
            //p1 without change
            //p2 without change
            p3 = p2 + (p3-p2) * (depth[1]/(fabsf(depth[2]) + depth[1]));
                
            //Volume properties
            //Tetra 1
            glm::vec3 p01 = p1-p0;
            glm::vec3 p02 = p2-p0;
            glm::vec3 p03 = p3-p0;
            glm::vec3 tetraCG = (p01+p02+p03)/4.f;
            GLfloat tetraV6 = glm::dot(p01, glm::cross(p02, p03));
            CBsub += tetraCG * tetraV6;
            Vsub += tetraV6;
            //Tetra 2
            glm::vec3 p04 = p4-p0;
            tetraCG = (p01+p03+p04)/4.f;
            tetraV6 = glm::dot(p01, glm::cross(p03, p04));
            CBsub += tetraCG * tetraV6;
            Vsub += tetraV6;
            
            //Face properties
            glm::vec3 fv1 = p2-p1;
            glm::vec3 fv2 = p4-p1;
            glm::vec3 fv3 = p2-p3;
            glm::vec3 fv4 = p4-p3;
            fc = (p1 + p2 + p3 + p4)/4.f;
            fn = glm::cross(fv1, fv2);
            GLfloat len = glm::length2(fn);
            if(len < 1e-12f) continue;
            len = glm::sqrt(len);
            fn1 = fn/len;
            A = (len + glm::length(glm::cross(fv3, fv4)))/2.f; //Quad
            fn = fn1 * A;
#ifdef DEBUG_HYDRO
            debugPoints->push_back(p1);
            debugPoints->push_back(p2);
            debugPoints->push_back(p2);
            debugPoints->push_back(p3);
            debugPoints->push_back(p3);
            debugPoints->push_back(p4);
            debugPoints->push_back(p4);
            debugPoints->push_back(p1);
#endif             
        }
        else //All underwater
        {
            //Volume properties
            glm::vec3 p01 = p1-p0;
            glm::vec3 p02 = p2-p0;
            glm::vec3 p03 = p3-p0;
            glm::vec3 tetraCG = (p01+p02+p03)/4.f;
            GLfloat tetraV6 = glm::dot(p01, glm::cross(p02, p03));
            CBsub += tetraCG * tetraV6;
            Vsub += tetraV6;

            //Face properties
            glm::vec3 fv1 = p2-p1; //One side of the face (triangle)
            glm::vec3 fv2 = p3-p1; //Another side of the face (triangle)
            fn = glm::cross(fv1, fv2); //Normal of the face (length != 1)
            GLfloat len = glm::length2(fn);
            if(len < 1e-12f) continue;
            len = glm::sqrt(len);
            fn1 = fn/len; //Normalised normal (length = 1)
            A = len/2.f; //Area of the face (triangle)
            fc = (p1+p2+p3)/3.f; //Face centroid
#ifdef DEBUG_HYDRO
            debugPoints->push_back(p1);
            debugPoints->push_back(p2);
            debugPoints->push_back(p2);
            debugPoints->push_back(p3);
            debugPoints->push_back(p3);
            debugPoints->push_back(p1);
#endif             
        }

        //Buoyancy force
        if(settings.reallisticBuoyancy && ocn->hasWaves())
        {
            GLfloat depthc = ocn->GetDepth(fc);
            glm::vec3 Fbi = -fn1 * A * depthc; //Buoyancy force per face (based on pressure)        
            
            //Accumulate
            Fb += Fbi;
            Tb += glm::cross(fc-p, Fbi);
        }
        
        //Damping force
        if(settings.dampingForces)
        {
            glm::vec3 vc = ocn->GetFluidVelocity(fc) - (v + glm::cross(omega, fc-p));
            GLfloat vc_n = glm::dot(vc, fn1);
            glm::vec3 vn = vc_n  * fn1; //Normal velocity
            glm::vec3 vt = vc - vn; //Tangent velocity
            
            if(vc_n < -1e-12f) //If liquid is approaching the surface
            {
                //Pressure drag acts along the flow, on the face area projected onto the flow direction (|v|^2 * A * cos(theta))
                glm::vec3 quadratic = vc * -vc_n * A;
                Fdq += quadratic;
                Tdq += glm::cross(fc - p, quadratic);
            }

            GLfloat vmag2 = glm::length2(vt);
            if(vmag2 > 1e-9f)
            {
                //Skin friction of a turbulent boundary layer (|vt| * vt * A)
                glm::vec3 skin = vt * sqrtf(vmag2) * A;
                Fdf += skin;
                Tdf += glm::cross(fc - p, skin);
            }
        }

        //Wetted surface area
        Swet += A;
    }

    //Submerged volume
    if(Vsub > 1e-9f)
        _Vsub = Vsub/6.f;

    //Buoyancy
    if(settings.reallisticBuoyancy && Vsub > 1e-9f)
    {
        if(ocn->hasWaves())
        {
            Fb *= ocn->getLiquid().density * SimulationApp::getApp()->getSimulationManager()->getGravity().getZ();
            Tb *= ocn->getLiquid().density * SimulationApp::getApp()->getSimulationManager()->getGravity().getZ();
            _Fb = Vector3(Fb.x, Fb.y, Fb.z);
            _Tb = Vector3(Tb.x, Tb.y, Tb.z);
        }
        else
        {
            CBsub = CBsub/Vsub + p0;
            Vector3 _CBsub(CBsub.x, CBsub.y, CBsub.z);
            _Fb = -_Vsub * ocn->getLiquid().density * SimulationApp::getApp()->getSimulationManager()->getGravity();
            _Tb = (_CBsub - T_CG.getOrigin()).cross(_Fb);
        }        
    }
    
    //Damping forces
    if(settings.dampingForces)
    {
        _Fdq = Vector3(Fdq.x, Fdq.y, Fdq.z);
        _Tdq = Vector3(Tdq.x, Tdq.y, Tdq.z);
        _Fdf = Vector3(Fdf.x, Fdf.y, Fdf.z);
        _Tdf = Vector3(Tdf.x, Tdf.y, Tdf.z);
    }

    //Wetted surface area
    _Swet = Swet;
}

void SolidEntity::ComputeHydrodynamicForcesSubmerged(const Mesh* mesh, Ocean* ocn, const Transform& T_CG, const Transform& T_C,
                                              const Vector3& _v, const Vector3& _omega, Vector3& _Fdq, Vector3& _Tdq, Vector3& _Fdf, Vector3& _Tdf)
{
    if(mesh == nullptr)
    {
        _Fdq.setZero();
        _Tdq.setZero();
        _Fdf.setZero();
        _Tdf.setZero();
        return;
    }

    //Computation with floats (geometry has float precision)
    glm::vec3 Fdq(0.f);
    glm::vec3 Tdq(0.f);
    glm::vec3 Fdf(0.f);
    glm::vec3 Tdf(0.f);
    glm::mat4 TCG = glMatrixFromTransform(T_CG);
    glm::mat4 TC = glMatrixFromTransform(T_C);
    glm::vec3 v = glVectorFromVector(_v);
    glm::vec3 omega = glVectorFromVector(_omega);
    
    //Calculate fluid dynamics forces and torques
    glm::vec3 p = glm::vec3(TCG[3]);

    //Loop through all faces...
    const VertexPosAccessor vertexPos = mesh->getVertexPosAccessor();
    const size_t numFaces = mesh->faces.size();

    for(size_t i=0; i<numFaces; ++i)
    {
        //Global coordinates
        const Face& face = mesh->faces[i];
        const glm::vec3& p1gl = vertexPos(face, 0);
        const glm::vec3& p2gl = vertexPos(face, 1);
        const glm::vec3& p3gl = vertexPos(face, 2);
        glm::vec3 p1 = glm::vec3(TC * glm::vec4(p1gl, 1.f));
        glm::vec3 p2 = glm::vec3(TC * glm::vec4(p2gl, 1.f));
        glm::vec3 p3 = glm::vec3(TC * glm::vec4(p3gl, 1.f));
        
        //Face properties
        glm::vec3 fv1 = p2-p1; //One side of the face (triangle)
        glm::vec3 fv2 = p3-p1; //Another side of the face (triangle)
        glm::vec3 fn = glm::cross(fv1, fv2); //Normal of the face (length != 1)
        GLfloat len = glm::length2(fn);
        if(len < 1e-12f) continue;
        len = glm::sqrt(len);
        glm::vec3 fn1 = fn/len; //Normalised normal (length = 1)
        GLfloat A = len/2.f; //Area of the face (triangle)
        glm::vec3 fc = (p1+p2+p3)/3.f; //Face centroid
     
        //Forces
        glm::vec3 vc = ocn->GetFluidVelocity(fc) - (v + glm::cross(omega, fc-p));
        GLfloat vc_n = glm::dot(vc, fn1);
        glm::vec3 vn = vc_n  * fn1; //Normal velocity
        glm::vec3 vt = vc - vn; //Tangent velocity
        
        if(vc_n < -1e-12f) //If liquid is approaching the surface
        {
            //Pressure drag acts along the flow, on the face area projected onto the flow direction (|v|^2 * A * cos(theta))
            glm::vec3 quadratic = vc * -vc_n * A;
            Fdq += quadratic;
            Tdq += glm::cross(fc - p, quadratic);
        }

        GLfloat vmag2 = glm::length2(vt);
        if(vmag2 > 1e-9f)
        {
            //Skin friction of a turbulent boundary layer (|vt| * vt * A)
            glm::vec3 skin = vt * sqrtf(vmag2) * A;
            Fdf += skin;
            Tdf += glm::cross(fc - p, skin);
        }
    }

    _Fdq = Vector3(Fdq.x, Fdq.y, Fdq.z);
    _Tdq = Vector3(Tdq.x, Tdq.y, Tdq.z);
    _Fdf = Vector3(Fdf.x, Fdf.y, Fdf.z);
    _Tdf = Vector3(Tdf.x, Tdf.y, Tdf.z);
}

void SolidEntity::ComputeHydrodynamicForces(HydrodynamicsSettings settings, Ocean* ocn)
{
    if(phy_.mode != PhysicsMode::FLOATING && phy_.mode != PhysicsMode::SUBMERGED) return;
    
    auto points = submerged_.getDataAsPoints();
    if (points != nullptr)
        points->clear();

    BodyFluidPosition bf = CheckBodyFluidPosition(ocn);
    
    //If completely outside fluid just set all torques and forces to 0
    if(bf == BodyFluidPosition::OUTSIDE)
    {
        Fb_.setZero();
        Tb_.setZero();
        Fdq_.setZero();
        Tdq_.setZero();
        Fdf_.setZero();
        Tdf_.setZero();
        Swet_ = Scalar(0);
        Vsub_ = Scalar(0);
        return;
    }
    
    //Get velocities and transformations
    Vector3 v = getLinearVelocity();
    Vector3 omega = getAngularVelocity();
    
    //Check if fully submerged --> simplifies buoyancy calculation
    if(bf == BodyFluidPosition::INSIDE)
    {
        //Compute buoyancy based on CB position
        if(isBuoyant())
        {
            Fb_ = -volume_*ocn->getLiquid().density * SimulationApp::getApp()->getSimulationManager()->getGravity();
            Tb_ = (getCGTransform() * P_CB_ - getCGTransform().getOrigin()).cross(Fb_);
        }
        
        if(settings.dampingForces)
            ComputeHydrodynamicForcesSubmerged(getPhysicsMesh(), ocn, getCGTransform(), getCTransform(), v, omega, Fdq_, Tdq_, Fdf_, Tdf_);

        Swet_ = surface_;
        Vsub_ = volume_;
    }
    else //CROSSING_FLUID_SURFACE
    {
        if(!isBuoyant()) settings.reallisticBuoyancy = false;
        ComputeHydrodynamicForcesSurface(settings, getPhysicsMesh(), ocn, getCGTransform(), getCTransform(), v, omega, Fb_, Tb_, Fdq_, Tdq_, Fdf_, Tdf_, Swet_, Vsub_, submerged_);
        Scalar shellRatio = getShellBuoyancyRatio();
        Fb_ *= shellRatio;
        Tb_ *= shellRatio;
        Vsub_ *= shellRatio;
    }
    
    if(settings.dampingForces)
    {
        Scalar speed = (ocn->GetFluidVelocity(getCGTransform().getOrigin()) - v).length();
        CorrectHydrodynamicForces(ocn, Fdq_, Tdq_, Fdf_, Tdf_, fdCd_, fdCf_, getOTransform(), SkinFrictionCoefficient(ocn, speed, getCharacteristicLength()));
    }
}

void SolidEntity::ComputeAerodynamicForces(Atmosphere* atm)
{
    if(phy_.mode != PhysicsMode::AERODYNAMIC) return;
    
    //Get velocities and transformations
    Vector3 v = getLinearVelocity();
    Vector3 omega = getAngularVelocity();
    
    //Compute drag
    ComputeAerodynamicForces(getPhysicsMesh(), atm, getCGTransform(), getCTransform(), v, omega, Fda_, Tda_);
    CorrectAerodynamicForces(atm, Fda_, Tda_);
}

void SolidEntity::ComputeAerodynamicForces(const Mesh* mesh, Atmosphere* atm, const Transform& T_CG, const Transform& T_C,
                                           const Vector3& _v, const Vector3& _omega, Vector3& _Fda, Vector3& _Tda)
{
    if(mesh == nullptr) 
    {
        _Fda.setZero();
        _Tda.setZero();
        return;
    }
    
    glm::vec3 Fda(0.f);
    glm::vec3 Tda(0.f);
    glm::mat4 TCG = glMatrixFromTransform(T_CG);
    glm::mat4 TC = glMatrixFromTransform(T_C);
    glm::vec3 v = glVectorFromVector(_v);
    glm::vec3 omega = glVectorFromVector(_omega);
    
    //Calculate fluid dynamics forces and torques
    glm::vec3 p = glm::vec3(TCG[3]);

    //Loop through all faces...
    const VertexPosAccessor vertexPos = mesh->getVertexPosAccessor();
    const size_t numFaces = mesh->faces.size();

    for(size_t i=0; i<numFaces; ++i)
    {
        //Global coordinates
        const Face& face = mesh->faces[i];
        const glm::vec3& p1gl = vertexPos(face, 0);
        const glm::vec3& p2gl = vertexPos(face, 1);
        const glm::vec3& p3gl = vertexPos(face, 2);
        glm::vec3 p1 = glm::vec3(TC * glm::vec4(p1gl, 1.f));
        glm::vec3 p2 = glm::vec3(TC * glm::vec4(p2gl, 1.f));
        glm::vec3 p3 = glm::vec3(TC * glm::vec4(p3gl, 1.f));
        
        //Calculate face properties
        glm::vec3 fv1 = p2-p1; //One side of the face (triangle)
        glm::vec3 fv2 = p3-p1; //Another side of the face (triangle)
        glm::vec3 fn = glm::cross(fv1, fv2); //Normal of the face (length != 1)
        GLfloat len = glm::length2(fn);
        if(len < 1e-12f) continue;
        len = glm::sqrt(len);
        glm::vec3 fn1 = fn/len; //Normalised normal (length = 1)
        GLfloat A = len/2.f; //Area of the face (triangle)
        glm::vec3 fc = (p1+p2+p3)/3.f; //Face centroid

        //Damping forces
        glm::vec3 vc = atm->GetFluidVelocity(fc) - (v + glm::cross(omega, fc-p));
        glm::vec3 vn = glm::dot(vc, fn1) * fn1; //Normal velocity
        
        if(glm::dot(fn1, vn) < -1e-12f)
        {
            glm::vec3 quadratic = vn * glm::length(vn) * A; //glm::vec3::length() is the component count, not the magnitude!
            //Accumulate
            Fda += quadratic;
            Tda += glm::cross(fc - p, quadratic);
        }
    }

    Scalar density = atm->getGas().density;
    _Fda = Scalar(0.5) * density * Vector3(Fda.x, Fda.y, Fda.z);
    _Tda = Scalar(0.5) * density * Vector3(Tda.x, Tda.y, Tda.z);
}

void SolidEntity::CorrectAerodynamicForces(Atmosphere* atm, Vector3& _Fda, Vector3& _Tda)
{
    //Correct forces
    Vector3 Fdan = (T_CG2H_.getBasis().inverse() * _Fda).safeNormalize(); //Transform force to proxy frame
    Scalar corFactor(1.0);
    
    switch(fdApproxType_)
    {
        case  GeometryApproxType::AUTO:
        case  GeometryApproxType::SPHERE:
            break;
                
        case  GeometryApproxType::CYLINDER:
        {
            Vector3 Cd(1.1, 1.1, 0.9);
            corFactor = Cd.dot(Fdan);
        }
            break;
                
        case  GeometryApproxType::ELLIPSOID:
        {
            Vector3 Cd(Scalar(1)/fdApproxParams_[0] , Scalar(1)/fdApproxParams_[1], Scalar(1)/fdApproxParams_[2]);
            Scalar maxCd = btMax(btMax(Cd.x(), Cd.y()), Cd.z());
            Cd /= maxCd;
            corFactor = Cd.dot(Fdan);
        }
            break;
    }
    
    _Fda *= btFabs(corFactor);
    _Tda *= btFabs(corFactor);
}

void SolidEntity::ApplyHydrodynamicForces()
{
    ApplyCentralForce(Fb_ + Fdq_ + Fdf_);
    ApplyTorque(Tb_ + Tdq_ + Tdf_);
}

void SolidEntity::ApplyAddedMassCorrection(Ocean* ocn)
{
    //The physics engine supports only a scalar mass, so the body is simulated with the mean augmented mass.
    //The accumulated external force is corrected, in the body frame, so that the translational motion follows
    //the anisotropic added mass, including the added mass Coriolis force and the Munk moment
    //(T.I. Fossen, Handbook of Marine Craft Hydrodynamics and Motion Control, 2011, Sec. 6.3).
    if(phy_.mode != PhysicsMode::SUBMERGED || ocn == nullptr || volume_ <= Scalar(0))
        return;
    if(rigidBody_ == nullptr && (multibodyCollider_ == nullptr || multibodyCollider_->m_link >= 0)) //Only free bodies and bases of multibodies
        return;
    
    Scalar f = btClamped(Vsub_/volume_, Scalar(0), Scalar(1)); //Added mass depends on the submerged part of the body
    Matrix3 A = aMassT_.scaled(Vector3(f, f, f));
    Scalar Mb = getAugmentedMass();
    Matrix3 Meff = A + Matrix3(mass_, 0, 0, 0, mass_, 0, 0, 0, mass_);
    
    //Velocities in the body (CG) frame
    Transform T_CG = getCGTransform();
    Matrix3 Rt = T_CG.getBasis().transpose();
    Vector3 v = Rt * getLinearVelocity();
    Vector3 omega = Rt * getAngularVelocity();
    Vector3 vr = Rt * (getLinearVelocity() - ocn->GetFluidVelocity(T_CG.getOrigin())); //Relative to liquid
    Vector3 Avr = A * vr;
    
    //Force accumulated so far (all external forces)
    Vector3 F = Rt * getAppliedForce();
    
    //Desired: Meff dv/dt = F - omega x (m v + A vr); engine: Mb (dv/dt + omega x v) = F'
    Vector3 Fc = F - omega.cross(v * mass_ + Avr);
    Vector3 Fcorr = (Meff.inverse() * Fc + omega.cross(v)) * Mb;
    
    //Munk moment (destabilizing moment of added mass)
    Vector3 Tmunk = Avr.cross(vr);
    
    ApplyCentralForce(T_CG.getBasis() * (Fcorr - F));
    ApplyTorque(T_CG.getBasis() * Tmunk);
}

void SolidEntity::ApplyAerodynamicForces()
{
    ApplyCentralForce(Fda_);
    ApplyTorque(Tda_);
}

}
