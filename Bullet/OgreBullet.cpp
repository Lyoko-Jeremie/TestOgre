// This file is part of the OGRE project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at https://www.ogre3d.org/licensing.
// SPDX-License-Identifier: MIT

#include "OgreBullet.h"

#include <memory>

#include "./VertexIndexToShape.h"

namespace Ogre::Bullet {

    boost::shared_ptr<btSphereShape> DynamicsWorld::createSphereCollider(const MovableObject *mo) {
        OgreAssert(mo->getParentSceneNode(), "MovableObject must be attached");

        auto shape = memoryContainerManager_->makeSharedPtr<btSphereShape>(mo->getBoundingRadius());
        shape->setLocalScaling(convert(mo->getParentSceneNode()->getScale()));

        return shape;
    }

    boost::shared_ptr<btBoxShape> DynamicsWorld::createBoxCollider(const MovableObject *mo) {
        OgreAssert(mo->getParentSceneNode(), "MovableObject must be attached");

        auto shape = memoryContainerManager_->makeSharedPtr<btBoxShape>(convert(mo->getBoundingBox().getHalfSize()));
        shape->setLocalScaling(convert(mo->getParentSceneNode()->getScale()));

        return shape;
    }

    boost::shared_ptr<btCapsuleShape> DynamicsWorld::createCapsuleCollider(const MovableObject *mo) {
        OgreAssert(mo->getParentSceneNode(), "MovableObject must be attached");

        auto sz = mo->getBoundingBox().getHalfSize();

        btScalar height = std::max(sz.x, std::max(sz.y, sz.z));
        btScalar radius;
        boost::shared_ptr<btCapsuleShape> shape;
        // Orient the capsule such that its height is aligned with the largest dimension.
        if (height == sz.y) {
            radius = std::max(sz.x, sz.z);
            shape = memoryContainerManager_->makeSharedPtr<btCapsuleShape>(radius, 2 * height - 2 * radius);
        } else if (height == sz.x) {
            radius = std::max(sz.y, sz.z);
            shape = memoryContainerManager_->makeSharedPtr<btCapsuleShapeX>(radius, 2 * height - 2 * radius);
        } else {
            radius = std::max(sz.x, sz.y);
            shape = memoryContainerManager_->makeSharedPtr<btCapsuleShapeZ>(radius, 2 * height - 2 * radius);
        }

        shape->setLocalScaling(convert(mo->getParentSceneNode()->getScale()));

        return shape;
    }

    /// create capsule collider using ogre provided data
    boost::shared_ptr<btCylinderShape> DynamicsWorld::createCylinderCollider(const MovableObject *mo) {
        OgreAssert(mo->getParentSceneNode(), "MovableObject must be attached");

        auto sz = convert(mo->getBoundingBox().getHalfSize());

        btScalar height = std::max(sz.x(), std::max(sz.y(), sz.z()));
        boost::shared_ptr<btCylinderShape> shape;
        // Orient the capsule such that its height is aligned with the largest dimension.
        if (height == sz.y()) {
            shape = memoryContainerManager_->makeSharedPtr<btCylinderShape>(sz);
        } else if (height == sz.x()) {
            shape = memoryContainerManager_->makeSharedPtr<btCylinderShapeX>(sz);
        } else {
            shape = memoryContainerManager_->makeSharedPtr<btCylinderShapeZ>(sz);
        }

        shape->setLocalScaling(convert(mo->getParentSceneNode()->getScale()));

        return shape;
    }

    boost::shared_ptr<BulletMemoryContainer::BulletMemoryContainerManager::RigidObjectType>
    DynamicsWorld::addRigidBody(float mass,
                                Entity *ent,
                                ColliderType ct,
                                const boost::shared_ptr<DynamicsWorld::Bullet2OgreTracer> &bullet2OgreTracer,
                                int group, int mask) {
        auto node = ent->getParentSceneNode();
        OgreAssert(node, "entity must be attached to a SceneNode");
        auto state = memoryContainerManager_->makeSharedPtr<RigidBodyState>(
                node
        );
        OgreAssert(bullet2OgreTracer->entity == ent, "entity must same");

        if (ent->hasSkeleton()) {
            ent->addSoftwareAnimationRequest(false);
            ent->_updateAnimation();
            ent->setUpdateBoundingBoxFromSkeleton(true);
        }

        boost::shared_ptr<btCollisionShape> cs = nullptr;
        switch (ct) {
            case ColliderType::CT_BOX:
                cs = createBoxCollider(ent);
                break;
            case ColliderType::CT_SPHERE:
                cs = createSphereCollider(ent);
                break;
            case ColliderType::CT_CYLINDER:
                cs = createCylinderCollider(ent);
                break;
            case ColliderType::CT_CAPSULE:
                cs = createCapsuleCollider(ent);
                break;
            case ColliderType::CT_TRIMESH:
                cs = VertexIndexToShape(ent).createTrimesh(memoryContainerManager_);
                break;
            case ColliderType::CT_HULL:
                cs = VertexIndexToShape(ent).createConvex(memoryContainerManager_);
                break;
        }

        if (ent->hasSkeleton())
            ent->removeSoftwareAnimationRequest(false);

        btVector3 inertia(0, 0, 0);
        if (mass != 0) // mass = 0 -> static
            cs->calculateLocalInertia(mass, inertia);

        memoryContainerManager_->makeShape(cs);

        auto rigidBody = memoryContainerManager_->makeBody(
                memoryContainerManager_->makeRigidBodyPtr(
                        mass,
                        dynamic_cast<btMotionState *>(state.get()),
                        dynamic_cast<btCollisionShape *>(cs.get()),
                        inertia
                ),
                state
        );
        mBtWorld->addRigidBody(rigidBody->ptrRigidBody.get(), group, mask);
        rigidBody->userPtr = bullet2OgreTracer;

        if (bullet2OgreTracer && !bullet2OgreTracer->sceneNodeName.empty()) {
            rigidBody->name = bullet2OgreTracer->sceneNodeName;
        } else if (!node->getName().empty()) {
            rigidBody->name = !node->getName().empty();
        } else if (ent->getMesh() && !ent->getMesh()->getName().empty()) {
            rigidBody->name = !ent->getMesh()->getName().empty();
        } else {
            // empty
            //      rigidBody->name;
            //      rigidBody->uuid;
        }

        return rigidBody;
    }

    void DynamicsWorld::rayTest(const Ray &ray,
                                const boost::shared_ptr<btCollisionWorld::RayResultCallback> &callback,
                                float maxDist) {
        btVector3 from = convert(ray.getOrigin());
        btVector3 to = convert(ray.getPoint(maxDist));
        mBtWorld->rayTest(from, to, *callback);
    }

    void DynamicsWorld::onTick() {
        auto nowStep = simulationStepNow;
        auto &ccs = memoryContainerManager_->getCollisionStateContainer();

        // https://github.com/enable3d/enable3d/blob/faedc9b90fa01f676b8bcddbbf1a8274512e691c/packages/ammoPhysics/src/physics.ts
        for (int i = 0; i < mDispatcher->getNumManifolds(); ++i) {
            auto contactManifold = mDispatcher->getManifoldByIndexInternal(i);

            auto b0 = contactManifold->getBody0();
            auto b1 = contactManifold->getBody1();
//            auto rb0 = btRigidBody::upcast(b0);
//            auto rb1 = btRigidBody::upcast(b1);

            int idA = b0->getUserIndex();
            int idB = b1->getUserIndex();

            if (idA < 1 && idB < 1) {
                // invalid object
                continue;
            }

            // makes sure idA-idB pair
            if (idA > idB)
                std::exchange(idA, idB);

            for (int j = 0; j < contactManifold->getNumContacts(); ++j) {
                auto contactPoint = contactManifold->getContactPoint(j);
                auto distance = contactPoint.getDistance();


                // Distance definition: when the distance between objects is positive, they are separated. When the distance is negative, they are penetrating. Zero distance means exactly touching.
                // https://pybullet.org/Bullet/phpBB3/viewtopic.php?t=5831
                if (distance <= 0) {

                    auto it = ccs.get<BulletMemoryContainer::CollisionState::ID>()
                            .find(std::make_tuple(idA, idB));
                    if (it != ccs.get<BulletMemoryContainer::CollisionState::ID>().end()) {
                        if ((*it)->lastCheckTime != nowStep) {
                            // collision
                            ccs.get<BulletMemoryContainer::CollisionState::ID>()
                                    .modify(it, [nowStep](BulletMemoryContainer::CollisionStateContainerItemType &a) {
                                        a->lastCheckTime = nowStep;
                                        a->state = BulletMemoryContainer::CollisionState::State::collision;
                                    });
                        } else {
                            // don't touch it
                        }
                    } else {
                        // start
                        memoryContainerManager_->makeCollisionState(
                                idA, idB, BulletMemoryContainer::CollisionState::State::start, nowStep
                        );
                    }

                    break;
                }
            }
        }

        // trigger event
        auto dynamicsWorldPtr = shared_from_this();
        for (const auto &a: ccs) {
            auto pairs = memoryContainerManager_->getBody2(
                    a->idA, a->idB
            );
            if (pairs.first->userPtr && pairs.first->userPtr->typeName == Bullet2OgreTracer::TypeNameTag) {
                auto dp = boost::dynamic_pointer_cast<Bullet2OgreTracer>(pairs.first->userPtr);
                if (!dp) {
                    // TODO never go there in logic
                } else {
                    dp->collisionTrigger(
                            a->state,
                            a->idA,
                            a->idB,
                            pairs.first,
                            pairs.second,
                            a,
                            dynamicsWorldPtr
                    );
                }
            }
            if (pairs.second->userPtr && pairs.second->userPtr->typeName == Bullet2OgreTracer::TypeNameTag) {
                auto dp = boost::dynamic_pointer_cast<Bullet2OgreTracer>(pairs.second->userPtr);
                if (!dp) {
                    // TODO never go there in logic
                } else {
                    dp->collisionTrigger(
                            a->state,
                            a->idA,
                            a->idB,
                            pairs.second,
                            pairs.first,
                            a,
                            dynamicsWorldPtr
                    );
                }
            }
//            if (a->lastCheckTime == nowStep) {
//                // a->state;
//                if (a->state == BulletMemoryContainer::CollisionState::State::start) {
////                    BOOST_LOG_TRIVIAL(trace) << a->idA << "-" << a->idB << " start";
//                } else if (a->state == BulletMemoryContainer::CollisionState::State::collision) {
////                    BOOST_LOG_TRIVIAL(trace) << a->idA << "-" << a->idB << " collision";
//                }
//            } else {
//                // end
////                BOOST_LOG_TRIVIAL(trace) << a->idA << "-" << a->idB << " end";
//            }
        }


        // remove event ended
        ccs.remove_if([nowStep](const BulletMemoryContainer::CollisionStateContainerItemType &a) {
            return a->lastCheckTime != nowStep;
        });

    }


/*
 * =============================================================================================
 * BtDebugDrawer
 * =============================================================================================
 */
//------------------------------------------------------------------------------------------------
    void DebugDrawer::drawLine(const btVector3 &from, const btVector3 &to, const btVector3 &color) {
        if (mLines.getSections().empty()) {
            const char *matName = "Ogre/Debug/LinesMat";
            auto mat = MaterialManager::getSingleton().getByName(matName, RGN_INTERNAL);
            if (!mat) {
                mat = MaterialManager::getSingleton().create(matName, RGN_INTERNAL);
                auto p = mat->getTechnique(0)->getPass(0);
                p->setLightingEnabled(false);
                p->setVertexColourTracking(TVC_AMBIENT);
            }
            mLines.setBufferUsage(HBU_CPU_TO_GPU);
            mLines.begin(mat, RenderOperation::OT_LINE_LIST);
        } else if (mLines.getCurrentVertexCount() == 0)
            mLines.beginUpdate(0);

        ColourValue col(color.x(), color.x(), color.z());
        mLines.position(convert(from));
        mLines.colour(col);
        mLines.position(convert(to));
        mLines.colour(col);
    }
} // namespace Ogre
