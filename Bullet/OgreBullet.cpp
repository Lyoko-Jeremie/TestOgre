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
                                Entity *entity,
                                const boost::shared_ptr<btCollisionShape> &collisionShape,
                                const boost::shared_ptr<DynamicsWorld::Bullet2OgreTracer> &bullet2OgreTracer,
                                int group, int mask) {
        OgreAssert(entity, "entity must valid");
        auto node = entity->getParentSceneNode();
        OgreAssert(node, "entity must be attached to a SceneNode");
        if (bullet2OgreTracer) {
            OgreAssert(bullet2OgreTracer->entity == entity, "entity must same");
        }
        auto state = memoryContainerManager_->makeSharedPtr<RigidBodyState>(shared_from_this(), node);


        btVector3 inertia(0, 0, 0);
        if (mass != 0) // mass = 0 -> static
            collisionShape->calculateLocalInertia(mass, inertia);

        auto shape = memoryContainerManager_->makeShape(collisionShape);

        auto rigidBody = memoryContainerManager_->makeBody(
                memoryContainerManager_->makeRigidBodyPtr(
                        mass,
                        dynamic_cast<btMotionState *>(state.get()),
                        dynamic_cast<btCollisionShape *>(shape->ptr.get()),
                        inertia
                ),
                state
        );
        mBtWorld->addRigidBody(rigidBody->ptrRigidBody.get(), group, mask);
        rigidBody->userPtr = bullet2OgreTracer;

        if (bullet2OgreTracer && !bullet2OgreTracer->sceneNodeName.empty()) {
            rigidBody->name = bullet2OgreTracer->sceneNodeName;
        } else if (!node->getName().empty()) {
            rigidBody->name = node->getName();
        } else if (entity->getMesh() && !entity->getMesh()->getName().empty()) {
            rigidBody->name = entity->getMesh()->getName();
        } else {
            // empty
            //      rigidBody->name;
            //      rigidBody->uuid;
        }

        entity->getUserObjectBindings().setUserAny("id_bullet", rigidBody->id);
        entity->getUserObjectBindings().setUserAny("uuid_bullet", rigidBody->uuid);
        if (!node->getUserObjectBindings().getUserAny("id_bullet").has_value()) {
            node->getUserObjectBindings().setUserAny("id_bullet", rigidBody->id);
            node->getUserObjectBindings().setUserAny("uuid_bullet", rigidBody->uuid);
        }

        state->bodyId = rigidBody->id;
        state->bodyUUID = rigidBody->uuid;
        state->bodyName = rigidBody->name;
        shape->uuid = rigidBody->uuid;
        shape->name = rigidBody->name;

        return rigidBody;

    }

    boost::shared_ptr<BulletMemoryContainer::BulletMemoryContainerManager::RigidObjectType>
    DynamicsWorld::addRigidBody(float mass,
                                Entity *entity,
                                ColliderType colliderType,
                                const boost::shared_ptr<DynamicsWorld::Bullet2OgreTracer> &bullet2OgreTracer,
                                int group, int mask) {
        OgreAssert(entity, "entity must valid");
        auto node = entity->getParentSceneNode();
        OgreAssert(node, "entity must be attached to a SceneNode");
        if (bullet2OgreTracer) {
            OgreAssert(bullet2OgreTracer->entity == entity, "entity must same");
        }

        if (entity->hasSkeleton()) {
            entity->addSoftwareAnimationRequest(false);
            entity->_updateAnimation();
            entity->setUpdateBoundingBoxFromSkeleton(true);
        }

        boost::shared_ptr<btCollisionShape> collisionShape = nullptr;
        switch (colliderType) {
            case ColliderType::CT_BOX:
                collisionShape = createBoxCollider(entity);
                break;
            case ColliderType::CT_SPHERE:
                collisionShape = createSphereCollider(entity);
                break;
            case ColliderType::CT_CYLINDER:
                collisionShape = createCylinderCollider(entity);
                break;
            case ColliderType::CT_CAPSULE:
                collisionShape = createCapsuleCollider(entity);
                break;
            case ColliderType::CT_TRIMESH:
                collisionShape = VertexIndexToShape(entity).createTrimesh(memoryContainerManager_);
                break;
            case ColliderType::CT_HULL:
                collisionShape = VertexIndexToShape(entity).createConvex(memoryContainerManager_);
                break;
        }

        if (entity->hasSkeleton())
            entity->removeSoftwareAnimationRequest(false);

        return addRigidBody(
                mass,
                entity,
                collisionShape,
                bullet2OgreTracer,
                group,
                mask
        );
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

    void RigidBodyState::setDirty() const {
        if (bodyId > 0) {
            if (auto p = dynamicsWorldPtr.lock()) {
                auto r = *atomic_load(&lastTransformPtr);
                boost::lock_guard lg{p->mtxDirtyBody};
                p->dirtyBody.insert_or_assign(bodyId,r);
            }
        }
    }
} // namespace Ogre
