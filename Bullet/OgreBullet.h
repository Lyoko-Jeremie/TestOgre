// This file is part of the OGRE project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at https://www.ogre3d.org/licensing.
// SPDX-License-Identifier: MIT

#ifndef COMPONENTS_BULLET_H_
#define COMPONENTS_BULLET_H_

#include <utility>

#include "btBulletDynamicsCommon.h"
#include "Ogre.h"

#include "./BulletMemoryContainer.h"

namespace Ogre {
    namespace Bullet {


        enum ColliderType {
            CT_BOX,
            CT_SPHERE,
            CT_CYLINDER,
            CT_CAPSULE,
            CT_TRIMESH,
            CT_HULL
        };

        inline btQuaternion convert(const Quaternion &q) { return btQuaternion(q.x, q.y, q.z, q.w); }

        inline btVector3 convert(const Vector3 &v) { return btVector3(v.x, v.y, v.z); }

        inline Quaternion convert(const btQuaternion &q) { return Quaternion(q.w(), q.x(), q.y(), q.z()); }

        inline Vector3 convert(const btVector3 &v) { return Vector3(v.x(), v.y(), v.z()); }

        /** A MotionState is Bullet's way of informing you about updates to an object.
         * Pass this MotionState to a btRigidBody to have your SceneNode updated automaticaly.
         */
        class RigidBodyState : public btMotionState, public boost::enable_shared_from_this<RigidBodyState> {
            Node *mNode;

        public:
            explicit RigidBodyState(Node *node) : mNode(node) {}

            void getWorldTransform(btTransform &ret) const override {
                ret = btTransform(convert(mNode->getOrientation()), convert(mNode->getPosition()));
            }

            void setWorldTransform(const btTransform &in) override {
                btQuaternion rot = in.getRotation();
                btVector3 pos = in.getOrigin();
                mNode->setOrientation(rot.w(), rot.x(), rot.y(), rot.z());
                mNode->setPosition(pos.x(), pos.y(), pos.z());
            }
        };

        struct CollisionListener {
            virtual ~CollisionListener() = default;

            virtual void contact(const MovableObject *other, const btManifoldPoint &manifoldPoint) = 0;
        };

        struct RayResultCallback {
            virtual ~RayResultCallback() = default;

            virtual void addSingleResult(const MovableObject *other, float distance) = 0;
        };

//        static void onTick(btDynamicsWorld *world, btScalar timeStep);


        class DebugDrawer : public btIDebugDraw {
            SceneNode *mNode;

            ManualObject mLines;
            int mDebugMode;

        public:
            explicit DebugDrawer(SceneNode *node)
                    : mNode(node),
                      mLines(""),
                      mDebugMode(DBG_DrawWireframe) {
                mLines.setCastShadows(false);
                mNode->attachObject(&mLines);
            }

            void updateDebugDrawWorld(btDynamicsWorld *mWorld) {
                mWorld->debugDrawWorld();
                if (!mLines.getSections().empty()) // `mLines.begin` was called
                    mLines.end();
            }

            void drawLine(const btVector3 &from, const btVector3 &to, const btVector3 &color) override;

            void drawContactPoint(const btVector3 &PointOnB, const btVector3 &normalOnB,
                                  btScalar distance, int lifeTime,
                                  const btVector3 &color) override {
                drawLine(PointOnB, PointOnB + normalOnB * distance * 20, color);
            }

            void reportErrorWarning(const char *warningString) override {
                // TODO
                LogManager::getSingleton().logWarning(warningString);
            }

            void draw3dText(const btVector3 &location, const char *textString) override {}

            void setDebugMode(int mode) override {
                mDebugMode = mode;

                if (mDebugMode == DBG_NoDebug)
                    clear();
            }

            void clear() { mLines.clear(); }

            int getDebugMode() const override { return mDebugMode; }
        };


        /// simplified wrapper with automatic memory management
        class DynamicsWorld {

            boost::shared_ptr<BulletMemoryContainer::BulletMemoryContainerManager> memoryContainerManager_;

            MemoryPool::unique_ptr_with_alloc_deleter<btDefaultCollisionConfiguration> mCollisionConfig;
            MemoryPool::unique_ptr_with_alloc_deleter<btCollisionDispatcher> mDispatcher;
            MemoryPool::unique_ptr_with_alloc_deleter<btDbvtBroadphase> mBroadphase;
            MemoryPool::unique_ptr_with_alloc_deleter<btSequentialImpulseConstraintSolver> mSolver;
            MemoryPool::unique_ptr_with_alloc_deleter<btDiscreteDynamicsWorld> mBtWorld;

        public:
            explicit DynamicsWorld(
                    boost::shared_ptr<BulletMemoryContainer::BulletMemoryContainerManager> memoryContainerManager,
                    const Vector3 &gravity
            ) : memoryContainerManager_(std::move(memoryContainerManager)),
                mCollisionConfig(memoryContainerManager_->makeUniquePtr<btDefaultCollisionConfiguration>()),
                mDispatcher(memoryContainerManager_->makeUniquePtr<btCollisionDispatcher>(
                        &*mCollisionConfig
                )),
                mBroadphase(memoryContainerManager_->makeUniquePtr<btDbvtBroadphase>()),
                mSolver(memoryContainerManager_->makeUniquePtr<btSequentialImpulseConstraintSolver>()),
                mBtWorld(memoryContainerManager_->makeUniquePtr<btDiscreteDynamicsWorld>(
                        &*mDispatcher,
                        &*mBroadphase,
                        &*mSolver,
                        &*mCollisionConfig
                )),
                mDebugDrawer(nullptr) {


                mBtWorld->setGravity(convert(gravity));
//                mBtWorld->setInternalTickCallback(onTick);
            }

            ~DynamicsWorld() = default;

            boost::shared_ptr<BulletMemoryContainer::BulletMemoryContainerManager::RigidObjectType>
            addRigidBody(float mass,
                         Entity *ent,
                         ColliderType ct,
                         CollisionListener *listener = nullptr,
                         int group = 1, int mask = -1);

            const decltype(mBtWorld) &getBtWorld() const { return mBtWorld; }

            void rayTest(const Ray &ray, RayResultCallback *callback, float maxDist = 1000);


            /// create sphere collider using ogre provided data
            boost::shared_ptr<btSphereShape> createSphereCollider(const MovableObject *mo);

            /// create box collider using ogre provided data
            boost::shared_ptr<btBoxShape> createBoxCollider(const MovableObject *mo);

            /// create capsule collider using ogre provided data
            boost::shared_ptr<btCapsuleShape> createCapsuleCollider(const MovableObject *mo);

            /// create capsule collider using ogre provided data
            boost::shared_ptr<btCylinderShape> createCylinderCollider(const MovableObject *mo);

            void stepSimulation(float timeStep, int maxSubSteps = 1,
                                float fixedTimeStep = btScalar(1.) / btScalar(60.)) {
                ++simulationStepNow;
                mBtWorld->stepSimulation(timeStep, maxSubSteps, fixedTimeStep);
                this->onTick();
            }

        private:

            boost::shared_ptr<DebugDrawer> mDebugDrawer;

            size_t simulationStepNow = 0;

        public:

            void initDebugDrawer(SceneNode *node) {
                if (node == nullptr) {
                    mBtWorld->setDebugDrawer(nullptr);
                    return;
                }
                if (mDebugDrawer) {
                    mBtWorld->setDebugDrawer(nullptr);
                }
                mDebugDrawer = memoryContainerManager_->makeSharedPtr<DebugDrawer>(node);
                mBtWorld->setDebugDrawer(mDebugDrawer.get());
            }

            int setDebugMode(int mode) {
                if (mDebugDrawer) {
                    mDebugDrawer->setDebugMode(mode);
                }
                return getDebugMode();
            }

            int getDebugMode() const { return mDebugDrawer && mDebugDrawer->getDebugMode(); }

            void updateDebugDrawWorld() {
                mDebugDrawer->updateDebugDrawWorld(&*mBtWorld);
            }


        private:

            void onTick() {
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
                                            .modify(it, [nowStep](
                                                    BulletMemoryContainer::BulletMemoryContainerManager::CollisionStateContainerItemType &a) {
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

//                // TODO trigger event
//                for (const auto &a: ccs) {
//                    if (a->lastCheckTime == nowStep) {
//                        // a->state;
//                        if (a->state == BulletMemoryContainer::CollisionState::State::start) {
//                            BOOST_LOG_TRIVIAL(trace) << a->idA << "-" << a->idB << " start";
//                        } else if (a->state == BulletMemoryContainer::CollisionState::State::collision) {
//                            BOOST_LOG_TRIVIAL(trace) << a->idA << "-" << a->idB << " collision";
//                        }
//                    } else {
//                        // end
//                        BOOST_LOG_TRIVIAL(trace) << a->idA << "-" << a->idB << " end";
//                    }
//                }


                // remove event ended
                ccs.remove_if([nowStep](
                        const BulletMemoryContainer::BulletMemoryContainerManager::CollisionStateContainerItemType &a) {
                    return a->lastCheckTime != nowStep;
                });

            }

        };

    } // namespace Bullet
} // namespace Ogre

#endif
