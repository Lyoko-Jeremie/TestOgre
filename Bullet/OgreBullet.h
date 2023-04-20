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

        /** \addtogroup Optional
        *  @{
        */
        /** \defgroup Bullet Bullet
         * [Bullet-Physics](https://pybullet.org) to %Ogre connection
         * @{
         */
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
        class RigidBodyState : public btMotionState, boost::enable_shared_from_this<RigidBodyState> {
            Node *mNode;

        public:
            RigidBodyState(Node *node) : mNode(node) {}

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

        /// create sphere collider using ogre provided data
        btSphereShape *createSphereCollider(const MovableObject *mo);

        /// create box collider using ogre provided data
        btBoxShape *createBoxCollider(const MovableObject *mo);

        /// create capsule collider using ogre provided data
        btCapsuleShape *createCapsuleCollider(const MovableObject *mo);

        /// create capsule collider using ogre provided data
        btCylinderShape *createCylinderCollider(const MovableObject *mo);

        struct CollisionListener {
            virtual ~CollisionListener() {}

            virtual void contact(const MovableObject *other, const btManifoldPoint &manifoldPoint) = 0;
        };

        struct RayResultCallback {
            virtual ~RayResultCallback() {}

            virtual void addSingleResult(const MovableObject *other, float distance) = 0;
        };

        static void onTick(btDynamicsWorld *world, btScalar timeStep);

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
                    const Vector3 &gravity)
                    : memoryContainerManager_(std::move(memoryContainerManager)),
                      mCollisionConfig(boost::allocate_unique<btDefaultCollisionConfiguration>(
                              MemoryPool::MemoryCustomAllocator<btDefaultCollisionConfiguration>(
                                      memoryContainerManager_->getMemoryPoolManager()))),
                      mDispatcher(boost::allocate_unique<btCollisionDispatcher>(
                              MemoryPool::MemoryCustomAllocator<btCollisionDispatcher>(
                                      memoryContainerManager_->getMemoryPoolManager()),
                              &*mCollisionConfig
                      )),
                      mBroadphase(boost::allocate_unique<btDbvtBroadphase>(
                              MemoryPool::MemoryCustomAllocator<btDbvtBroadphase>(
                                      memoryContainerManager_->getMemoryPoolManager())
                      )),
                      mSolver(boost::allocate_unique<btSequentialImpulseConstraintSolver>(
                              MemoryPool::MemoryCustomAllocator<btSequentialImpulseConstraintSolver>(
                                      memoryContainerManager_->getMemoryPoolManager())
                      )),
                      mBtWorld(boost::allocate_unique<btDiscreteDynamicsWorld>(
                              MemoryPool::MemoryCustomAllocator<btDiscreteDynamicsWorld>(
                                      memoryContainerManager_->getMemoryPoolManager()),
                              &*mDispatcher,
                              &*mBroadphase,
                              &*mSolver,
                              &*mCollisionConfig
                      )) {
                mBtWorld->setGravity(convert(gravity));
                mBtWorld->setInternalTickCallback(onTick);
            }

            ~DynamicsWorld() = default;

            BulletMemoryContainer::BulletMemoryContainerManager::RigidObjectType::PtrRigidBody
            addRigidBody(float mass, Entity *ent, ColliderType ct,
                         CollisionListener *listener = nullptr,
                         int group = 1, int mask = -1);

            const decltype(mBtWorld) &getBtWorld() const { return mBtWorld; }

            void rayTest(const Ray &ray, RayResultCallback *callback, float maxDist = 1000);
        };

        class DebugDrawer : public btIDebugDraw {
            SceneNode *mNode;
            btDynamicsWorld *mWorld;

            ManualObject mLines;
            int mDebugMode;

        public:
            DebugDrawer(SceneNode *node, btDynamicsWorld *world)
                    : mNode(node), mWorld(world), mLines(""), mDebugMode(DBG_DrawWireframe) {
                mLines.setCastShadows(false);
                mNode->attachObject(&mLines);
                mWorld->setDebugDrawer(this);
            }

            void update() {
                mWorld->debugDrawWorld();
                if (!mLines.getSections().empty()) // begin was called
                    mLines.end();
            }

            void drawLine(const btVector3 &from, const btVector3 &to, const btVector3 &color) override;

            void
            drawContactPoint(const btVector3 &PointOnB, const btVector3 &normalOnB, btScalar distance, int lifeTime,
                             const btVector3 &color) override {
                drawLine(PointOnB, PointOnB + normalOnB * distance * 20, color);
            }

            void reportErrorWarning(const char *warningString) override {
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
/** @} */
/** @} */
    } // namespace Bullet
} // namespace Ogre

#endif
