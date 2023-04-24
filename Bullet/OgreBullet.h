// This file is part of the OGRE project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at https://www.ogre3d.org/licensing.
// SPDX-License-Identifier: MIT

#ifndef COMPONENTS_BULLET_H_
#define COMPONENTS_BULLET_H_

#include <utility>

#include <boost/assert.hpp>

#include "btBulletDynamicsCommon.h"
#include <Ogre.h>
#include <OgreApplicationContext.h>

#include "./BulletMemoryContainer.h"

namespace Ogre {
    namespace Bullet {


        enum class ColliderType {
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

            // Bullet will call this function to update Ogre object info
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

            void onTick();

        public:

            struct Bullet2OgreTracer : public boost::enable_shared_from_this<Bullet2OgreTracer> {

                // ------ memory host information ------
                boost::weak_ptr<OgreBites::ApplicationContext> pCtx;
                Ogre::Root *root;
                Ogre::SceneManager *scnMgr;
                std::string nameSceneManager;
                // ------ memory host information ------

                Ogre::SceneNode *sceneNode;
                std::string sceneNodeName;
                Entity *entity;

                Bullet2OgreTracer(
                        const boost::shared_ptr<OgreBites::ApplicationContext> &ctx,
                        Ogre::Root *root,
                        Ogre::SceneManager *scnMgr,
                        Entity *ent
                ) : pCtx(ctx), root(root), scnMgr(scnMgr), entity(ent) {
                    BOOST_ASSERT_MSG(
                            !pCtx.expired() && root && scnMgr && sceneNode && entity,
                            "Bullet2OgreTracer invalid Ogre info"
                    );
                    sceneNode = ent->getParentSceneNode();
                    sceneNodeName = sceneNode->getName();
                }


                /** check the scene still valid and the node still in the scene
                 * this is a slow operator, call it carefully
                 */
                bool checkValid() const {

                    // check context top-down
                    if (pCtx.expired() || !root || !scnMgr || !sceneNode || !entity) {
                        return false;
                    }
                    auto ctx = pCtx.lock();
                    if (!ctx) {
                        return false;
                    }
                    if (root != ctx->getRoot()) {
                        return false;
                    }
                    auto scnMgrPtr = root->getSceneManager(nameSceneManager);
                    if (scnMgrPtr != scnMgr) {
                        return false;
                    }

                    const String &nameToFind = sceneNodeName;
                    SceneNode *nodeToFind = sceneNode;
                    SceneNode *nodeRootToFindStart = scnMgrPtr->getRootSceneNode();

                    // https://forums.ogre3d.org/viewtopic.php?t=20799

                    BOOST_ASSERT_MSG(nodeRootToFindStart, "nodeRootToFindStart must not nullptr");
                    BOOST_ASSERT_MSG(
                            nodeToFind || !nameToFind.empty(),
                            "nodeToFind||nameToFind must valid at lest one"
                    );

                    // if node have name, direct find use name
                    auto p = scnMgrPtr->getSceneNode(nameToFind, false);
                    if (!nameToFind.empty()) {
                        if (p) {
                            return true;
                        }
                    }

                    BOOST_ASSERT_MSG(nodeToFind, "nodeToFind must not nullptr when nameToFind.empty()");

                    // a deep first search use stack(std::ref)
                    //    std::deque(std::pair( ChildNodeMap& , ChildNodeMap_Index ))
                    std::deque<std::pair<std::reference_wrapper<const SceneNode::ChildNodeMap>, int>> stack;

                    stack.emplace_back(nodeRootToFindStart->getChildren(), 0);

                    while (!stack.empty()) {
                        auto N = stack.rbegin();
                        if (N == stack.rend()) {
                            // empty, still not find
                            return false;
                        }
                        if (N->second >= N->first.get().size()) {
                            // this node look end, now to look next node
                            stack.pop_back();
                            continue;
                        }
                        Node *n = N->first.get().at(N->second);
                        if (n == nodeToFind) {
                            // get it
                            return true;
                        }
                        // next
                        ++N->second;
                        if (!n) {
                            // invalid ptr
                            continue;
                        }
                        // save child list, now try to look the list
                        stack.emplace_back(n->getChildren(), 0);
                    }

                    return false;
                }

            };

        };

    } // namespace Bullet
} // namespace Ogre

#endif
