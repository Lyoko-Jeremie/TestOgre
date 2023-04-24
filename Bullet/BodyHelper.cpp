// jeremie

#include "BodyHelper.h"


namespace Ogre::Bullet::BodyHelper {

    boost::shared_ptr<BulletMemoryContainer::BulletMemoryContainerManager::RigidObjectType>
    createInfiniteGround(boost::shared_ptr<Ogre::Bullet::DynamicsWorld> dynamicsWorld,
                         btVector3 origin, Ogre::Entity *ent,
                         const boost::shared_ptr<DynamicsWorld::Bullet2OgreTracer> &bullet2OgreTracer) {
        const auto &memoryContainerManager_ = dynamicsWorld->getMemoryContainerManager();

        // https://gitlab.com/phantasyisland/PhantasyIsland/-/blob/jeremie/PhantasyEngine/src/PhantasyScene/initInfiniteGround.ts
        // http://aicdg.com/oldblog/c++/2017/02/20/glitter-bullet.html


        auto infiniteGround = memoryContainerManager_->makeShape(
                memoryContainerManager_->makeSharedPtr<btStaticPlaneShape>(
                        btVector3{0, 1, 0}, 0.0f
                )
        );

        btTransform t{};
        t.setIdentity();
        t.setOrigin(origin);

        boost::shared_ptr<btMotionState> state;
        if (ent) {
            auto node = ent->getParentSceneNode();
            OgreAssert(node, "entity must be attached to a SceneNode");
            state = memoryContainerManager_->makeSharedPtr<RigidBodyState>(
                    node
            );
        } else {
            state = memoryContainerManager_->makeSharedPtr<btDefaultMotionState>(t);
        }

        btRigidBody::btRigidBodyConstructionInfo rbInfo(
                0,
                &*state,
                &*infiniteGround->ptr
        );
        auto rigidBody = memoryContainerManager_->makeBody(
                memoryContainerManager_->makeRigidBodyPtr(rbInfo),
                state
        );

        dynamicsWorld->getBtWorld()->addRigidBody(&*rigidBody->ptrRigidBody);

        if (bullet2OgreTracer) {
            rigidBody->userPtr = bullet2OgreTracer;
        }

        rigidBody->name = "infiniteGround";
        // rigidBody->uuid;

        return rigidBody;
    }


}

