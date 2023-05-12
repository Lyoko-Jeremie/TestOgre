// jeremie

#include "BodyHelper.h"

#include <utility>


namespace Ogre::Bullet::BodyHelper {

    boost::shared_ptr<BulletMemoryContainer::BulletMemoryContainerManager::RigidObjectType>
    createInfiniteGround(boost::shared_ptr<Ogre::Bullet::DynamicsWorld> dynamicsWorld,
                         std::string name,
                         btVector3 origin, Ogre::Entity *entity,
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

//        boost::shared_ptr<btMotionState> state;
//        if (entity) {
//            auto node = entity->getParentSceneNode();
//            OgreAssert(node, "entity must be attached to a SceneNode");
//            state = memoryContainerManager_->makeSharedPtr<RigidBodyState>(dynamicsWorld);
//        } else {
//            state = memoryContainerManager_->makeSharedPtr<btDefaultMotionState>(t);
//        }
        auto state = memoryContainerManager_->makeSharedPtr<RigidBodyState>(dynamicsWorld);
        if (entity) {
            auto node = entity->getParentSceneNode();
            if (node) {
                state->setWithNode(node);
            }
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
        rigidBody->name = std::move(name);

        dynamicsWorld->getBtWorld()->addRigidBody(&*rigidBody->ptrRigidBody);

        if (bullet2OgreTracer) {
            rigidBody->userPtr = bullet2OgreTracer;
        }

        if (entity) {
            auto node = entity->getParentSceneNode();
//            if (node && !node->getName().empty()) {
//                rigidBody->name = node->getName();
//            } else if (entity->getMesh() && !entity->getMesh()->getName().empty()) {
//                rigidBody->name = entity->getMesh()->getName();
//            } else {
//                // empty
//                //      rigidBody->name;
//                //      rigidBody->uuid;
//            }
            entity->getUserObjectBindings().setUserAny("id_bullet", rigidBody->id);
            entity->getUserObjectBindings().setUserAny("uuid_bullet", rigidBody->uuid);
            if (node) {
                if (!node->getUserObjectBindings().getUserAny("id_bullet").has_value()) {
                    node->getUserObjectBindings().setUserAny("id_bullet", rigidBody->id);
                    node->getUserObjectBindings().setUserAny("uuid_bullet", rigidBody->uuid);
                }
            }
        }

        state->bodyId = rigidBody->id;
        state->bodyUUID = rigidBody->uuid;
        state->bodyName = rigidBody->name;
        infiniteGround->uuid = rigidBody->uuid;
        infiniteGround->name = rigidBody->name;

        return rigidBody;
    }


}

