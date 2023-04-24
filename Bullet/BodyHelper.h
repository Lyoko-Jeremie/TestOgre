// jeremie

#ifndef TESTOGRE_BODYHELPER_H
#define TESTOGRE_BODYHELPER_H

#include "./OgreBullet.h"

namespace Ogre::Bullet::BodyHelper {

    extern boost::shared_ptr<BulletMemoryContainer::BulletMemoryContainerManager::RigidObjectType>
    createInfiniteGround(
            boost::shared_ptr<Ogre::Bullet::DynamicsWorld> dynamicsWorld,
            btVector3 origin = btVector3{0, 0, 0},
            Entity *ent = nullptr,
            const boost::shared_ptr<DynamicsWorld::Bullet2OgreTracer> &bullet2OgreTracer = nullptr
    );

}

#endif //TESTOGRE_BODYHELPER_H
