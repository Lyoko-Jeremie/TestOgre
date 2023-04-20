// jeremie

#include "./BulletMemoryContainer.h"


namespace BulletMemoryContainer {

    std::atomic_int CollisionShape::idGenerator{1};

    std::atomic_int RigidObject::idGenerator{1};

}

