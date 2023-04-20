// jeremie

#include "MemoryPool.h"

namespace MemoryPool {

    void setup() {

        MemoryPool::gpMemoryPoolManager = boost::make_shared<MemoryPool::MemoryCustomAllocatorManager>();


//    btAlignedAllocSetCustomAligned(
//            MemoryPool::btAlignedAllocFunc,
//            MemoryPool::btAlignedFreeFunc
//    );
//        btAlignedAllocSetCustom(
//                MemoryPool::btAllocFunc,
//                MemoryPool::btFreeFunc
//        );

//        BulletMemoryContainer::CollisionStateContainer<MemoryPool::MemoryCustomAllocator<BulletMemoryContainer::CollisionState>> ccs{
//                MemoryPool::MemoryCustomAllocator<BulletMemoryContainer::CollisionState>(
//                        MemoryPool::gpMemoryPoolManager)
//        };
//        BulletMemoryContainer::CollisionShapeContainer<MemoryPool::MemoryCustomAllocator<BulletMemoryContainer::CollisionShape>> csc{
//                MemoryPool::MemoryCustomAllocator<BulletMemoryContainer::CollisionShape>(
//                        MemoryPool::gpMemoryPoolManager)
//        };
//
//        BulletMemoryContainer::RigidObjectContainer<MemoryPool::MemoryCustomAllocator<BulletMemoryContainer::RigidObject>> roc{
//                MemoryPool::MemoryCustomAllocator<BulletMemoryContainer::RigidObject>(
//                        MemoryPool::gpMemoryPoolManager)
//        };


//        auto groundShape = csc.emplace_back(
//                boost::allocate_shared<btBoxShape>(
//                        MemoryPool::MemoryCustomAllocator<btBoxShape>(MemoryPool::gpMemoryPoolManager),
//                        btVector3(btScalar(50.), btScalar(50.), btScalar(50.)))
//        ).first;
//        auto myMotionState = boost::allocate_shared<btDefaultMotionState>(
//                MemoryPool::MemoryCustomAllocator<btDefaultMotionState>(MemoryPool::gpMemoryPoolManager),
//                groundTransform
//        );
//        auto body = roc.emplace_back(
//                boost::allocate_shared<btRigidBody>(
//                        MemoryPool::MemoryCustomAllocator<btRigidBody>(MemoryPool::gpMemoryPoolManager),
//                        rbInfo
//                ),
//                myMotionState
//        ).first;


//        std::unique_ptr<btDefaultCollisionConfiguration, boost::alloc_deleter<btDefaultCollisionConfiguration, MemoryPool::MemoryCustomAllocator<btDefaultCollisionConfiguration>>>

//        unique_ptr_with_alloc_deleter<btDefaultCollisionConfiguration>

//        auto collisionConfiguration = boost::allocate_unique<btDefaultCollisionConfiguration>(
//                MemoryPool::MemoryCustomAllocator<btDefaultCollisionConfiguration>(MemoryPool::gpMemoryPoolManager)
//        );
//        auto dispatcher = boost::allocate_unique<btCollisionDispatcher>(
//                MemoryPool::MemoryCustomAllocator<btCollisionDispatcher>(MemoryPool::gpMemoryPoolManager),
//                &*collisionConfiguration
//        );
//        auto overlappingPairCache = boost::allocate_unique<btDbvtBroadphase>(
//                MemoryPool::MemoryCustomAllocator<btDbvtBroadphase>(MemoryPool::gpMemoryPoolManager)
//        );
//        auto solver = boost::allocate_unique<btSequentialImpulseConstraintSolver>(
//                MemoryPool::MemoryCustomAllocator<btSequentialImpulseConstraintSolver>(MemoryPool::gpMemoryPoolManager)
//        );
//
//        auto dynamicsWorld = boost::allocate_unique<btDiscreteDynamicsWorld>(
//                MemoryPool::MemoryCustomAllocator<btDiscreteDynamicsWorld>(MemoryPool::gpMemoryPoolManager),
//                &*dispatcher,
//                &*overlappingPairCache,
//                &*solver,
//                &*collisionConfiguration
//        );

//        auto debugDraw = boost::allocate_unique<DebugDraw>(
//                MemoryPool::MemoryCustomAllocator<DebugDraw>(MemoryPool::gpMemoryPoolManager)
//        );


//        roc.clear();
//        csc.clear();
//        ccs.clear();
//        dynamicsWorld.reset();
//        solver.reset();
//        overlappingPairCache.reset();
//        dispatcher.reset();
//        collisionConfiguration.reset();


//        MemoryPool::gpMemoryPoolManager.reset();


    }

    void *btAlignedAllocFunc(size_t size, int alignment) {
        return gpMemoryPoolManager->allocate(size);
    }

    void btAlignedFreeFunc(void *memblock) {
        return gpMemoryPoolManager->deallocate(memblock);
    }

    void *btAllocFunc(size_t size) {
        return gpMemoryPoolManager->allocate(size);
    }

    void btFreeFunc(void *memblock) {
        return gpMemoryPoolManager->deallocate(memblock);
    }

}

