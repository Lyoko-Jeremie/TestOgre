// jeremie

#include "BulletMemoryPool.h"
#include "LinearMath/btAlignedAllocator.h"

namespace BulletMemoryPool {

    boost::shared_ptr<BulletMemoryPool::MemoryCustomAllocatorManager> gpMemoryPoolManager;

    void setup() {

        boost::shared_ptr<BulletMemoryPool::MemoryCustomAllocatorManager> nil;
        boost::atomic_compare_exchange(
                &BulletMemoryPool::gpMemoryPoolManager,
                &nil,
                boost::make_shared<BulletMemoryPool::MemoryCustomAllocatorManager>()
        );
//        BulletMemoryPool::gpMemoryPoolManager = boost::make_shared<BulletMemoryPool::MemoryCustomAllocatorManager>();


        btAlignedAllocSetCustomAligned(
                BulletMemoryPool::btAlignedAllocFunc,
                BulletMemoryPool::btAlignedFreeFunc
        );
        btAlignedAllocSetCustom(
                BulletMemoryPool::btAllocFunc,
                BulletMemoryPool::btFreeFunc
        );

//        BulletMemoryContainer::CollisionStateContainer<BulletMemoryPool::MemoryCustomAllocator<BulletMemoryContainer::CollisionState>> ccs{
//                BulletMemoryPool::MemoryCustomAllocator<BulletMemoryContainer::CollisionState>(
//                        BulletMemoryPool::gpMemoryPoolManager)
//        };
//        BulletMemoryContainer::CollisionShapeContainer<BulletMemoryPool::MemoryCustomAllocator<BulletMemoryContainer::CollisionShape>> csc{
//                BulletMemoryPool::MemoryCustomAllocator<BulletMemoryContainer::CollisionShape>(
//                        BulletMemoryPool::gpMemoryPoolManager)
//        };
//
//        BulletMemoryContainer::RigidObjectContainer<BulletMemoryPool::MemoryCustomAllocator<BulletMemoryContainer::RigidObject>> roc{
//                BulletMemoryPool::MemoryCustomAllocator<BulletMemoryContainer::RigidObject>(
//                        BulletMemoryPool::gpMemoryPoolManager)
//        };


//        auto groundShape = csc.emplace_back(
//                boost::allocate_shared<btBoxShape>(
//                        BulletMemoryPool::MemoryCustomAllocator<btBoxShape>(BulletMemoryPool::gpMemoryPoolManager),
//                        btVector3(btScalar(50.), btScalar(50.), btScalar(50.)))
//        ).first;
//        auto myMotionState = boost::allocate_shared<btDefaultMotionState>(
//                BulletMemoryPool::MemoryCustomAllocator<btDefaultMotionState>(BulletMemoryPool::gpMemoryPoolManager),
//                groundTransform
//        );
//        auto body = roc.emplace_back(
//                boost::allocate_shared<btRigidBody>(
//                        BulletMemoryPool::MemoryCustomAllocator<btRigidBody>(BulletMemoryPool::gpMemoryPoolManager),
//                        rbInfo
//                ),
//                myMotionState
//        ).first;


//        std::unique_ptr<btDefaultCollisionConfiguration, boost::alloc_deleter<btDefaultCollisionConfiguration, BulletMemoryPool::MemoryCustomAllocator<btDefaultCollisionConfiguration>>>

//        unique_ptr_with_alloc_deleter<btDefaultCollisionConfiguration>

//        auto collisionConfiguration = boost::allocate_unique<btDefaultCollisionConfiguration>(
//                BulletMemoryPool::MemoryCustomAllocator<btDefaultCollisionConfiguration>(BulletMemoryPool::gpMemoryPoolManager)
//        );
//        auto dispatcher = boost::allocate_unique<btCollisionDispatcher>(
//                BulletMemoryPool::MemoryCustomAllocator<btCollisionDispatcher>(BulletMemoryPool::gpMemoryPoolManager),
//                &*collisionConfiguration
//        );
//        auto overlappingPairCache = boost::allocate_unique<btDbvtBroadphase>(
//                BulletMemoryPool::MemoryCustomAllocator<btDbvtBroadphase>(BulletMemoryPool::gpMemoryPoolManager)
//        );
//        auto solver = boost::allocate_unique<btSequentialImpulseConstraintSolver>(
//                BulletMemoryPool::MemoryCustomAllocator<btSequentialImpulseConstraintSolver>(BulletMemoryPool::gpMemoryPoolManager)
//        );
//
//        auto dynamicsWorld = boost::allocate_unique<btDiscreteDynamicsWorld>(
//                BulletMemoryPool::MemoryCustomAllocator<btDiscreteDynamicsWorld>(BulletMemoryPool::gpMemoryPoolManager),
//                &*dispatcher,
//                &*overlappingPairCache,
//                &*solver,
//                &*collisionConfiguration
//        );

//        auto debugDraw = boost::allocate_unique<DebugDraw>(
//                BulletMemoryPool::MemoryCustomAllocator<DebugDraw>(BulletMemoryPool::gpMemoryPoolManager)
//        );


//        roc.clear();
//        csc.clear();
//        ccs.clear();
//        dynamicsWorld.reset();
//        solver.reset();
//        overlappingPairCache.reset();
//        dispatcher.reset();
//        collisionConfiguration.reset();


//        BulletMemoryPool::gpMemoryPoolManager.reset();


    }

    void *btAlignedAllocFunc(size_t size, int alignment) {
        return gpMemoryPoolManager->allocateAligned(size, alignment);
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

