整个Bullet引擎及对Bullet对象的引用全部采用内存池/对象池托管模式

见 [MemoryPool.cpp](MemoryPool.cpp)

```c++


MemoryPool::gpMemoryPoolManager = boost::make_shared<MemoryPool::MemoryCustomAllocatorManager>();


btAlignedAllocSetCustom(
MemoryPool::btAllocFunc,
MemoryPool::btFreeFunc
);



MemoryPool::gpMemoryPoolManager.reset();

```

引擎中对象创建方法，见 [OgreBullet.cpp](OgreBullet.cpp)


```c++

auto state = memoryContainerManager_->makePtr<RigidBodyState>(node);

auto shape = memoryContainerManager->makePtr<btBvhTriangleMeshShape>(trimesh, useQuantizedAABB);

auto rb = memoryContainerManager_->makeBody(
    memoryContainerManager_->makeRigidBodyPtr(
        mass,
        dynamic_cast<btMotionState *>(state.get()),
        dynamic_cast<btCollisionShape *>(cs.get()),
        inertia
    ),
    state
);
mBtWorld->addRigidBody(rb->ptrRigidBody.get(), group, mask);



// btTriangleMesh *trimesh = new btTriangleMesh();
btTriangleMesh *trimesh = memoryContainerManager->newRawPtr<btTriangleMesh>();
memoryContainerManager->deleteRawPtr(trimesh);


```
