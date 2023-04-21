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




the Memory Manager

```

|-------------------------------------------------------------
| MemoryCustomAllocatorManager.memoryPool (memory pice)
| 
|    all memory hold by 
|         `extern boost::shared_ptr<MemoryPool::MemoryCustomAllocatorManager> gpMemoryPoolManager;`
| 
|
| |----------------------------------------
| | the bullet allocate function (thery are use the `gpMemoryPoolManager` )
| |        | btAlignedAllocFunc
| |        | btAlignedFreeFunc
| |        | btAllocFunc
| |        | btFreeFunc
| |
| |   MUST call `MemoryPool::setup()` to install the allocate functions
| |
| |-----------------------------------------
| 
| 
| |-------------------------------------
| | MemoryCustomAllocator (for any STL like Container who can use std::allocate)
| |
| |-------------------------------------
|
|
| |----------------------------------------------------
| | BulletMemoryContainerManager (create and manage all object in the memoryPool)
| |                              (life time tracer)
| |                                 | ccs
| |                                 | csc
| |                                 | roc
| |                               (tracer object create provider)
| |                                 | makeCollisionState
| |                                 | makeShape
| |                                 | makeBody
| |                                 |   + makeRigidBodyPtr
| |                                 |   + makeMotionStatePtr
| |                               (object create provider)
| |                                 | makePtr (make_shared)
| |                                 | makeUniquePtr (make_unique)
| |                               (object create provider without smart_ptr (work as raw `new` but allocate in memoryPool))
| |                                 | newRawPtr + deleteRawPtr
| | 
| |     |-------------------------------------------------------
| |     | The hole Bullet and data it used must place in there
| |     | 
| |     |
| |     |
| |     |-------------------------------------------------------
| |
| |-------------------------------------------------------------
|
|
|
|
|
|
|-------------------------------------------------------------






```















