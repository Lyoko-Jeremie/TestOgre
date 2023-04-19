// jeremie

#ifndef TESTOGRE_BULLETMEMORYCONTAINER_H
#define TESTOGRE_BULLETMEMORYCONTAINER_H

#include <chrono>

#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/make_shared.hpp>
#include <boost/make_unique.hpp>
#include <boost/smart_ptr/allocate_unique.hpp>

#include <boost/multi_index_container.hpp>
#include <boost/multi_index/tag.hpp>
#include <boost/multi_index/member.hpp>
#include <boost/multi_index/identity.hpp>
#include <boost/multi_index/hashed_index.hpp>
#include <boost/multi_index/sequenced_index.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/random_access_index.hpp>
#include <boost/multi_index/composite_key.hpp>
#include <boost/multi_index/key.hpp>

#include "btBulletDynamicsCommon.h"


namespace BulletMemoryContainer {

    struct CollisionShape {
        static std::atomic_int idGenerator;

        int id;
        struct ID {
        };
        std::string name;
        struct NAME {
        };
        std::string uuid;
        struct UUID {
        };

        boost::shared_ptr<btCollisionShape> ptr;

        explicit CollisionShape(
                boost::shared_ptr<btCollisionShape> ptr_
        ) : id(++idGenerator), ptr(std::move(ptr_)) {
            ptr->setUserIndex(id);
        }

        auto operator<=>(const CollisionShape &o) const {
            return id <=> o.id;
        }
    };

    std::atomic_int CollisionShape::idGenerator{1};


    template<typename Allocator=std::allocator<CollisionShape>>
    using CollisionShapeContainer = boost::multi_index_container<
            CollisionShape,
            boost::multi_index::indexed_by<
                    boost::multi_index::sequenced<>,
                    boost::multi_index::ordered_unique<
                            boost::multi_index::identity<CollisionShape>
                    >,
                    boost::multi_index::hashed_non_unique<
                            boost::multi_index::tag<CollisionShape::NAME>,
                            boost::multi_index::member<CollisionShape, std::string, &CollisionShape::name>
                    >,
                    boost::multi_index::hashed_non_unique<
                            boost::multi_index::tag<CollisionShape::UUID>,
                            boost::multi_index::member<CollisionShape, std::string, &CollisionShape::uuid>
                    >,
                    boost::multi_index::hashed_unique<
                            boost::multi_index::tag<CollisionShape::ID>,
                            boost::multi_index::member<CollisionShape, int, &CollisionShape::id>
                    >,
                    boost::multi_index::random_access<>
            >,
            Allocator
    >;

    struct RigidObject {
        static std::atomic_int idGenerator;

        int id;
        struct ID {
        };
        std::string name;
        struct NAME {
        };
        std::string uuid;
        struct UUID {
        };

        boost::shared_ptr<btRigidBody> ptr;
        boost::shared_ptr<btMotionState> ptrMS;

        RigidObject(
                boost::shared_ptr<btRigidBody> ptr_,
                boost::shared_ptr<btMotionState> ptrMS_
        ) : id(++idGenerator), ptr(std::move(ptr_)), ptrMS(std::move(ptrMS_)) {
            ptr->setUserIndex(id);
        }

        auto operator<=>(const RigidObject &o) const {
            return id <=> o.id;
        }
    };

    std::atomic_int RigidObject::idGenerator{1};

    template<typename Allocator=std::allocator<CollisionShape>>
    using RigidObjectContainer = boost::multi_index_container<
            RigidObject,
            boost::multi_index::indexed_by<
                    boost::multi_index::sequenced<>,
                    boost::multi_index::ordered_unique<
                            boost::multi_index::identity<RigidObject>
                    >,
                    boost::multi_index::hashed_non_unique<
                            boost::multi_index::tag<RigidObject::NAME>,
                            boost::multi_index::member<RigidObject, std::string, &RigidObject::name>
                    >,
                    boost::multi_index::hashed_non_unique<
                            boost::multi_index::tag<RigidObject::UUID>,
                            boost::multi_index::member<RigidObject, std::string, &RigidObject::uuid>
                    >,
                    boost::multi_index::hashed_unique<
                            boost::multi_index::tag<RigidObject::ID>,
                            boost::multi_index::member<RigidObject, int, &RigidObject::id>
                    >,
                    boost::multi_index::random_access<>
            >,
            Allocator
    >;

    struct CollisionState {
        enum class State {
            start,
            collision,
            end,
        };
        int idA;
        struct IDA {
        };
        int idB;
        struct IDB {
        };
        struct ID {
        };

        State state;
        std::chrono::steady_clock::time_point lastCheckTime;

        CollisionState(
                int idA_,
                int idB_,
                State state_,
                std::chrono::steady_clock::time_point lastCheckTime_
        ) : idA(idA_), idB(idB_), state(state_), lastCheckTime(lastCheckTime_) {}


//    bool operator==(const CollisionState &o) const {
//        return idA == o.idA && idB == o.idB;
//    }
        auto operator<=>(const CollisionState &o) const {
            if (idA == o.idA) {
                return idB <=> o.idB;
            }
            return idA <=> o.idA;
        }
    };

    template<typename Allocator=std::allocator<CollisionState>>
    using CollisionStateContainer = boost::multi_index_container<
            CollisionState,
            boost::multi_index::indexed_by<
                    boost::multi_index::sequenced<>,
                    boost::multi_index::ordered_unique<
                            boost::multi_index::identity<CollisionState>
                    >,
                    boost::multi_index::hashed_unique<
                            boost::multi_index::tag<CollisionState::ID>,
                            boost::multi_index::composite_key<
                                    CollisionState,
                                    boost::multi_index::member<CollisionState, int, &CollisionState::idA>,
                                    boost::multi_index::member<CollisionState, int, &CollisionState::idB>
                            >
                    >,
                    boost::multi_index::hashed_non_unique<
                            boost::multi_index::tag<CollisionState::IDA>,
                            boost::multi_index::member<CollisionState, int, &CollisionState::idA>
                    >,
                    boost::multi_index::hashed_non_unique<
                            boost::multi_index::tag<CollisionState::IDB>,
                            boost::multi_index::member<CollisionState, int, &CollisionState::idB>
                    >,
                    boost::multi_index::random_access<>
            >,
            Allocator
    >;

}

namespace MemoryPool {

    class MemoryCustomAllocatorManager : public boost::enable_shared_from_this<MemoryCustomAllocatorManager> {
    private:
        using DataType = unsigned char[];

        // we need the allocated memory pined at a place, so don't use std::vector replace smart ptr
        // when item erase, the memory block will deallocate by boost::shared_ptr
        std::unordered_map<unsigned char *, boost::shared_ptr<DataType>> memoryPool;

    public:

        void reset() {
            memoryPool.clear();
        }

        void *allocate(const size_t size) {
            auto b = boost::make_shared<DataType>(size);
            if (b) {
                memoryPool.emplace(std::make_pair(b.get(), b));
            }
            return b.get();
        }

        void deallocate(void *const memblock) {
            auto n = memoryPool.find((unsigned char *) memblock);
            BOOST_ASSERT_MSG(n != memoryPool.end(), "cannot find ptr, bad memory.");
            memoryPool.erase(n);
        }
    };

    boost::shared_ptr<MemoryPool::MemoryCustomAllocatorManager> gpMemoryPoolManager;

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

    // from c++ <xmemory>
    template<class Type>
    class MemoryCustomAllocator : public boost::enable_shared_from_this<MemoryCustomAllocator<Type>> {
    public:

//        // DEPRECATED_ALLOCATOR_MEMBERS start
//        using pointer = Type *;
//        using const_pointer = const Type *;
//        using reference = Type &;
//        using const_reference = const Type &;
//        template<class _Other>
//        struct rebind {
//            using other = MemoryCustomAllocator<_Other>;
//        };
//
//        Type *address(Type &_Val) const noexcept {
//            return _STD addressof(_Val);
//        }
//
//        const Type *address(const Type &_Val) const noexcept {
//            return _STD addressof(_Val);
//        }
//
//        __declspec(allocator) Type *allocate(
//                const size_t _Count, const void *) {
//            return allocate(_Count);
//        }
//
//        template<class _Objty, class... _Types>
//        void construct(_Objty *const _Ptr, _Types &&... _Args) {
//            ::new(_Voidify_iter(_Ptr)) _Objty(_STD forward<_Types>(_Args)...);
//        }
//
//        template<class _Uty>
//        void destroy(_Uty *const _Ptr) {
//            _Ptr->~_Uty();
//        }
//
//        size_t max_size() const noexcept {
//            return static_cast<size_t>(-1) / sizeof(Type);
//        }
//
//        // DEPRECATED_ALLOCATOR_MEMBERS end

        using value_type = Type;

        using size_type = size_t;
        using difference_type = ptrdiff_t;

        using propagate_on_container_move_assignment = std::true_type;
        using is_always_equal = std::true_type;

        boost::shared_ptr<MemoryPool::MemoryCustomAllocatorManager> memoryManager_;

        explicit MemoryCustomAllocator(
                boost::shared_ptr<MemoryPool::MemoryCustomAllocatorManager> memoryManager
        ) noexcept: memoryManager_(std::move(memoryManager)) {}

        MemoryCustomAllocator(const MemoryCustomAllocator &o) noexcept {
            memoryManager_ = o.memoryManager_;
        };

        // don't mark explicit in this
        template<class Other>
        MemoryCustomAllocator(const MemoryCustomAllocator<Other> &o) noexcept {
            memoryManager_ = o.memoryManager_;
        }

        ~MemoryCustomAllocator() = default;

        MemoryCustomAllocator &operator=(const MemoryCustomAllocator &o) {
            memoryManager_ = o.memoryManager_;
        }

        void deallocate(Type *const Ptr, const size_t Count) {
            _STL_ASSERT(Ptr != nullptr || Count == 0, "null pointer cannot point to a block of non-zero size");
            // no overflow check on the following multiply; we assume _Allocate did that check
            // need check size when use vector
            memoryManager_->deallocate(Ptr);
        }

        __declspec(allocator) Type *allocate(const size_t Count) {
            static_assert(sizeof(value_type) > 0, "value_type must be complete before calling allocate.");
            constexpr bool Overflow_is_possible = sizeof(value_type) > 1;
            if constexpr (Overflow_is_possible) {
                constexpr size_t Max_possible = static_cast<size_t>(-1) / sizeof(value_type);
                if (Count > Max_possible) {
                    throw std::bad_array_new_length();
                }
            }
            return (Type *) memoryManager_->allocate(sizeof(value_type) * Count);
        }

    };

    template<typename T>
    using unique_ptr_with_alloc_deleter = std::unique_ptr<T, boost::alloc_deleter<T, MemoryPool::MemoryCustomAllocator<T>>>;

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


}


#endif //TESTOGRE_BULLETMEMORYCONTAINER_H
