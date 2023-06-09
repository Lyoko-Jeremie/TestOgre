// jeremie

#ifndef TESTOGRE_BULLETMEMORYCONTAINER_H
#define TESTOGRE_BULLETMEMORYCONTAINER_H

#include <chrono>
#include <deque>

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
#include <utility>

#include "btBulletDynamicsCommon.h"

#include "./BulletMemoryPool.h"

namespace BulletMemoryContainer {

    class UserPtrBase {
    public:
        constexpr static const char *TypeNameTag = "UserPtrBase";
        const std::string typeName;

        UserPtrBase() = delete;

        UserPtrBase(const std::string &typeName) : typeName(typeName) {}

        virtual ~UserPtrBase() = default;
    };

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

        using Ptr = boost::shared_ptr<btCollisionShape>;
        Ptr ptr;

        boost::shared_ptr<UserPtrBase> userPtr;

        explicit CollisionShape(
                Ptr ptr_
        ) : id(++idGenerator), ptr(std::move(ptr_)) {
            ptr->setUserIndex(id);
        }

        auto operator<=>(const CollisionShape &o) const {
            return id <=> o.id;
        }
    };

    template<typename Allocator=std::allocator<CollisionShape>>
    using CollisionShapeContainer = boost::multi_index_container<
            boost::shared_ptr<CollisionShape>,
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

        using PtrRigidBody = boost::shared_ptr<btRigidBody>;
        PtrRigidBody ptrRigidBody;
        using PtrMotionState = boost::shared_ptr<btMotionState>;
        PtrMotionState ptrMotionState;

        boost::shared_ptr<UserPtrBase> userPtr;

        RigidObject(
                PtrRigidBody ptrRigidBody_,
                PtrMotionState ptrMotionState_
        ) : id(++idGenerator), ptrRigidBody(std::move(ptrRigidBody_)), ptrMotionState(std::move(ptrMotionState_)) {
            ptrRigidBody->setUserIndex(id);
        }

        auto operator<=>(const RigidObject &o) const {
            return id <=> o.id;
        }
    };


    template<typename Allocator=std::allocator<RigidObject>>
    using RigidObjectContainer = boost::multi_index_container<
            boost::shared_ptr<RigidObject>,
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
        size_t lastCheckTime;

        boost::shared_ptr<UserPtrBase> userPtr;

        CollisionState(
                int idA_,
                int idB_,
                State state_,
                size_t lastCheckTime_
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

    using CollisionStateContainerItemType = boost::shared_ptr<CollisionState>;
    template<typename Allocator=std::allocator<CollisionState>>
    using CollisionStateContainer = boost::multi_index_container<
            CollisionStateContainerItemType,
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


    class BulletMemoryContainerManager
            : public boost::enable_shared_from_this<BulletMemoryContainerManager> {
    public:

        using CollisionStateType = BulletMemoryContainer::CollisionState;
        using CollisionShapeType = BulletMemoryContainer::CollisionShape;
        using RigidObjectType = BulletMemoryContainer::RigidObject;

    private:

        boost::shared_ptr<BulletMemoryPool::MemoryCustomAllocatorManager> pMemoryPoolManager_;

        BulletMemoryContainer::CollisionStateContainer<BulletMemoryPool::MemoryCustomAllocator<CollisionStateType>> ccs{
                BulletMemoryPool::MemoryCustomAllocator<CollisionStateType>(pMemoryPoolManager_)
        };
        BulletMemoryContainer::CollisionShapeContainer<BulletMemoryPool::MemoryCustomAllocator<CollisionShapeType>> csc{
                BulletMemoryPool::MemoryCustomAllocator<CollisionShapeType>(pMemoryPoolManager_)
        };
        BulletMemoryContainer::RigidObjectContainer<BulletMemoryPool::MemoryCustomAllocator<RigidObjectType>> roc{
                BulletMemoryPool::MemoryCustomAllocator<RigidObjectType>(pMemoryPoolManager_)
        };

        // the weak_ptr will keep "control block" alive, so use it carefully
//        std::deque<boost::weak_ptr<UserPtrBase>> anyManagedPtrTracer;



    public:
        explicit BulletMemoryContainerManager(
                boost::shared_ptr<BulletMemoryPool::MemoryCustomAllocatorManager> pMemoryPoolManager
        ) : pMemoryPoolManager_(std::move(pMemoryPoolManager)) {}

        static boost::shared_ptr<BulletMemoryContainerManager>
        create(
                boost::shared_ptr<BulletMemoryPool::MemoryCustomAllocatorManager> pMemoryPoolManager
        ) {
            return boost::allocate_shared<BulletMemoryContainerManager>(
                    BulletMemoryPool::MemoryCustomAllocator<BulletMemoryContainerManager>(pMemoryPoolManager),
                    pMemoryPoolManager
            );
        }

    public:

        auto &getCollisionStateContainer() {
            return ccs;
        }

        auto getMemoryPoolManager() {
            return pMemoryPoolManager_;
        }

//        void cleanManagedPtrTracer() {
//            anyManagedPtrTracer.clear();
//        }

        template<typename Type, class... Args>
        auto makeSharedPtr(Args &&... args) {
            return boost::allocate_shared<Type>(
                    BulletMemoryPool::MemoryCustomAllocator<Type>(pMemoryPoolManager_),
                    std::forward<Args>(args)...
            );
        }

        template<typename Type, class... Args>
        BulletMemoryPool::unique_ptr_with_alloc_deleter<Type> makeUniquePtr(Args &&... args) {
            return boost::allocate_unique<Type>(
                    BulletMemoryPool::MemoryCustomAllocator<Type>(pMemoryPoolManager_),
                    std::forward<Args>(args)...
            );
        }

        template<class... Args>
        auto makeCollisionState(Args &&... args) {
            auto collisionState = ccs.emplace_back(
                    boost::allocate_shared<CollisionState>(
                            BulletMemoryPool::MemoryCustomAllocator<CollisionState>(pMemoryPoolManager_),
                            std::forward<Args>(args)...
                    )
            ).first;
            return *collisionState;
        }

        template<class... Args>
        auto makeRigidBodyPtr(Args &&... args) {
            return makeSharedPtr<btRigidBody, Args...>(
                    std::forward<Args>(args)...
            );
        }

        template<class... Args>
        auto makeMotionStatePtr(Args &&... args) {
            return makeSharedPtr<btMotionState, Args...>(
                    std::forward<Args>(args)...
            );
        }

        auto makeBody(
                RigidObjectType::PtrRigidBody ptrRigidBody_,
                RigidObjectType::PtrMotionState ptrMotionState_ = nullptr
        ) {
            auto body = roc.emplace_back(
                    makeSharedPtr<RigidObjectType>(ptrRigidBody_, ptrMotionState_)
            ).first;
            return *body;
        }

        auto makeShape(CollisionShapeType::Ptr shapePtr) {
            auto shape = csc.emplace_back(
                    makeSharedPtr<CollisionShapeType>(shapePtr)
            ).first;
            return *shape;
        }


        template<typename Type, class... Args>
        Type *newRawPtr(Args &&... args) {
            // placement new
//            return new(pMemoryPoolManager_->allocate(sizeof(Type))) Type(std::forward<Args>(args)...);
            Type *p = reinterpret_cast<Type *>(pMemoryPoolManager_->allocate(sizeof(Type)));
            // https://en.cppreference.com/w/cpp/memory/construct_at
            return std::construct_at(p, args...);
        }

        template<typename Type>
        void deleteRawPtr(Type *ptr) { pMemoryPoolManager_->deletePlacement<Type>(ptr); }

//        void *newRawPtr(size_t, void *ptr) { return ptr; }
//
//        void deleteRawPtr(void *, void *) {}
//
//        void *newRawPtr[](size_t sizeInBytes) { return btAlignedAlloc(sizeInBytes, 16); }
//
//        void deleteRawPtr[](void *ptr) { btAlignedFree(ptr); }
//
//        void *newRawPtr[](size_t, void *ptr) { return ptr; }
//
//        void deleteRawPtr[](void *, void *) {}

        boost::shared_ptr<CollisionShape> getCollisionShape(int id) {
            return *csc.get<CollisionShapeType::ID>().find(id);
        }

        boost::shared_ptr<RigidObject> getBody(int id) {
            return *roc.get<RigidObject::ID>().find(id);
        }


        std::pair<boost::shared_ptr<RigidObject>, boost::shared_ptr<RigidObject>> getBody2(int idA, int idB) {
            auto &it = roc.get<RigidObject::ID>();
            return std::make_pair(
                    *it.find(idA),
                    *it.find(idB)
            );
        }

    };

}


#endif //TESTOGRE_BULLETMEMORYCONTAINER_H
