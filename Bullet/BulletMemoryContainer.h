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
#include <utility>

#include "btBulletDynamicsCommon.h"

#include "./MemoryPool.h"

namespace BulletMemoryContainer {

    class UserPtrBase {
    public:
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
        std::chrono::steady_clock::time_point lastCheckTime;

        boost::shared_ptr<UserPtrBase> userPtr;

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
            boost::shared_ptr<CollisionState>,
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

        boost::shared_ptr<MemoryPool::MemoryCustomAllocatorManager> pMemoryPoolManager_;

        BulletMemoryContainer::CollisionStateContainer<MemoryPool::MemoryCustomAllocator<CollisionStateType>> ccs{
                MemoryPool::MemoryCustomAllocator<CollisionStateType>(pMemoryPoolManager_)
        };
        BulletMemoryContainer::CollisionShapeContainer<MemoryPool::MemoryCustomAllocator<CollisionShapeType>> csc{
                MemoryPool::MemoryCustomAllocator<CollisionShapeType>(pMemoryPoolManager_)
        };
        BulletMemoryContainer::RigidObjectContainer<MemoryPool::MemoryCustomAllocator<RigidObjectType>> roc{
                MemoryPool::MemoryCustomAllocator<RigidObjectType>(pMemoryPoolManager_)
        };

    public:

        explicit BulletMemoryContainerManager(
                boost::shared_ptr<MemoryPool::MemoryCustomAllocatorManager> pMemoryPoolManager
        ) : pMemoryPoolManager_(std::move(pMemoryPoolManager)) {}

        auto getMemoryPoolManager() {
            return pMemoryPoolManager_;
        }

        template<class... Args>
        auto makeCollisionState(Args &&... args) {
            auto collisionState = ccs.emplace_back(
                    boost::allocate_shared<CollisionState>(
                            MemoryPool::MemoryCustomAllocator<CollisionState>(pMemoryPoolManager_),
                            std::forward<Args>(args)...
                    )
            ).first;
            return *collisionState;
        }

        template<class... Args>
        auto makeRigidBodyPtr(Args &&... args) {
            return makePtr<btRigidBody, Args...>(
                    std::forward<Args>(args)...
            );
        }

        template<class... Args>
        auto makeMotionStatePtr(Args &&... args) {
            return makePtr<btMotionState, Args...>(
                    std::forward<Args>(args)...
            );
        }

        template<typename Type, class... Args>
        auto makePtr(Args &&... args) {
            return boost::allocate_shared<Type>(
                    MemoryPool::MemoryCustomAllocator<Type>(pMemoryPoolManager_),
                    std::forward<Args>(args)...
            );
        }

        auto makeBody(
                RigidObjectType::PtrRigidBody ptrRigidBody_,
                RigidObjectType::PtrMotionState ptrMotionState_ = nullptr
        ) {
            auto body = roc.emplace_back(
                    makePtr<RigidObjectType>(ptrRigidBody_, ptrMotionState_)
            ).first;
            return *body;
        }

        auto makeShape(CollisionShapeType::Ptr shapePtr) {
            auto shape = csc.emplace_back(
                    makePtr<CollisionShapeType>(shapePtr)
            ).first;
            return *shape;
        }


    };

}


#endif //TESTOGRE_BULLETMEMORYCONTAINER_H
