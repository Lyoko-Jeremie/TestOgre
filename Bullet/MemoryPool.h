// jeremie

#ifndef TESTOGRE_MEMORYPOOL_H
#define TESTOGRE_MEMORYPOOL_H

#include <mutex>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/make_shared.hpp>
#include <boost/make_unique.hpp>
#include <boost/smart_ptr/allocate_unique.hpp>

namespace MemoryPool {

    class MemoryCustomAllocatorManager : public boost::enable_shared_from_this<MemoryCustomAllocatorManager> {
    private:
        using DataType = unsigned char[];

        // we need the allocated memory pined at a place, so don't use std::vector replace smart ptr
        // when item erase, the memory block will deallocate by boost::shared_ptr
        std::unordered_map<unsigned char *, boost::shared_ptr<DataType>> memoryPool;
        std::mutex mtx;

    public:

        void reset() {
            std::lock_guard lg{mtx};
            memoryPool.clear();
        }

        void *allocate(const size_t size) {
            auto b = boost::make_shared<DataType>(size);
            if (b) {
                std::lock_guard lg{mtx};
                memoryPool.emplace(std::make_pair(b.get(), b));
            }
            return b.get();
        }

        void deallocate(void *const memblock) {
            std::lock_guard lg{mtx};
            auto n = memoryPool.find((unsigned char *) memblock);
            BOOST_ASSERT_MSG(n != memoryPool.end(), "cannot find ptr, bad memory.");
            memoryPool.erase(n);
        }

        // Is there a "placement delete"
        // https://www.stroustrup.com/bs_faq2.html#placement-delete
        template<typename Type>
        void deletePlacement(Type *const memblock) {
            std::lock_guard lg{mtx};
            auto n = memoryPool.find((unsigned char *) memblock);
            BOOST_ASSERT_MSG(n != memoryPool.end(), "cannot find ptr, bad memory.");
            memblock->~Type();
            memoryPool.erase(n);
        }
    };

    extern boost::shared_ptr<MemoryPool::MemoryCustomAllocatorManager> gpMemoryPoolManager;

    extern void *btAlignedAllocFunc(size_t size, int alignment);

    extern void btAlignedFreeFunc(void *memblock);

    extern void *btAllocFunc(size_t size);

    extern void btFreeFunc(void *memblock);

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

    extern void setup();


}


#endif //TESTOGRE_MEMORYPOOL_H
