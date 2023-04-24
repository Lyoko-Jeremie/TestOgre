// jeremie

#ifndef TESTOGRE_MEMORYPOOL_H
#define TESTOGRE_MEMORYPOOL_H

#include <mutex>
#include <cstdlib>
// when use boost::shared_ptr (boost::allocate_shared), both the data and control block are allocated use the given allocator
// but the c++ standard not define this, std::shared_ptr (std::allocate_shared) , it's impl defined.
// see https://stackoverflow.com/questions/68041405/who-allocates-the-memory-for-control-block-of-shared-ptr-when-using-custom-new
// so, don't use std::shared_ptr when you want to use the memory pool
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/make_shared.hpp>
#include <boost/make_unique.hpp>
#include <boost/smart_ptr/allocate_unique.hpp>

#include <boost/align/aligned_alloc.hpp>
#include <utility>

namespace MemoryPool {

    class MemoryCustomAllocatorManager : public boost::enable_shared_from_this<MemoryCustomAllocatorManager> {
    private:
        using DataType = unsigned char[];
        using DataTypePtr = unsigned char *;

        struct PtrInfoStruct {
            boost::shared_ptr<DataType> ptr{nullptr};
            size_t size{0};
            bool isAligned{false};

            PtrInfoStruct(
                    boost::shared_ptr<DataType> ptr,
                    size_t size,
                    bool isAligned
            ) : ptr(std::move(ptr)), size(size), isAligned(isAligned) {}
        };

        using PtrInfo = PtrInfoStruct;

        static PtrInfo createPtrInfo(
                boost::shared_ptr<DataType> ptr,
                size_t size,
                bool isAligned
        ) {
            return std::move(PtrInfoStruct{std::move(ptr), size, isAligned});
        }

        // we need the allocated memory pined at a place, so don't use std::vector replace smart ptr
        // when item erase, the memory block will deallocate by boost::shared_ptr
        std::unordered_map<DataTypePtr, PtrInfo> memoryPool;
        std::mutex mtx;

    public:

        void clear() {
            std::lock_guard lg{mtx};
            memoryPool.clear();
        }

        const auto &_getMemoryPoolRef() {
            return memoryPool;
        }

    public:

        void *allocate(const size_t size) {
            // https://en.cppreference.com/w/cpp/memory/c/malloc
            // The following functions are required to be thread-safe:
            //    * The library versions of operator new and operator delete
            //    * User replacement versions of global operator new and operator delete
            //    * std::calloc, std::malloc, std::realloc, std::aligned_alloc (since C++17), std::free
            // Calls to these functions that allocate or deallocate a particular unit of storage occur in a single total order, and each such deallocation call happens-before the next allocation (if any) in this order.
            //     --------(since C++11)
            //
            // So, the lock must protect hole new operate , don't try to smaller the lock area
            std::lock_guard lg{mtx};
            auto b = boost::make_shared<DataType>(size);
            // https://stackoverflow.com/questions/41748542/shared-ptr-custom-allocator-together-with-custom-deleter
            // https://en.cppreference.com/w/cpp/memory/c/malloc
            // https://en.cppreference.com/w/cpp/memory/c/free
            // auto b = boost::shared_ptr<DataType>((unsigned char *) std::malloc(size), std::free);
            if (b) {
                memoryPool.emplace(std::make_pair(b.get(), createPtrInfo(b, size, false)));
            }
            return b.get();
        }

        void *allocateAligned(const size_t size, const size_t alignment) {
            std::lock_guard lg{mtx};
            // https://stackoverflow.com/questions/41748542/shared-ptr-custom-allocator-together-with-custom-deleter
            // https://en.cppreference.com/w/cpp/memory/c/malloc
            // https://en.cppreference.com/w/cpp/memory/c/free
            auto b = boost::shared_ptr<DataType>(
                    (unsigned char *) boost::alignment::aligned_alloc(alignment, size),
                    boost::alignment::aligned_free
            );
            if (b) {
                memoryPool.emplace(std::make_pair(b.get(), createPtrInfo(b, size, true)));
            }
            return b.get();
        }

        void deallocate(void *const memblock) {
            std::lock_guard lg{mtx};
            auto n = memoryPool.find(reinterpret_cast<unsigned char *>(memblock));
            BOOST_ASSERT_MSG(n != memoryPool.end(), "cannot find ptr, bad memory.");
            memoryPool.erase(n);
        }

        // Is there a "placement delete"
        // https://www.stroustrup.com/bs_faq2.html#placement-delete
        template<typename Type>
        void deletePlacement(Type *const memblock) {
            std::lock_guard lg{mtx};
            auto n = memoryPool.find(reinterpret_cast<unsigned char *>(memblock));
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
