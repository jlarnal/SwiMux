#ifndef H_SWIMUXDELEGATEFUNC_H
#define H_SWIMUXDELEGATEFUNC_H

#include <stdint.h>
#include <stddef.h>

template <typename> class SwiMuxDelegateFunc_t;

template <typename Ret, typename... Args> class SwiMuxDelegateFunc_t<Ret(Args...)> {
    using FnPtr = Ret (*)(void*, Args...);

    void* obj;
    FnPtr fn;

  public:
   
    SwiMuxDelegateFunc_t() : obj(nullptr), fn(nullptr) {}

    /** Move constructor (when the argument is meant to be deleted afterhand).*/
    SwiMuxDelegateFunc_t(SwiMuxDelegateFunc_t&& other) : obj(other.obj), fn(other.fn) {}

    /** Copy constructor. */
    SwiMuxDelegateFunc_t(const SwiMuxDelegateFunc_t& other) : obj(other.obj), fn(other.fn) {}

    // lambdas / functors
    template <typename T> SwiMuxDelegateFunc_t(const T& lambda)
    {
        // We must cast away the const to store it in a void*, but it's safe
        // because the lambda's call operator is const by default.
        obj = const_cast<void*>(static_cast<const void*>(&lambda));
        fn  = [](void* o, Args... args) -> Ret {
            // Cast back to a const T* to correctly invoke the call operator
            return (*static_cast<const T*>(o))(args...);
        };
    }
    
    // raw function pointer
    SwiMuxDelegateFunc_t(Ret (*func)(Args...))
    {
        obj = reinterpret_cast<void*>(func);
        fn  = [](void* o, Args... args) -> Ret {
            auto f = reinterpret_cast<Ret (*)(Args...)>(o);
            return f(args...);
        };
    }

    void clear()
    {
        fn  = nullptr;
        obj = nullptr;
    }

    SwiMuxDelegateFunc_t& operator=(const SwiMuxDelegateFunc_t& other)
    {
        fn  = nullptr;
        obj = nullptr;
        obj = other.obj;
        fn  = other.fn;
        return *this;
    }

    bool operator==(SwiMuxDelegateFunc_t& other) { return obj == other.obj && fn == other.fn; }

    // allow = nullptr
    SwiMuxDelegateFunc_t(nullptr_t) : obj(nullptr), fn(nullptr) {}


    Ret operator()(Args... args) const { return fn(obj, args...); }

    explicit operator bool() const { return fn != nullptr; }
};

#endif // H_SWIMUXDELEGATEFUNC_H