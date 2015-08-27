#ifndef ARC_HELPERS_HPP
#define ARC_HELPERS_HPP

// Branch prediction hints
// Figure out which compiler we have
#if defined(__clang__)
    /* Clang/LLVM */
    #define likely(x) __builtin_expect(!!(x), 1)
    #define unlikely(x) __builtin_expect(!!(x), 0)
#elif defined(__ICC) || defined(__INTEL_COMPILER)
    /* Intel ICC/ICPC */
    #define likely(x) __builtin_expect(!!(x), 1)
    #define unlikely(x) __builtin_expect(!!(x), 0)
#elif defined(__GNUC__) || defined(__GNUG__)
    /* GNU GCC/G++ */
    #define likely(x) __builtin_expect(!!(x), 1)
    #define unlikely(x) __builtin_expect(!!(x), 0)
#elif defined(_MSC_VER)
    /* Microsoft Visual Studio */
    /* MSVC doesn't support branch prediction hints. Use PGO instead. */
    #define likely(x) (x)
    #define unlikely(x) (x)
#endif

#endif // ARC_HELPERS_HPP
