
#pragma once

#ifdef _WIN32
#ifdef EXA_STITCH_DEVICE_STATIC_DEFINE
#define EXA_STITCH_DEVICE_INTERFACE
#else
#ifdef anari_library_exastitch_EXPORTS
#define EXA_STTICH_DEVICE_INTERFACE __declspec(dllexport)
#else
#define EXA_STITCH_DEVICE_INTERFACE __declspec(dllimport)
#endif
#endif
#elif defined __GNUC__
#define EXA_STITCH_DEVICE_INTERFACE __attribute__((__visibility__("default")))
#else
#define EXA_STITCH_DEVICE_INTERFACE
#endif

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

