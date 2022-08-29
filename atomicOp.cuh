#pragma once

  // Lifted from https://github.com/treecode/Bonsai/blob/master/runtime/profiling/derived_atomic_functions.h
  __device__ __forceinline__ float atomicMin(float *address, float val)
  {
      int ret = __float_as_int(*address);
      while(val < __int_as_float(ret))
      {
          int old = ret;
          if((ret = atomicCAS((int *)address, old, __float_as_int(val))) == old)
              break;
      }
      return __int_as_float(ret);
  }
  
  __device__ __forceinline__ float atomicMax(float *address, float val)
  {
      int ret = __float_as_int(*address);
      while(val > __int_as_float(ret))
      {
          int old = ret;
          if((ret = atomicCAS((int *)address, old, __float_as_int(val))) == old)
              break;
      }
      return __int_as_float(ret);
  }

