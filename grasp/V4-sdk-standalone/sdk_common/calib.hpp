#ifndef _LIBRC_CALIB_HPP_
#define _LIBRC_CALIB_HPP_

#include <stdint.h>
#include <stdbool.h>
#include <numeric>
#include "matrix.hpp"

namespace librc
{
	namespace linalg
	{
		template<typename T = float>
		inline T interp_1d(T y1, T x1, T y2, T x2, T val)
		{
			T dy = y2 - y1;
			T dx = x2 - x1;

			if (dx == (T)0.f)
				dx = std::numeric_limits<T>::epsilon();

			T a = dy / dx;
			T b = y1;

			return a * (val - x1) + b;
		}

		template<uint32_t N, typename T = float>
		class calib_t
		{
		public:
			T internal_base;
			T internal_gap;
			//vector_t<N, T> internal_val;
			vector_t<N, T> external_val;

		public:
			inline T map(T val)
			{
				int32_t index = std::floor(val / internal_gap);
				
				if (index < 0)
				{
					// use first 2 samples to calibrate
					T x1 = internal_base;
					T x2 = internal_base + internal_gap;

					T y1 = external_val[0];
					T y2 = external_val[1];

					return interp_1d(y1, x1, y2, x2, val);

				}
				else if (index > N - 2)
				{
					// use last 2 samples to calibrate
					T x1 = internal_base + (N - 2) * internal_gap;
					T x2 = internal_base + (N - 1) * internal_gap;

					T y1 = external_val[N - 2];
					T y2 = external_val[N - 1];

					return interp_1d(y1, x1, y2, x2, val);
				}
				else
				{
					T x1 = internal_base + index * internal_gap;
					T x2 = internal_base + (index + 1) * internal_gap;

					T y1 = external_val[index];
					T y2 = external_val[index + 1];

					return interp_1d(y1, x1, y2, x2, val);
				}
			}

			inline T operator()(T val)
			{
				return map(val);
			}
//# pragma GCC push_options
//# pragma GCC optimize ("-O0")
			// map_src must be incremental
			static calib_t<N, T> create(vector_t<N, T> map_src, vector_t<N, T> map_dst)
			{
				calib_t<N, T> block;
				
				vector_t<N, T> norm_src;
				vector_t<N, T> norm_dst;
				
				T interval = (map_src(N - 1) - map_src(0)) / N;
				
				uint32_t sector = 0;
				
				T x1 = 0.f;
				T y1 = 0.f;
						
				T x2 = 0.f;
				T y2 = 0.f;
				
				for(uint32_t i=0; i<N; i++)
				{
					T val = norm_src(i) = map_src(0) + interval * i;
					
					while (sector + 1 < N)
					{
						x1 = map_src(sector);
						y1 = map_dst(sector);
					
						x2 = map_src(sector + 1);
						y2 = map_dst(sector + 1);
							
						if (val > x2)
							sector++;
						else
							break;
					}
						
					norm_dst(i) = interp_1d(y1, x1, y2, x2, val);
				}
				
				block.external_val = norm_dst;
				block.internal_base = norm_src(0);
				block.internal_gap = interval;
				
				return block;
			}

//# pragma GCC pop_options
		};
		
		// template<uint32_t N, typename T = float>
		// inline calib_t<N, T> create_calib_forward(T internal_base, T internal_gap, vector_t<N, T> external_val)
		// {
		// 	calib_t<N, T> block;
		// 	
		// 	block.internal_base = internal_base;
		// 	block.internal_gap = internal_gap;
		// 	block.external_val = external_val;
		// 	
		// 	return block;
		// }
		// 
		// template<uint32_t N, typename T = float>
		// inline calib_t<N, T> create_calib_reverse(calib_t<N, T> forward)
		// {
		// 	calib_t<N, T> block;
		// 	
		// 	block.internal_base = forward.external_val(0);
		// 	block.internal_gap = (forward.external_val(N-1) - forward.external_val(0)) / N;
		// 	
		// 	for (uint32_t i = 0; i < N; i++)
		// 		block.external_val = forward(block.internal_base + i * block.internal_gap);
		// 	
		// 	return block;
		// }
	}
}

#endif