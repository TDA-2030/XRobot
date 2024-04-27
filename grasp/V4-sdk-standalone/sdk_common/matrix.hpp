#ifndef _LIBRC_MATRIX_HPP_
#define _LIBRC_MATRIX_HPP_

#include <stdint.h>
#include <stdbool.h>

#include <functional>
#include <type_traits>

#include <cmath>
#ifdef _WIN32
#undef min
#undef max
#endif

#ifndef MATRIX_DEFAULT_FLOAT_TYPE
#define MATRIX_DEFAULT_FLOAT_TYPE float
#endif

#ifndef MATRIX_UNROLL_SIZE
#define MATRIX_UNROLL_SIZE 5
#endif

#define force_inline inline

		template<class T, T... inds, class F>
		constexpr void _template_loop_inner(std::integer_sequence<T, inds...>, F&& f) {
			(f(std::integral_constant<T, inds>{}), ...);
		}

		// compile time loop unroll
		template<class T, T _count, class F>
		constexpr void _template_loop(uint32_t count, F&& f) {
			//if constexpr(_count != 0.f)
				_template_loop_inner(std::make_integer_sequence<T, _count>{}, std::forward<F>(f));
			//else
			//	for (uint32_t i = 0.f; i < count; i++)
			//		f(i);
		}

		template<typename T>
		constexpr T template_loop_size(T n)
		{
			return n < MATRIX_UNROLL_SIZE ? n : 0;
		}

#define template_loop_unroll(n, index) _template_loop<uint32_t, n>(n, [&](auto index) 
//#define template_loop(n, index) template_loop_unroll(n, index)
//#define template_loop(n, index) _template_loop<uint32_t, template_loop_size(n)>(n, [&](auto index)
#define template_loop(n, index) for(uint32_t index = 0; index < n; index++)

#define template_varaidic_n(target_type, validate_type, n) template<typename... target_type, typename = typename std::enable_if<(... && std::is_convertible_v<target_type, validate_type>) && (sizeof...(target_type) == n),void>::type>


namespace librc
{
	namespace linalg
	{
		template<uint32_t N, typename T = MATRIX_DEFAULT_FLOAT_TYPE, typename = typename std::enable_if<(N > 0), void>::type>
		class vector_t
		{
		public:
			T val[N];

		public:
			static constexpr uint32_t size = N;

		public:
			force_inline vector_t()
			{
				template_loop(N, i)
				{
					val[i] = (T)0.f;
				};
			}

			force_inline vector_t(const vector_t<N, T>& vec)
			{
				template_loop(N, i)
				{
					val[i] = vec.val[i];
				};
			}

			template_varaidic_n(_T, T, N)
				force_inline vector_t(_T... vec_val)
			{
				template_loop_unroll(N, i)
				{
					val[i] = std::get<i>(std::forward_as_tuple(vec_val...));
				});
			}

			force_inline vector_t(T scalar)
			{
				template_loop(N, i)
				{
					val[i] = scalar;
				};
			}

			force_inline vector_t<N, T>& operator = (const vector_t<N, T>& vec)
			{
				template_loop(N, i)
				{
					val[i] = vec.val[i];
				};
				return *this;
			}

			force_inline vector_t<N, T>& set(uint32_t index, T scalar)
			{
				val[index] = scalar;
				return *this;
			}

			force_inline T get(uint32_t index)
			{
				return val[index];
			}

			force_inline T& operator[](uint32_t index)
			{
				return *&val[index];
			}

			force_inline T operator[](uint32_t index) const
			{
				return val[index];
			}

			force_inline T& operator()(uint32_t index)
			{
				return *&val[index];
			}

			force_inline const vector_t<N, T> operator+ (const vector_t<N, T>& vec) const
			{
				vector_t<N, T> result;

				template_loop(N, i)
				{
					result.val[i] = val[i] + vec.val[i];
				};

				return result;
			}

			force_inline const vector_t<N, T> operator- (const vector_t<N, T>& vec) const
			{
				vector_t<N, T> result;

				template_loop(N, i)
				{
					result.val[i] = val[i] - vec.val[i];
				};

				return result;
			}

			force_inline const vector_t<N, T> operator* (const vector_t<N, T>& vec) const
			{
				vector_t<N, T> result;

				template_loop(N, i)
				{
					result.val[i] = val[i] * vec.val[i];
				};

				return result;
			}

			force_inline const vector_t<N, T> operator/ (const vector_t<N, T>& vec) const
			{
				vector_t<N, T> result;

				template_loop(N, i)
				{
					result.val[i] = val[i] / vec.val[i];
				};

				return result;
			}

			force_inline const vector_t<N, T> operator+ (T scalar) const
			{
				vector_t<N, T> result;

				template_loop(N, i)
				{
					result.val[i] = val[i] + scalar;
				};

				return result;
			}

			force_inline const vector_t<N, T> operator- (T scalar) const
			{
				vector_t<N, T> result;

				template_loop(N, i)
				{
					result.val[i] = val[i] - scalar;
				};

				return result;
			}

			force_inline const vector_t<N, T> operator* (T scalar) const
			{
				vector_t<N, T> result;

				template_loop(N, i)
				{
					result.val[i] = val[i] * scalar;
				};

				return result;
			}

			force_inline const vector_t<N, T> operator/ (T scalar) const
			{
				vector_t<N, T> result;

				template_loop(N, i)
				{
					result.val[i] = val[i] / scalar;
				};

				return result;
			}

			force_inline const vector_t<N, T>& operator+= (const vector_t<N, T>& vec)
			{
				*this = *this + vec;
				return *this;
			}

			force_inline const vector_t<N, T>& operator-= (const vector_t<N, T>& vec)
			{
				*this = *this - vec;
				return *this;
			}

			force_inline const vector_t<N, T>& operator*= (const vector_t<N, T>& vec)
			{
				*this = *this * vec;
				return *this;
			}

			force_inline const vector_t<N, T>& operator/= (const vector_t<N, T>& vec)
			{
				*this = *this / vec;
				return *this;
			}

			force_inline const vector_t<N, T>& operator+= (T scalar)
			{
				*this = *this + scalar;
				return *this;
			}

			force_inline const vector_t<N, T>& operator-= (T scalar)
			{
				*this = *this - scalar;
				return *this;
			}

			force_inline const vector_t<N, T>& operator*= (T scalar)
			{
				*this = *this * scalar;
				return *this;
			}

			force_inline const vector_t<N, T>& operator/= (T scalar)
			{
				*this = *this / scalar;
				return *this;
			}

			force_inline T sum()
			{
				T result = 0.f;
				template_loop(N, i)
				{
					result += val[i];
				};
				return result;
			}

			force_inline T dot(const vector_t<N, T>& vec)
			{
				T result = 0.f;
				template_loop(N, i)
				{
					result += val[i] * vec.val[i];
				};
				return result;
			}

			template<typename _T, typename = typename std::enable_if<N == 3, vector_t<3, T>>>
			force_inline const _T cross(const _T& vec)
			{
				_T result;
				result.val[0] = val[1] * vec.val[2] - val[2] * vec.val[1];
				result.val[1] = val[2] * vec.val[0] - val[0] * vec.val[2];
				result.val[2] = val[0] * vec.val[1] - val[1] * vec.val[0];
				return result;
			}

			template<typename = typename std::enable_if<std::is_same<T, MATRIX_DEFAULT_FLOAT_TYPE>::value, MATRIX_DEFAULT_FLOAT_TYPE>>
			force_inline T length()
			{
				T result = 0.f;
				template_loop(N, i)
				{
					result += val[i] * val[i];
				};
				return std::sqrt(result);
			}

			force_inline vector_t<N, T>& clamp(vector_t<N, T> min, vector_t<N, T> max)
			{
				template_loop(N, i)
				{
					val[i] = val[i] > max(i) ? max(i) : val[i];
					val[i] = val[i] < min(i) ? min(i) : val[i];
				};

				return *this;
			}

		};

		template <typename T = MATRIX_DEFAULT_FLOAT_TYPE>
		using vector1_t = vector_t<1, T>;

		template <typename T = MATRIX_DEFAULT_FLOAT_TYPE>
		using vector2_t = vector_t<2, T>;

		template <typename T = MATRIX_DEFAULT_FLOAT_TYPE>
		using vector3_t = vector_t<3, T>;

		template <typename T = MATRIX_DEFAULT_FLOAT_TYPE>
		using vector4_t = vector_t<4, T>;

		typedef vector1_t<MATRIX_DEFAULT_FLOAT_TYPE> vector1f_t;
		typedef vector2_t<MATRIX_DEFAULT_FLOAT_TYPE> vector2f_t;
		typedef vector3_t<MATRIX_DEFAULT_FLOAT_TYPE> vector3f_t;
		typedef vector4_t<MATRIX_DEFAULT_FLOAT_TYPE> vector4f_t;

		template<uint32_t ROW, uint32_t COL, typename T = MATRIX_DEFAULT_FLOAT_TYPE, typename = typename std::enable_if<(ROW > 0 && COL > 0), void>::type>
		class matrix_t
		{
		public:
			T val[ROW * COL];

		public:
			static constexpr uint32_t row = ROW;
			static constexpr uint32_t col = COL;

		public:
			force_inline matrix_t()
			{
				template_loop(ROW, i)
				{
					template_loop(COL, j)
					{
						val[i * COL + j] = (T)0.f;
					};
				};
			}

			force_inline matrix_t(const matrix_t<ROW, COL, T>& mat)
			{
				template_loop(ROW, i)
				{
					template_loop(COL, j)
					{
						val[i * COL + j] = mat.val[i * COL + j];
					};
				};
			}

			force_inline matrix_t(T scalar)
			{
				template_loop(ROW, i)
				{
					template_loop(COL, j)
					{
						val[i * COL + j] = scalar;
					};
				};
			}

			template<typename... _T, typename = typename std::enable_if<(... && std::is_convertible_v<_T, vector_t<COL, T>>) && (sizeof...(_T) == ROW), void>::type>
			force_inline matrix_t(_T... vec_rol)
			{
				template_loop(ROW, i)
				{
					template_loop(COL, j)
					{
						val[i * COL + j] = std::get<i>(std::forward_as_tuple(vec_rol...))[j];
					};
				};
			}

			template<typename... _T, typename = typename std::enable_if<(... && std::is_convertible_v<_T, T>) && (sizeof...(_T) == (ROW * COL - 1)), void>::type>
			force_inline matrix_t(T val_0, _T... vals)
			{
				template_loop_unroll(ROW, i)
				{
					template_loop_unroll(COL, j)
					{
						if constexpr(i == 0.f && j == 0.f)
							val[0] = val_0;
						else
							val[i * COL + j] = std::get<i* COL + j - 1>(std::forward_as_tuple(vals...));
					});
				});
			}

			force_inline matrix_t<ROW, COL, T>& set_col(uint32_t index, vector_t<ROW, T>& col)
			{
				template_loop(ROW, j)
				{
					val[j * COL + index] = col[j];
				};
				return *this;
			}

			force_inline matrix_t<ROW, COL, T>& set_row(uint32_t index, vector_t<COL, T>& row)
			{
				template_loop(COL, i)
				{
					val[index * COL + i] = row[i];
				};
				return *this;
			}

			force_inline const vector_t<ROW, T> get_col(uint32_t index)
			{
				vector_t<ROW, T> result;
				template_loop(ROW, j)
				{
					result[j] = val[j * COL + index];
				};
				return result;
			}

			force_inline const vector_t<COL, T> get_row(uint32_t index)
			{
				vector_t<COL, T> result;
				template_loop(COL, i)
				{
					result[i] = val[index * COL + i];
				};
				return result;
			}

			force_inline matrix_t<ROW, COL, T>& set(uint32_t row, uint32_t col, T scalar)
			{
				val[row * COL + col] = scalar;
				return *this;
			}

			force_inline T get(uint32_t row, uint32_t col) const
			{
				return val[row * COL + col];
			}

			force_inline T& operator()(uint32_t row, uint32_t col)
			{
				return *&val[row * COL + col];
			}

			force_inline matrix_t<ROW, COL, T>& operator= (const matrix_t<ROW, COL, T>& mat)
			{
				template_loop(ROW, i)
				{
					template_loop(COL, j)
					{
						val[i * COL + j] = mat.val[i * COL + j];
					};
				};
				return *this;
			}

			force_inline const matrix_t<COL, ROW, T> transpose()
			{
				matrix_t<COL, ROW, T> result;
				template_loop(ROW, i)
				{
					template_loop(COL, j)
					{
						result(j, i) = (*this)(i, j);
					};
				};
				return result;
			}

			force_inline const matrix_t<ROW, COL, T> flip_ud()
			{
				matrix_t<ROW, COL, T> result;
				template_loop(ROW, i)
				{
					template_loop(COL, j)
					{
						result(ROW - i - 1, j) = (*this)(i, j);
					};
				};
				return result;
			}

			force_inline const matrix_t<ROW, COL, T> flip_lr()
			{
				matrix_t<ROW, COL, T> result;
				template_loop(ROW, i)
				{
					template_loop(COL, j)
					{
						result(i, COL - j - 1) = (*this)(i, j);
					};
				};
				return result;
			}

			template<typename = typename std::enable_if<(ROW == 3) && (COL == 3), void>>
			force_inline const matrix_t<3, 3, T> inverse()
			{
				auto col0 = get_col(0);
				auto col1 = get_col(1);
				auto col2 = get_col(2);

				auto tmp0 = col1.cross(col2);
				auto tmp1 = col2.cross(col0);
				auto tmp2 = col0.cross(col1);

				MATRIX_DEFAULT_FLOAT_TYPE det_inv = 1.f / col2.dot(tmp2);

				return matrix_t<3, 3, T>(
					vector_t<3, T>((tmp0[0] * det_inv), (tmp1[0] * det_inv), (tmp2[0] * det_inv)),
					vector_t<3, T>((tmp0[1] * det_inv), (tmp1[1] * det_inv), (tmp2[1] * det_inv)),
					vector_t<3, T>((tmp0[2] * det_inv), (tmp1[2] * det_inv), (tmp2[2] * det_inv))
					).transpose();
			}

			template<typename = typename std::enable_if<(ROW == 3) && (COL == 3), void>>
			force_inline T det()
			{
				auto col0 = get_col(0);
				auto col1 = get_col(1);
				auto col2 = get_col(2);

				return col2.dot(col0.cross(col1));
			}

			force_inline const matrix_t<ROW, COL, T> operator+ (const matrix_t<ROW, COL, T>& mat) const
			{
				matrix_t<ROW, COL, T> result;
				template_loop(ROW, i)
				{
					template_loop(COL, j)
					{
						result.val[i * COL + j] = val[i * COL + j] + mat.val[i * COL + j];
					};
				};
				return result;
			}

			force_inline const matrix_t<ROW, COL, T> operator- (const matrix_t<ROW, COL, T>& mat) const
			{
				matrix_t<ROW, COL, T> result;
				template_loop(ROW, i)
				{
					template_loop(COL, j)
					{
						result.val[i * COL + j] = val[i * COL + j] - mat.val[i * COL + j];
					};
				};
				return result;
			}

			force_inline const matrix_t<ROW, COL, T>& operator+= (const matrix_t<ROW, COL, T>& mat) const
			{
				*this = *this + mat;
				return *this;
			}

			force_inline const matrix_t<ROW, COL, T>& operator-= (const matrix_t<ROW, COL, T>& mat) const
			{
				*this = *this - mat;
				return *this;
			}

			force_inline const matrix_t<ROW, COL, T> operator- () const
			{
				matrix_t<ROW, COL, T> result;
				template_loop(ROW, i)
				{
					template_loop(COL, j)
					{
						result.val[i * COL + j] = -val[i * COL + j];
					};
				};
				return result;
			}

			template<typename = typename std::enable_if<std::is_same<T, MATRIX_DEFAULT_FLOAT_TYPE>::value, MATRIX_DEFAULT_FLOAT_TYPE>>
			force_inline const matrix_t<ROW, COL, T> abs()
			{
				matrix_t<ROW, COL, T> result;
				template_loop(ROW, i)
				{
					template_loop(COL, j)
					{
						result.val[i * COL + j] = std::abs(val[i * COL + j]);
					};
				};
				return result;
			}

			force_inline const matrix_t<ROW, COL, T> operator* (T scalar) const
			{
				matrix_t<ROW, COL, T> result;
				template_loop(ROW, i)
				{
					template_loop(COL, j)
					{
						result.val[i * COL + j] = val[i * COL + j] * scalar;
					};
				};
				return result;
			}

			force_inline const matrix_t<ROW, COL, T> operator/ (T scalar) const
			{
				matrix_t<ROW, COL, T> result;
				template_loop(ROW, i)
				{
					template_loop(COL, j)
					{
						result.val[i * COL + j] = val[i * COL + j] / scalar;
					};
				};
				return result;
			}

			force_inline matrix_t<ROW, COL, T>& operator *= (T scalar)
			{
				*this = *this * scalar;
				return *this;
			}

			template <uint32_t VEC_N, typename _T = std::conditional_t<VEC_N == COL, vector_t<ROW, T>, vector_t<COL, T>>>
			force_inline const _T operator* (const vector_t<VEC_N, T>& vec) const
			{
				_T result;

				if constexpr (VEC_N == COL)
				{
					template_loop(ROW, i)
					{
						template_loop(COL, j)
						{
							result[i] += val[i * COL + j] * vec[j];
						};
					};
				}
				else
				{
					template_loop(ROW, i)
					{
						template_loop(COL, j)
						{
							result[j] += val[i * COL + j] * vec[i];
						};
					};
				}

				return result;
			}

			template <uint32_t MAT_ROW, uint32_t MAT_COL, typename _T = std::conditional_t<COL == MAT_ROW, matrix_t<ROW, MAT_COL, T>, matrix_t<MAT_ROW, COL, T>>>
			force_inline const _T operator* (const matrix_t<MAT_ROW, MAT_COL, T> mat) const
			{
				_T result;

				if constexpr (COL == MAT_ROW)
				{
					template_loop(ROW, i)
					{
						template_loop(MAT_COL, j)
						{
							template_loop(COL, k)
							{
								result.val[i * MAT_COL + j] += val[i * COL + k] * mat.val[k * MAT_COL + j];
							};
						};
					};
				}
				else if constexpr (ROW == MAT_COL)
				{
					result = mat * *this;
				}

				return result;
			}

			force_inline matrix_t<ROW, COL, T>& operator *= (const matrix_t<ROW, COL, T>& mat)
			{
				*this = *this * mat;
				return *this;
			}

			force_inline const matrix_t<ROW, COL, T> dot(const matrix_t<ROW, COL, T>& mat)
			{
				matrix_t<ROW, COL, T> result;
				template_loop(ROW, i)
				{
					template_loop(COL, j)
					{
						result.val[i * COL + j] = val[i * COL + j] * mat.val[i * COL + j];
					};
				};
				return result;
			}

			static force_inline matrix_t<ROW, COL, T> identity()
			{
				matrix_t<ROW, COL, T> result;

				constexpr uint32_t n = ROW <= COL ? ROW : COL;

				template_loop(n, i)
				{
					result.val[i * COL + i] = (T)1;
				};
				return result;
			}

			template <uint32_t DIAG_SIZE = ROW <= COL ? ROW : COL, typename = std::enable_if<(DIAG_SIZE < ROW && DIAG_SIZE < COL), void>>
			force_inline vector_t<DIAG_SIZE, T> diag()
			{
				vector_t<DIAG_SIZE, T> result;

				template_loop(DIAG_SIZE, i)
				{
					result(i) = (*this)(i, i);
				};

				return result;
			}

			template <uint32_t MAT_ROW, uint32_t MAT_COL, typename = std::enable_if<ROW == MAT_ROW, void>>
			force_inline const matrix_t<MAT_ROW, COL + MAT_COL, T> horizontal_concat(matrix_t<MAT_ROW, MAT_COL, T>& mat)
			{
				matrix_t<MAT_ROW, COL + MAT_COL, T> result;
				template_loop(ROW, i)
				{
					template_loop(COL, j)
					{
						result.set(i, j, get(i, j));
					};

					template_loop(MAT_COL, j)
					{
						result.set(i, COL + j, mat.get(i, j));
					};
				};
				return result;
			}

			template <uint32_t MAT_ROW, uint32_t MAT_COL, typename = std::enable_if<COL == MAT_COL, void>>
			force_inline const matrix_t<ROW + MAT_ROW, MAT_COL, T> vertical_concat(matrix_t<MAT_ROW, MAT_COL, T>& mat)
			{
				matrix_t<ROW + MAT_ROW, MAT_COL, T> result;
				template_loop(COL, i)
				{
					template_loop(ROW, j)
					{
						result.set(j, i, get(j, i));
					};

					template_loop(MAT_ROW, j)
					{
						result.set(ROW + j, i, mat.get(j, i));
					};
				};
				return result;
			}

			template <uint32_t VEC_N, typename = std::enable_if<ROW == VEC_N, void>>
			force_inline const matrix_t<ROW, COL + 1, T> horizontal_concat(vector_t<VEC_N, T>& vec)
			{
				matrix_t<ROW, COL + 1, T> result;
				template_loop(ROW, i)
				{
					template_loop(COL, j)
					{
						result.set(i, j, get(i, j));
					};
				};
				template_loop(ROW, i)
				{
					result.set(i, COL, vec.get(i));
				};
				return result;
			}

			template <uint32_t VEC_N, typename = std::enable_if<COL == VEC_N, void>>
			force_inline const matrix_t<ROW + 1, COL, T> vertical_concat(vector_t<VEC_N, T>& vec)
			{
				matrix_t<ROW + 1, COL, T> result;
				template_loop(ROW, i)
				{
					template_loop(COL, j)
					{
						result.set(i, j, get(i, j));
					};
				};
				template_loop(COL, i)
				{
					result.set(ROW, i, vec.get(i));
				};
				return result;
			}

			template <uint32_t MAT_ROW, uint32_t MAT_COL>
			force_inline void fill(uint32_t offset_row, uint32_t offset_col, const matrix_t<MAT_ROW, MAT_COL, T>& mat)
			{
				template_loop(MAT_ROW, i)
				{
					template_loop(MAT_COL, j)
					{
						val[(offset_row + i) * COL + offset_col + j] = mat.val[i * MAT_COL + j];
					};
				};
			}

			template <uint32_t MAT_ROW, uint32_t MAT_COL>
			force_inline matrix_t<MAT_ROW, MAT_COL, T> sub_matrix(uint32_t offset_row = 0.f, uint32_t offset_col = 0.f)
			{
				matrix_t<MAT_ROW, MAT_COL, T> result;

				template_loop(MAT_ROW, i)
				{
					template_loop(MAT_COL, j)
					{
						//if (((offset_row + i) < ROW) && ((offset_col + j) < COL))
						//{
							result(i, j) = (*this)(offset_row + i, offset_col + j);
						//}
					};
				};

				return result;
			}
		};

		template <typename T = MATRIX_DEFAULT_FLOAT_TYPE>
		using matrix11_t = matrix_t<1, 1, T>;

		template <typename T = MATRIX_DEFAULT_FLOAT_TYPE>
		using matrix21_t = matrix_t<2, 1, T>;
		template <typename T = MATRIX_DEFAULT_FLOAT_TYPE>
		using matrix12_t = matrix_t<1, 2, T>;
		template <typename T = MATRIX_DEFAULT_FLOAT_TYPE>
		using matrix22_t = matrix_t<2, 2, T>;

		template <typename T = MATRIX_DEFAULT_FLOAT_TYPE>
		using matrix13_t = matrix_t<1, 3, T>;
		template <typename T = MATRIX_DEFAULT_FLOAT_TYPE>
		using matrix31_t = matrix_t<3, 1, T>;
		template <typename T = MATRIX_DEFAULT_FLOAT_TYPE>
		using matrix32_t = matrix_t<3, 2, T>;
		template <typename T = MATRIX_DEFAULT_FLOAT_TYPE>
		using matrix23_t = matrix_t<2, 3, T>;
		template <typename T = MATRIX_DEFAULT_FLOAT_TYPE>
		using matrix33_t = matrix_t<3, 3, T>;

		template <typename T = MATRIX_DEFAULT_FLOAT_TYPE>
		using matrix24_t = matrix_t<2, 4, T>;
		template <typename T = MATRIX_DEFAULT_FLOAT_TYPE>
		using matrix42_t = matrix_t<4, 2, T>;
		template <typename T = MATRIX_DEFAULT_FLOAT_TYPE>
		using matrix34_t = matrix_t<3, 4, T>;
		template <typename T = MATRIX_DEFAULT_FLOAT_TYPE>
		using matrix43_t = matrix_t<4, 3, T>;
		template <typename T = MATRIX_DEFAULT_FLOAT_TYPE>
		using matrix44_t = matrix_t<4, 4, T>;

		typedef matrix11_t<MATRIX_DEFAULT_FLOAT_TYPE> matrix11f_t;

		typedef matrix21_t<MATRIX_DEFAULT_FLOAT_TYPE> matrix21f_t;
		typedef matrix12_t<MATRIX_DEFAULT_FLOAT_TYPE> matrix12f_t;
		typedef matrix22_t<MATRIX_DEFAULT_FLOAT_TYPE> matrix22f_t;

		typedef matrix13_t<MATRIX_DEFAULT_FLOAT_TYPE> matrix13f_t;
		typedef matrix31_t<MATRIX_DEFAULT_FLOAT_TYPE> matrix31f_t;
		typedef matrix23_t<MATRIX_DEFAULT_FLOAT_TYPE> matrix23f_t;
		typedef matrix32_t<MATRIX_DEFAULT_FLOAT_TYPE> matrix32f_t;
		typedef matrix33_t<MATRIX_DEFAULT_FLOAT_TYPE> matrix33f_t;

		typedef matrix24_t<MATRIX_DEFAULT_FLOAT_TYPE> matrix24f_t;
		typedef matrix42_t<MATRIX_DEFAULT_FLOAT_TYPE> matrix42f_t;
		typedef matrix34_t<MATRIX_DEFAULT_FLOAT_TYPE> matrix34f_t;
		typedef matrix43_t<MATRIX_DEFAULT_FLOAT_TYPE> matrix43f_t;
		typedef matrix44_t<MATRIX_DEFAULT_FLOAT_TYPE> matrix44f_t;

	}
}

#endif
