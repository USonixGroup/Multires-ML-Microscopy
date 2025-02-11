////////////////////////////////////////////////////////////////////////////////
//
// Utilities of matrix operation 
//
////////////////////////////////////////////////////////////////////////////////

#ifndef TMW_MATRIXUTILS_HPP
#define TMW_MATRIXUTILS_HPP

#include <cmath>

namespace vision
{
	// helper function to transpose a matrix.
	// The matrix assumes a row major memory layout.
	template <typename T>
	void matrixTranspose(const T* A, const int m, const int n, T* B)
	{
		for (int i = 0; i < m; i++) {
			for (int j = 0; j < n; j++) {
				B[j * m + i] = A[i * n + j];
			}
		}
	}

	// helper function to multiply two matrices.
	// The matrix assumes a row major memory layout.
	template <typename T>
	void matrixMultiply(const T* A, const int m, const int n, const T* B, const int k, T* C)
	{
		for (int i = 0; i < m; i++) {
			for (int j = 0; j < k; j++) {
				C[i * k + j] = 0;
			}
		}

		for (int i = 0; i < m; i++){		//row of first matrix
			for (int j = 0; j < k; j++){	//column of second matrix
				T sum = 0;
				for (int d = 0; d < n; d++) {
					sum += A[i * n + d] * B[d * k + j];
				}
				C[i * k + j] = sum;
			}
		}
	}

	// helper function to compute A'*A.
	// The matrix assumes a row major memory layout.
	template <typename T>
	void matrixMultiplyAtA(const T* A, const int m, const int n, T* C)
	{
		for (int i = 0; i < n; i++) {
			for (int j = 0; j < n; j++) {
				C[i * n + j] = 0;
			}
		}

		for (int i = 0; i < n; i++) { //column of first matrix
			for (int j = 0; j <= i; j++) { //column of second matrix
				T sum = 0;
				for (int d = 0; d < m; d++) {
					sum += A[d * n + i] * A[d * n + j];
				}
				C[i * n + j] = sum;
			}
		}
	
		for (int i = 0; i < n; i++) { //column of first matrix
			for (int j = i + 1; j < n; j++) { //column of second matrix
				C[i * n + j] = C[j * n + i];
			}
		}
	}

	// helper function to compute A*B.
	// The matrix assumes a column major memory layout.
	template <typename T>
	void matrixMultiplyColumn(const T* A, const int m, const int n, const T* B, const int k, T* C)
	{
		for (int i = 0; i < m; i++) {
			for (int j = 0; j < k; j++) {
				C[j * m + i] = 0;
			}
		}

		for (int i = 0; i < m; i++){ //row of first matrix
			for (int j = 0; j < k; j++){  //column of second matrix
				T sum = 0;
				for (int d = 0; d < n; d++) {
					sum += A[d * m + i] * B[j * n + d];
				}
				C[j * m + i] = sum;
			}
		}
	}
}

#endif // TMW_MATRIXUTILS_HPP
