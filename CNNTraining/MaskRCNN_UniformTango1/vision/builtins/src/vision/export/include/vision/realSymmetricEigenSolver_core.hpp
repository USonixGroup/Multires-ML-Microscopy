////////////////////////////////////////////////////////////////////////////////
//
//  3x3 Real Symmetric Eigen solver with Householder transform and QL method
//
// John Mathews, "Numerical methods for Mathematics, Science, and Engineering, 2nd Edition"
// http://en.wikipedia.org/wiki/Householder_transformation
// http://beige.ucs.indiana.edu/B673/node38.html
////////////////////////////////////////////////////////////////////////////////
#ifndef REALSYMMETRICEIGENSOLVER_CORE_HPP
#define REALSYMMETRICEIGENSOLVER_CORE_HPP

#include <cmath>
#include <cstddef> // Scope for NULL for portable codegen
namespace vision
{
	enum EigenSolverCode { ES_SUCCESS, ES_NOT_CONVERGE };

	//========================================================================================
	// Reduce a real symmetric matrix to tridiagonal form by applying Householder 
    // transformation:
	//            [ d[0]  e[0]       ]
	//    A = P . [ e[0]  d[1]  e[1] ] . P^T
	//            [       e[1]  d[2] ]
	// Only one iteration is performed so A is assumed to be 3x3!!!
	//
	// All input/output are allocated by caller. Some steps can be skipped when P 
	// is empty. This is typically when eigen vector is not needed.
	//
	// Parameters:
	//   A          	: 3x3 input matrix
	//   D				: buffer for diagonal elements (allocated by caller)
	//   E				: buffer for upper/lower diagonal elements (allocated by caller)
	//   P              : transformation matrix. Skip computation for P if this is NULL. (allocated by caller)
	//
	// Reference:
	// John Mathews, "Numerical methods for Mathematics, Science, and Engineering, 2nd Edition", p574
	//========================================================================================
	template <typename T>
	void householder(T* A, T* D, T* E, T* P)
	{
		// This is designed for 3x3 symmetric real matrix
		const int n = 3;
		T W[n], Q[n], V[n];
		T sqsum, S, r, c, aw, w;

		if (P != NULL) {
			// Initialize P to the identitity
			for (int i = 0; i < n; i++)	{
				P[i * n + i] = 1;
				for (int j = 0; j < i; j++) {
					P[i * n + j] = 0;
					P[j * n + i] = 0;
				}
			}
		}

		sqsum = (A[n] * A[n]) + (A[n * 2] * A[n * 2]);
		S = A[n] > 0 ? -sqrt(sqsum) : sqrt(sqsum);
		// R^2 = 2 * r
		r = sqsum - S * A[n];

		// Unnormalized vector W, s.t, P = I - 2*W*W^T (if normalized)
		// normalization term R: 2*sqrt(0.5*(S^2+A[1]*S)) = sqrt(2*r)
		W[1] = A[n] - S;
		W[2] = A[n * 2];

		E[0] = S;
		D[0] = A[0];

		if (r > 0) { // numerically safe
			// Theorem 11.24 in "Numerical methods for Mathematics, Science, and Engineering, 2nd Edition"
			r = 1 / r;
			c = 0;
			for (int i = 1; i < n; i++) {
				aw = A[i * n + 1] * W[1] + A[2 * n + i] * W[2];
				V[i] = r * aw;						// V = A*W
				c += W[i] * aw;						// c = W^T*V
			}

			if (c != 0) {
				c = (r * r) / 2 * c;				

				// Q = V - c*W
				for (int i = 1; i < n; i++)				
					Q[i] = V[i] - c * W[i];

				// PAP = A - 2*W*Q^T - 2*Q*W^T
				// Note W and Q are unnormalized version here.
				D[1] = A[1 + n] - 2 * Q[1] * W[1];      
				D[2] = A[2 + 2 * n] - 2 * Q[2] * W[2];

				// Store inverse Householder transformation: P = I - W * W^T
				if (P != NULL) {
					for (int j = 1; j < n; j++)	{
						w = r * W[j];	
						for (int i = 1; i < n; i++)
							P[j * n + i] = P[j * n + i] - w * W[i];
					}
				}

				E[1] = A[n * 2 + 1] - Q[1] * W[2] - W[1] * Q[2];
			}
			else {
				for (int i = 1; i < n; i++)
					D[i] = A[i + n * i];
				E[1] = A[n * 2 + 1];
			}
		}
		else {
			for (int i = 1; i < n; i++)
				D[i] = A[i + n * i];
			E[1] = A[n * 2 + 1];
		}
	}

	//========================================================================================
	// Solve the eigenvalues and normalized eigenvectors of a symmetric real 3x3 matrix A
	// using the QL algorithm with implicit shifts, preceded by a Householder step
	// 
	// Parameters:
	//   A	   : 3x3 input matrix
	//   D     : buffer for eigenvalues (allocated by caller)
	//   P	   : buffer for eigenvectors (allocated by caller, NULL if eigen vector is not needed)
	//
	// Reference:
	// John Mathews, "Numerical methods for Mathematics, Science, and Engineering, 2nd Edition"
	// http://beige.ucs.indiana.edu/B673/node38.html
	//========================================================================================
	template <typename T>
	EigenSolverCode eig3(T* A, T* D, T* P)
	{
		const int n = 3;
		T E[n];						// off-diagonal elements
		T g, r, p, f, b, s, c, t;	// intermediate variables defined in http://beige.ucs.indiana.edu/B673/node38.html
	
		// This is designed for 3x3 symmetric real matrix

		// Transform A to tridiagonal form
		householder(A, D, E, P);

		// Solve eigen problem for a real symmetric tridiagonal matrix with the QL method
		// The algorithm produces eigenvalues in the order of diminishing absolute value
		for (int l = 0; l < n - 1; l++)	{
			int iterations = 0;
			while (true) {
				// Split the submatrix on a very small subdiagonal element e[m]
				// or skip if element e[l] is already zero
				int m;
				for (m = l; m < n - 1; m++) {
					g = fabs(D[m]) + fabs(D[m + 1]);
					if (fabs(E[m]) + g == g)
						break;
				}
				if (m == l)
					break;

				iterations++;
				if (iterations >= 30)
					return ES_NOT_CONVERGE;

				// g = d[m] - k_s
				g = (D[l + 1] - D[l]) / (E[l] + E[l]);
				r = sqrt((g * g) + 1);
				if (g > 0)
					g = D[m] - D[l] + E[l] / (g + r);
				else
					g = D[m] - D[l] + E[l] / (g - r);

				s = c = 1;
				p = 0;
				for (int i = m - 1; i >= l; i--) {
					// Jacobi rotation
					f = s * E[i];
					b = c * E[i];

					// Make sure the SQUARE and division is safe
					// Equivalent to :
					// r = sqrt(f^2+g^2);
					// e[i+1] = r;
					// s = f / r, c = g / r;
					if (fabs(f) > fabs(g)) {
						c = g / f;
						r = sqrt((c * c) + 1);
						E[i + 1] = f * r;
						s = 1 / r;
						c = c * s ;
					}
					else {
						s = f / g;
						r = sqrt((s * s) + 1);
						E[i + 1] = g * r;
						c = 1 / r;
						s = s * c;
					}

					g = D[i + 1] - p;
					r = (D[i] - g) * s + 2 * c * b;
					p = s * r;
					D[i + 1] = g + p;
					g = c * r - b;

					// Return eigenvectors
					if (P != NULL) {
						for (int k = 0; k < n; k++)	{
							t = P[k + (i + 1) * n];
							P[k + (i + 1) * n] = s * P[k + i * n] + c * t;
							P[k + i * n] = c * P[k + i * n] - s * t;
						}
					}
				}
				
				D[l] -= p;
				E[l] = g;
				E[m] = 0;
			}
		}

		return ES_SUCCESS;
	}

	//========================================================================================
	// Invert a symmetric real 3x3 matrix A using eigen decomposition:
	// [V,D] = eig(A); A_inv = V * (1./D) * V';
	// Note for practical reason, the smallest eigen value cannot be zero in this case.
	// The function truncates those eigenvalues here to return an approximation.
	// 
	// Parameters:
	//   A	   : 3x3 input matrix
	//   B     : 3x3 output matrix. allocated by the caller.
	//
	// Reference:
	// https://en.wikipedia.org/wiki/Eigendecomposition_of_a_matrix
	//========================================================================================
	template <typename T>
	EigenSolverCode syminv3(T* A, T* B)
	{
		const int n = 3;
		T D[n], V[n*n], Vt[n*n];
		EigenSolverCode retCode;

		retCode = eig3(A, D, V); // 3 column vectors in V.
		// Note, although eig3 assumes column-major layout, it has no effect on the output.
		// The inverse of a symmetric matrix is also symmetric.

		T smallValue = 1e-5;

		for (int j = 0; j < n; ++j) {
			if (fabs(D[j]) <= smallValue) {
				D[j] = 0;
			}
			else {
				D[j] = 1 / D[j];
			}
		}

		for (int j = 0; j < n; ++j) {
			for (int k = 0; k < n; ++k) {
				Vt[j * n + k] = V[k * n + j];
			}
		}

		for (int j = 0; j < n; ++j) {
			Vt[j * n] = Vt[j * n] * D[0];
			Vt[j * n + 1] = Vt[j * n + 1] * D[1];
			Vt[j * n + 2] = Vt[j * n + 2] * D[2];
		}

		// B = Vt*V;

		for (int i = 0; i < n; ++i) {		// row of first matrix
			for (int j = i; j < n; ++j) {	// column of second matrix
				T sum = 0;
				for (int d = 0; d < n; d++) {
					sum += (Vt[i * n + d] * V[d * n + j]);
				}
				B[i * n + j] = sum;
			}
		}

		// Fill out the lower triangle part
		for (int i = 0; i < n; ++i) {		// row of first matrix
			for (int j = 0; j < i; ++j) {	// column of second matrix
				B[i * n + j] = B[j * n + i];
			}
		}

		return retCode;
	}
}

#endif
