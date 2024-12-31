function  [P, D, status] = eig3(A)
% Solve the eigenvalues and normalized eigenvectors of a symmetric 
% real 3x3 matrix A using the QL algorithm with implicit shifts, preceded 
% by a Householder step.
%
%   Inputs:
%   -------
%   A      : 3x3 input matrix

%   Outputs:
%   -------
%   D     : buffer for eigenvalues
%   P     : buffer for eigenvectors
%
% Reference:
% John Mathews, "Numerical methods for Mathematics, Science, 
% and Engineering, 2nd Edition".
% http://beige.ucs.indiana.edu/B673/node38.html

% Copyright 2021 The MathWorks, Inc.
%#codegen

    % input matrix dimension
    n = int32(3);
    % allocate buffers for eigenvalues & eigenvectors
    D = coder.nullcopy(zeros(n, 1, 'like', A));
    P = coder.nullcopy(zeros(n, 1, 'like', A));

    % off-diagonal elements
    E = coder.nullcopy(zeros(n, 1, 'like', A));
    % intermediate variables defined in 
    % http://beige.ucs.indiana.edu/B673/node38.html

    % This is designed for 3x3 symmetric real matrix
    % Transform A to tridiagonal form.
    [D, E, P] = householder(A);

    % Solve eigen problem for a real symmetric tridiagonal matrix 
    % with the QL method.
    % The algorithm produces eigenvalues in the order of diminishing absolute value
    for l = 0 : n-2
        iterations = int32(0);
        while true
            % Split the submatrix on a very small subdiagonal element e[m]
            % or skip if element e[l] is already zero
            m = int32(l);
            while(m < n-1)
                g = abs(D(m + 1)) + abs(D(m + 1 + 1));
                if ( (abs(E(m + 1)) + g) == g )
                    break;
                end
                m = m + 1;
            end
            if(m == l)
                break;
            end
            iterations = iterations + 1;
            if (iterations >= 30)
                status = -1; %ES_NOT_CONVERGE
                return;
            end
            % g = d[m] - k_s
            g = (D(l + 1 + 1) - D(l + 1)) / (E(l + 1) + E(l + 1));
            r = sqrt((g * g) + 1);
            if (g > 0)
                g = D(m + 1) - D(l + 1) + E(l + 1) / (g + r);
            else
                g = D(m + 1) - D(l + 1) + E(l + 1) / (g - r);
            end
            s = cast(1,'like', A);
            c = cast(1,'like', A);
            p = cast(0,'like', A);
            for i = m-1: -1: l
                % Jacobi rotation
                f = s * E(i + 1);
                b = c * E(i + 1);

                % Make sure the SQUARE and division is safe
                % Equivalent to :
                % r = sqrt(f^2+g^2);
                % e[i+1] = r;
                % s = f / r, c = g / r;
                if (abs(f) > abs(g))
                    c = g / f;
                    r = sqrt((c * c) + 1);
                    E(i + 1 + 1) = f * r;
                    s = 1 / r;
                    c = c * s;
                else
                    s = f / g;
                    r = sqrt((s * s) + 1);
                    E(i + 1 + 1) = g * r;
                    c = 1 / r;
                    s = s * c;
                end
                g = D(i + 1 + 1) - p;
                r = (D(i + 1) - g) * s + 2 * c * b;
                p = s * r;
                D(i + 1 + 1) = g + p;
                g = c * r - b;

                % Return eigenvectors
                for k = 1 : n
                    t = P(k + (i + 1) * n );
                    P(k + (i + 1) * n) = s * P(k + i * n) + c * t;
                    P(k + i * n) = c * P(k + i * n) - s * t;
                end
            end
            D(l + 1) = D(l + 1) - p;
            E(l + 1) = g;
            E(m + 1) = 0;
        end
    end

    status = 1; %ES_SUCCESS
end


function  [D, E, P] = householder(A)
% Reduce a real symmetric matrix to tridiagonal form 
% by applying Householder transformation:
%             [ d[0]  e[0]       ]
%     A = P . [ e[0]  d[1]  e[1] ] . P^T
%             [       e[1]  d[2] ]
%  Only one iteration is performed so A is assumed to be 3x3.
%
%   Inputs:
%   -------
%    A                  : 3x3 input matrix

%   Outputs:
%   -------
%    D  : buffer for diagonal elements
%    E  : buffer for upper/lower diagonal elements
%    P  : transformation matrix. Skip computation for P if this is NULL.
%
% Reference:
% John Mathews, "Numerical methods for Mathematics, Science, 
% and Engineering, 2nd Edition", p574.

% Copyright 2021 The MathWorks, Inc.
%#codegen

    % This is designed for 3x3 symmetric real matrix
    n = int32(3);

    W = coder.nullcopy(zeros(n, 1, 'like', A));
    Q = coder.nullcopy(zeros(n, 1, 'like', A));
    V = coder.nullcopy(zeros(n, 1, 'like', A));

    D = coder.nullcopy(zeros(n, 1, 'like', A));
    E = coder.nullcopy(zeros(n, 1, 'like', A));

    % Initialize P to the identitity
    P = eye(n);
    P = P(:);

    sqsum = (A(n +1) * A(n +1)) + (A(n * 2 +1) * A(n * 2 + 1));
    if(A(n +1) > 0)
        S = -sqrt(sqsum);
    else
        S = sqrt(sqsum);
    end
    % R^2 = 2 * r
    r = sqsum - S * A(n + 1);

    % Unnormalized vector W, s.t, P = I - 2*W*W^T (if normalized)
    % normalization term R: 2*sqrt(0.5*(S^2+A[1]*S)) = sqrt(2*r)
    W(2) = A(n + 1) - S;
    W(3) = A(n * 2 + 1);
    E(1)= S;
    D(1) = A(1);

    if(r>0)
        % Theorem 11.24 in "Numerical methods for Mathematics, Science, and Engineering, 2nd Edition"
        r = 1 / r;
        c = cast(0, 'like', W);
        for i = 1 : n-1
            aw = A(i*n + 1 + 1) * W(2) + A(2 * n + i + 1) * W(3);
            V(i + 1) = r * aw; % V = A*W
            c = c + W(i + 1) * aw; % c = W^T*V
        end
        if (c ~= 0)
            c = (r * r) / 2 * c;
            % Q = V - c*W
            for i = 1:n-1
                Q(i + 1) = V(i + 1) - c * W(i + 1);
            end
            % PAP = A - 2*W*Q^T - 2*Q*W^T
            % Note W and Q are unnormalized version here.
            D(2) = A(1 + n + 1)  - 2 * Q(2) * W(2);
            D(3) = A(2 + 2*n + 1) - 2 * Q(3) * W(3);

            % Store inverse Householder transformation: P = I - W * W^T
            for j = 1 : n - 1
                w = r * W(j + 1);
                for i = 1 : n - 1
                    P(j * n + i + 1) = P(j * n + i + 1) - w * W(i + 1);
                end
            end

            E(2) = A(n * 2 + 1 +1) - Q(2) * W(3) - W(2) * Q(3);
        else
            for i = 1 : n - 1
                D(i + 1) = A(i + n * i + 1);
            end
            E(2) = A(n * 2 + 1 + 1);
        end
    else
        for i = 1 : n - 1
            D(i + 1) = A(i + n*i + 1);
        end

        E(2) = A(n * 2 + 1 + 1);
    end

end
