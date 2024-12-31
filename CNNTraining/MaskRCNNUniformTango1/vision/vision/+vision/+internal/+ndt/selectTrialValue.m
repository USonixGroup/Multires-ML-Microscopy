function c = selectTrialValue(a_l, f_l, g_l, a_u, f_u, g_u, a_t, f_t, g_t)
% selectTrialValue select trial value a_tprime given the endpoints a_l,
% a_u, old trial value a_t, their function values and their gradients.
% The function follows the method by More and Thuente.
%
% [1] More and Thuente, Line Search Algorithm with Guaranteed Sufficient
% Decrease, ACM Trans. on Mathematical Software, Vol 20, No. 3, 1994
%
% [2] Wenyu Song, Ya-Xiang Yuan, Optimization Theory and Methods - Nonlinear Programming, 2006

% Copyright 2017-2019 The MathWorks, Inc.
%#codegen

if f_t > f_l
    % Case 1 in Trial Value Selection [More, Thuente 1994]
    
    % Calculate the minimizer of the cubic that interpolates f_l, f_t, g_l and
    % g_t. Equation 2.4.52 [Sun, Yuan 2006]
    a_c = cubicInterp(a_t, f_t, g_t, a_l, f_l, g_l);
    
    % Calculate the minimizer of the quadratic that interpolates f_l, f_t and 
    % g_l. Equation 2.4.2 [Sun, Yuan 2006]
    a_q = quadInterp2(a_l, a_t, f_l, f_t, g_l);
    
    if abs(a_c - a_l) < abs(a_q - a_l)
        c = a_c;
    else
        c = 0.5 * (a_q + a_c);
    end
    
elseif ( g_t * g_l < 0 )
    % Case 2 in Trial Value Selection [More, Thuente 1994]
    
    % Calculate the minimizer of the cubic that interpolates f_l, f_t, g_l and 
    % g_t. Equation 2.4.52 [Sun, Yuan 2006]
    a_c = cubicInterp(a_t, f_t, g_t, a_l, f_l, g_l);
    
    % Calculate the minimizer of the quadratic that interpolates f_l, g_l and 
    % g_t. Equation 2.4.5 [Sun, Yuan 2006]
    a_s = quadInterp1(a_l, a_t, g_l, g_t);
    
    if abs(a_c - a_t) >= abs(a_s - a_t)
        c = a_c;
    else
        c = a_s;
    end
    
elseif ( abs(g_t) <= abs (g_l) )
    % Case 3 in Trial Value Selection [More, Thuente 1994]
    
    % Calculate the minimizer of the cubic that interpolates f_l, f_t, g_l and 
    % g_t. Equation 2.4.52 [Sun, Yuan 2006]
    a_c = cubicInterp(a_t, f_t, g_t, a_l, f_l, g_l);
    
    % Calculate the minimizer of the quadratic that interpolates g_l and g_t
    % Equation 2.4.5 [Sun, Yuan 2006]
    a_s = quadInterp1(a_l, a_t, g_l, g_t);
    
    if abs(a_c - a_t) < abs (a_s - a_t)
        a_t_next = a_c;
    else
        a_t_next = a_s;
    end
    
    if a_t > a_l
        c = min (a_t + 0.66 * (a_u - a_t), a_t_next);
    else
        c = max (a_t + 0.66 * (a_u - a_t), a_t_next);
    end
    
else
    % Case 4 in Trial Value Selection [More, Thuente 1994]
    
    % Calculate the minimizer of the cubic that interpolates f_u, f_t, g_u and 
    % g_t. Equation 2.4.52 [Sun, Yuan 2006]
    c = cubicInterp(a_t, f_t, g_t, a_u, f_u, g_u);
end

%==========================================================================
% Calculate the minimizer of the cubic that interpolates fa, fb, ga and 
% gb. Equation 2.4.52 [Sun, Yuan 2006]
function c = cubicInterp(a, fa, ga, b, fb, gb)
z = 3 * (fa - fb) / (a - b) - ga - gb;
w = sqrt(z * z - ga * gb);

c = b + (a - b) * (w - gb - z) / (ga - gb + 2 * w);

%==========================================================================
% Calculate the minimizer of the quadratic that interpolates g_l and g_t
% Equation 2.4.5 [Sun, Yuan 2006]
function c = quadInterp1(a, b, ga, gb)
c = a - (a - b) / (ga - gb) * ga;

%==========================================================================
% Calculate the minimizer of the quadratic that interpolates f_l, f_t and 
% g_l. Equation 2.4.2 [Sun, Yuan 2006]
function c = quadInterp2(a, b, fa, fb, ga)
c = a - 0.5 * (a - b) * ga / (ga - (fa - fb) / (a - b));
