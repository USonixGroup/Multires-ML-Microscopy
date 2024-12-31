function [a_l, f_l, g_l, a_u, f_u, g_u, isClosed] = updateInterval(a_l, ...
    f_l, g_l, a_u, f_u, g_u, a_t, f_t, g_t)
% updateInterval update interval given a trial value and two endpoints.
% The function follows the method by More and Thuente.
%
% [1] More and Thuente, Line Search Algorithm with Guaranteed Sufficient
% Decrease, ACM Trans. on Mathematical Software, Vol 20, No. 3, 1994

% Copyright 2017-2019 The MathWorks, Inc.
%#codegen

isClosed = false;

if (f_t > f_l)
    % Case 1 in Update Algorithm / Modified Update Algorithm [More, Thuente 1994]
    a_u = a_t;
    f_u = f_t;
	g_u = g_t;
    
elseif (g_t * (a_l - a_t) > 0)
    % Case 2 in Update Algorithm / Modified Update Algorithm [More, Thuente 1994]
    a_l = a_t;
    f_l = f_t;
	g_l = g_t;
    
elseif (g_t * (a_l - a_t) < 0)
    % Case 3 in Update Algorithm / Modified Update Algorithm [More, Thuente 1994]    
    a_u = a_l;
	f_u = f_l;
	g_u = g_l;

	a_l = a_t;
	f_l = f_t;
	g_l = g_t;
    
else
    % Interval Converged
    isClosed = true;
end
