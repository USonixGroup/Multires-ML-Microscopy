function [Jp, Hp] = jacobianHessian(Ja, Ha, refPoint)
% Compute Jacobian and Hessian of a point given the Jacobian and Hessian coefficients.

% [1] Martin Magnusson, The Three-Dimensional Normal-Distributions
% Transform - an Efficient Representation for Registration, Surface
% Analysis, and Loop Detection, Thesis, 2013

% $matlabroot/toolbox/vision/builtins/src/vision/include/ndtUtils.hpp

%   Copyright 2021 The MathWorks, Inc.

%#codegen

% The Jacobian coefficients as given by [1] Eq 6.19
Jp = zeros(3,6,'like',Ja);
Jp_coeff = Ja(:,:,1)*refPoint(1) + Ja(:,:,2)*refPoint(2) + Ja(:,:,3)*refPoint(3);
Jp(1,1) = 1;
Jp(2,2) = 1;
Jp(3,3) = 1;
for i = 1:9
    Jp(i+9) = Jp_coeff(i);
end

Hp = zeros(6, 6, 3, 'like', Ha);
Hp_coeff = zeros(6, 3, 'like', Ha);

for i = 1:6
    %Hp_coeff(i,:) = Ha(:,:,i)*[refPoint(1); refPoint(2); refPoint(3)];
    Hp_coeff(i,:) = Ha(:,:,i) * refPoint(:);
end

% The Hessian coefficients as given by [1] Eq 6.21
for ch = 1:3
    % a
    Hp(4,4,ch) = Hp_coeff(1,ch);
    % d
    Hp(5,5,ch) = Hp_coeff(4,ch);
    % f
    Hp(6,6,ch) = Hp_coeff(6,ch);
    
    % b
    Hp(4,5,ch) = Hp_coeff(2,ch);
    Hp(5,4,ch) = Hp_coeff(2,ch);
    % c
    Hp(4,6,ch) = Hp_coeff(3,ch);
    Hp(6,4,ch) = Hp_coeff(3,ch);
    % e
    Hp(5,6,ch) = Hp_coeff(5,ch);
    Hp(6,5,ch) = Hp_coeff(5,ch);
end

end
