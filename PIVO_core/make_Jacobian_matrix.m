clc;
close all;
clear variables; %clear classes;
rand('state',0); % rand('state',sum(100*clock));
dbstop if error;


%% make symbolic variables


syms fx fy cx cy X Y Z
syms r11 r21 r31 r12 r22 r32 r13 r23 r33 tx ty tz


% rigid body transformation matrix in SE(3)
T_21 = [r11 r12 r13 tx;
    r21 r22 r23 ty;
    r31 r32 r33 tz;
    0   0    0  1];


% basis of Lie algebra se(3)
E1 = zeros(4);  % translational part v1 (xi1) in se(3)
E1(1,4) = 1;

E2 = zeros(4);  % translational part v2 (xi2) in se(3)
E2(2,4) = 1;

E3 = zeros(4);  % translational part v3 (xi3) in se(3)
E3(3,4) = 1;

E4 = zeros(4);  % rotational part w1 (xi4) in se(3)
E4(3,2) = 1;
E4(2,3) = -1;

E5 = zeros(4);  % rotational part w2 (xi5) in se(3)
E5(3,1) = -1;
E5(1,3) = 1;

E6 = zeros(4);  % rotational part w3 (xi6) in se(3)
E6(2,1) = 1;
E6(1,2) = -1;


% basis of Lie algebra se(3) with 3-dimensional matrix
E_se3_basis(:,:,1) = E1;
E_se3_basis(:,:,2) = E2;
E_se3_basis(:,:,3) = E3;
E_se3_basis(:,:,4) = E4;
E_se3_basis(:,:,5) = E5;
E_se3_basis(:,:,6) = E6;


%%  calculate Jacobian matrix


% convert the point from P1 to P2
P1 = [X;Y;Z;1];
P2 = T_21 * P1;


% warping function results
uhat = fx*(P2(1)/P2(3)) + cx;
vhat = fy*(P2(2)/P2(3)) + cy;


% Jacobian of rigid body transformation matrix in SE(3)
dTdv1 = E1 * T_21;
dTdv1 = dTdv1(1:3,:);

dTdv2 = E2 * T_21;
dTdv2 = dTdv2(1:3,:);

dTdv3 = E3 * T_21;
dTdv3 = dTdv3(1:3,:);

dTdw1 = E4 * T_21;
dTdw1 = dTdw1(1:3,:);

dTdw2 = E5 * T_21;
dTdw2 = dTdw2(1:3,:);

dTdw3 = E6 * T_21;
dTdw3 = dTdw3(1:3,:);

J_G = [dTdv1(:) , dTdv2(:) , dTdv3(:) , dTdw1(:) , dTdw2(:) , dTdw3(:)];


% Jacobian of warping function ( [uhat ; vhat] )
duhatdT = jacobian(uhat,[r11 r21 r31 r12 r22 r32 r13 r23 r33 tx ty tz]);
duhatdxi = duhatdT * J_G;

dvhatdT = jacobian(vhat,[r11 r21 r31 r12 r22 r32 r13 r23 r33 tx ty tz]);
dvhatdxi = dvhatdT * J_G;


% make duhatdxi and dvhatdxi matlab functions
matlabFunction(duhatdxi,'file','duhat_dxi.m','vars',{[fx;fy;cx;cy;X;Y;Z;r11;r21;r31;r12;r22;r32;r13;r23;r33;tx;ty;tz]});
matlabFunction(dvhatdxi,'file','dvhat_dxi.m','vars',{[fx;fy;cx;cy;X;Y;Z;r11;r21;r31;r12;r22;r32;r13;r23;r33;tx;ty;tz]});



%% calculate Hessian matrix


% 1. Hu in feature-based approach
d2uhatdT2 = jacobian(duhatdT.',[r11 r21 r31 r12 r22 r32 r13 r23 r33 tx ty tz]);

for i=1:6
    for j=1:6
        % second-order derivatives of rigid body transformation matrix in SE(3)
        d2T_dxi_i_dxi_j = (1/2) * (E_se3_basis(:,:,i)*E_se3_basis(:,:,j) + E_se3_basis(:,:,j)*E_se3_basis(:,:,i)) * T_21;
        d2T_dxi_i_dxi_j = d2T_dxi_i_dxi_j(1:3,:);
        
        % Hessian matrix
        Hu(i,j) = J_G(:,j).' * d2uhatdT2 * J_G(:,i) + duhatdT * d2T_dxi_i_dxi_j(:);
    end
end



% 2. Hv in feature-based approach
d2vhatdT2 = jacobian(dvhatdT.',[r11 r21 r31 r12 r22 r32 r13 r23 r33 tx ty tz]);

for i=1:6
    for j=1:6
        % second-order derivatives of rigid body transformation matrix in SE(3)
        d2T_dxi_i_dxi_j = (1/2) * (E_se3_basis(:,:,i)*E_se3_basis(:,:,j) + E_se3_basis(:,:,j)*E_se3_basis(:,:,i)) * T_21;
        d2T_dxi_i_dxi_j = d2T_dxi_i_dxi_j(1:3,:);
        
        % Hessian matrix
        Hv(i,j) = J_G(:,j).' * d2vhatdT2 * J_G(:,i) + dvhatdT * d2T_dxi_i_dxi_j(:);
    end
end



% make Hu and Hv matlab functions
matlabFunction(Hu,'file','d2uhat_dxi2.m','vars',{[fx;fy;cx;cy;X;Y;Z;r11;r21;r31;r12;r22;r32;r13;r23;r33;tx;ty;tz]});
matlabFunction(Hv,'file','d2vhat_dxi2.m','vars',{[fx;fy;cx;cy;X;Y;Z;r11;r21;r31;r12;r22;r32;r13;r23;r33;tx;ty;tz]});



%% test the made functions


% camera intrinsic parameters
fx = 350;
fy = 350;
cx = 320;
cy = 240;


% current 3D position of point in {1} camera frame
X = 0.5;
Y = 0.5;
Z = 2;


% current T_21 (rigid body motion)
xi_temp = [0.05 0.07 0.04 0.15 0.11 0.13].';
T = LieAlgebra2LieGroup(xi_temp);

r11 = T(1,1);
r21 = T(2,1);
r31 = T(3,1);

r12 = T(1,2);
r22 = T(2,2);
r32 = T(3,2);

r13 = T(1,3);
r23 = T(2,3);
r33 = T(3,3);

tx = T(1,4);
ty = T(2,4);
tz = T(3,4);


tempInput = [fx;fy;cx;cy;X;Y;Z;r11;r21;r31;r12;r22;r32;r13;r23;r33;tx;ty;tz];


duhat_dxi(tempInput)
dvhat_dxi(tempInput)

d2uhat_dxi2(tempInput)
d2vhat_dxi2(tempInput)



































%% temporary part 1



%% make symbolic variables


syms fx fy cx cy X Y Z
syms tx ty tz phi theta psi


% rigid body transformation vector
translation = [tx; ty; tz];

Rx = [1 0 0;0 cos(phi) -sin(phi);0 sin(phi) cos(phi)];
Ry = [cos(theta) 0 sin(theta);0 1 0;-sin(theta) 0 cos(theta)];
Rz = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
rotation = [Rz * Ry * Rx].';



%%  calculate Jacobian & Hessian matrix


% convert the point from P1 to P2
P1 = [X;Y;Z];
P2 = rotation * P1 + translation;


% warping function results
uhat = fx*(P2(1)/P2(3)) + cx;
vhat = fy*(P2(2)/P2(3)) + cy;


% Jacobian of warping function ( [uhat ; vhat] )
duhatdx = jacobian(uhat,[tx ty tz phi theta psi]);
dvhatdx = jacobian(vhat,[tx ty tz phi theta psi]);


% make duhatdx and dvhatdx matlab functions
matlabFunction(duhatdx,'file','duhat_dx.m','vars',{[fx;fy;cx;cy;X;Y;Z;tx;ty;tz;phi;theta;psi]});
matlabFunction(dvhatdx,'file','dvhat_dx.m','vars',{[fx;fy;cx;cy;X;Y;Z;tx;ty;tz;phi;theta;psi]});



% Hessian matrix of warping function ( [uhat ; vhat] )
Hu = jacobian(duhatdx.',[tx ty tz phi theta psi]);
Hv = jacobian(dvhatdx.',[tx ty tz phi theta psi]);


% make Hu and Hv matlab functions
matlabFunction(Hu,'file','d2uhat_dx2.m','vars',{[fx;fy;cx;cy;X;Y;Z;tx;ty;tz;phi;theta;psi]});
matlabFunction(Hv,'file','d2vhat_dx2.m','vars',{[fx;fy;cx;cy;X;Y;Z;tx;ty;tz;phi;theta;psi]});



%% test the made functions


% camera intrinsic parameters
fx = 350;
fy = 350;
cx = 320;
cy = 240;


% current 3D position of point in {1} camera frame
X = 0.5;
Y = 0.5;
Z = 2;


% current T_21 (rigid body motion)
tx = 0.05;
ty = 0.07;
tz = 0.04;

phi = 0.15;
theta = 0.11;
psi = 0.13;


tempInput = [fx;fy;cx;cy;X;Y;Z;tx;ty;tz;phi;theta;psi];


duhat_dx(tempInput)
dvhat_dx(tempInput)

d2uhat_dx2(tempInput)
d2vhat_dx2(tempInput)

















%% temporary part 2


% 1. Hu in feature-based approach
Hu(1,1) = -(fx/(Zprm^2)) * (J_g(3,:) * J_G(:,1));
Hu(1,2) = -(fx/(Zprm^2)) * (J_g(3,:) * J_G(:,2));
Hu(1,3) = -(fx/(Zprm^2)) * (J_g(3,:) * J_G(:,3));
Hu(1,4) = -(fx/(Zprm^2)) * (J_g(3,:) * J_G(:,4));
Hu(1,5) = -(fx/(Zprm^2)) * (J_g(3,:) * J_G(:,5));
Hu(1,6) = -(fx/(Zprm^2)) * (J_g(3,:) * J_G(:,6));

Hu(2,1) = 0;
Hu(2,2) = 0;
Hu(2,3) = 0;
Hu(2,4) = 0;
Hu(2,5) = 0;
Hu(2,6) = 0;

Hu(3,1) = -fx*(Zprm^-2) * (J_g(1,:) * J_G(:,1))  +  2*fx*Xprm*(Zprm^-3) * (J_g(3,:) * J_G(:,1));
Hu(3,2) = -fx*(Zprm^-2) * (J_g(1,:) * J_G(:,2))  +  2*fx*Xprm*(Zprm^-3) * (J_g(3,:) * J_G(:,2));
Hu(3,3) = -fx*(Zprm^-2) * (J_g(1,:) * J_G(:,3))  +  2*fx*Xprm*(Zprm^-3) * (J_g(3,:) * J_G(:,3));
Hu(3,4) = -fx*(Zprm^-2) * (J_g(1,:) * J_G(:,4))  +  2*fx*Xprm*(Zprm^-3) * (J_g(3,:) * J_G(:,4));
Hu(3,5) = -fx*(Zprm^-2) * (J_g(1,:) * J_G(:,5))  +  2*fx*Xprm*(Zprm^-3) * (J_g(3,:) * J_G(:,5));
Hu(3,6) = -fx*(Zprm^-2) * (J_g(1,:) * J_G(:,6))  +  2*fx*Xprm*(Zprm^-3) * (J_g(3,:) * J_G(:,6));

Hu(4,1) = -fx*Yprm*(Zprm^-2) * (J_g(1,:) * J_G(:,1))  -  fx*Xprm*(Zprm^-2) * (J_g(2,:) * J_G(:,1))  +  2*fx*Xprm*Yprm*(Zprm^-3) * (J_g(3,:) * J_G(:,1));
Hu(4,2) = -fx*Yprm*(Zprm^-2) * (J_g(1,:) * J_G(:,2))  -  fx*Xprm*(Zprm^-2) * (J_g(2,:) * J_G(:,2))  +  2*fx*Xprm*Yprm*(Zprm^-3) * (J_g(3,:) * J_G(:,2));
Hu(4,3) = -fx*Yprm*(Zprm^-2) * (J_g(1,:) * J_G(:,3))  -  fx*Xprm*(Zprm^-2) * (J_g(2,:) * J_G(:,3))  +  2*fx*Xprm*Yprm*(Zprm^-3) * (J_g(3,:) * J_G(:,3));
Hu(4,4) = -fx*Yprm*(Zprm^-2) * (J_g(1,:) * J_G(:,4))  -  fx*Xprm*(Zprm^-2) * (J_g(2,:) * J_G(:,4))  +  2*fx*Xprm*Yprm*(Zprm^-3) * (J_g(3,:) * J_G(:,4));
Hu(4,5) = -fx*Yprm*(Zprm^-2) * (J_g(1,:) * J_G(:,5))  -  fx*Xprm*(Zprm^-2) * (J_g(2,:) * J_G(:,5))  +  2*fx*Xprm*Yprm*(Zprm^-3) * (J_g(3,:) * J_G(:,5));
Hu(4,6) = -fx*Yprm*(Zprm^-2) * (J_g(1,:) * J_G(:,6))  -  fx*Xprm*(Zprm^-2) * (J_g(2,:) * J_G(:,6))  +  2*fx*Xprm*Yprm*(Zprm^-3) * (J_g(3,:) * J_G(:,6));

Hu(5,1) = 2*fx*Xprm*(Zprm^-2) * (J_g(1,:) * J_G(:,1))  -  2*fx*(Xprm^2)*(Zprm^-3) * (J_g(3,:) * J_G(:,1));
Hu(5,2) = 2*fx*Xprm*(Zprm^-2) * (J_g(1,:) * J_G(:,2))  -  2*fx*(Xprm^2)*(Zprm^-3) * (J_g(3,:) * J_G(:,2));
Hu(5,3) = 2*fx*Xprm*(Zprm^-2) * (J_g(1,:) * J_G(:,3))  -  2*fx*(Xprm^2)*(Zprm^-3) * (J_g(3,:) * J_G(:,3));
Hu(5,4) = 2*fx*Xprm*(Zprm^-2) * (J_g(1,:) * J_G(:,4))  -  2*fx*(Xprm^2)*(Zprm^-3) * (J_g(3,:) * J_G(:,4));
Hu(5,5) = 2*fx*Xprm*(Zprm^-2) * (J_g(1,:) * J_G(:,5))  -  2*fx*(Xprm^2)*(Zprm^-3) * (J_g(3,:) * J_G(:,5));
Hu(5,6) = 2*fx*Xprm*(Zprm^-2) * (J_g(1,:) * J_G(:,6))  -  2*fx*(Xprm^2)*(Zprm^-3) * (J_g(3,:) * J_G(:,6));

Hu(6,1) = -fx*(Zprm^-1) * (J_g(2,:) * J_G(:,1))  +  fx*Yprm*(Zprm^-2) * (J_g(3,:) * J_G(:,1));
Hu(6,2) = -fx*(Zprm^-1) * (J_g(2,:) * J_G(:,2))  +  fx*Yprm*(Zprm^-2) * (J_g(3,:) * J_G(:,2));
Hu(6,3) = -fx*(Zprm^-1) * (J_g(2,:) * J_G(:,3))  +  fx*Yprm*(Zprm^-2) * (J_g(3,:) * J_G(:,3));
Hu(6,4) = -fx*(Zprm^-1) * (J_g(2,:) * J_G(:,4))  +  fx*Yprm*(Zprm^-2) * (J_g(3,:) * J_G(:,4));
Hu(6,5) = -fx*(Zprm^-1) * (J_g(2,:) * J_G(:,5))  +  fx*Yprm*(Zprm^-2) * (J_g(3,:) * J_G(:,5));
Hu(6,6) = -fx*(Zprm^-1) * (J_g(2,:) * J_G(:,6))  +  fx*Yprm*(Zprm^-2) * (J_g(3,:) * J_G(:,6));


% 2. Hv in feature-based approach









%% temporary part 3



% Hessian matrix of rigid body transformation matrix in SE(3)
d2T_dxi_j_dxi_i = cell(6,6);

d2T_dxi_j_dxi_i{1,1} = zeros(12,1);
d2T_dxi_j_dxi_i{1,2} = zeros(12,1);
d2T_dxi_j_dxi_i{1,3} = zeros(12,1);
d2T_dxi_j_dxi_i{1,4} = zeros(12,1);
d2T_dxi_j_dxi_i{1,5} = zeros(12,1);
d2T_dxi_j_dxi_i{1,6} = zeros(12,1);

d2T_dxi_j_dxi_i{2,1} = zeros(12,1);
d2T_dxi_j_dxi_i{2,2} = zeros(12,1);
d2T_dxi_j_dxi_i{2,3} = zeros(12,1);
d2T_dxi_j_dxi_i{2,4} = zeros(12,1);
d2T_dxi_j_dxi_i{2,5} = zeros(12,1);
d2T_dxi_j_dxi_i{2,6} = zeros(12,1);

d2T_dxi_j_dxi_i{3,1} = zeros(12,1);
d2T_dxi_j_dxi_i{3,2} = zeros(12,1);
d2T_dxi_j_dxi_i{3,3} = zeros(12,1);
d2T_dxi_j_dxi_i{3,4} = zeros(12,1);
d2T_dxi_j_dxi_i{3,5} = zeros(12,1);
d2T_dxi_j_dxi_i{3,6} = zeros(12,1);

d2T_dxi_j_dxi_i{4,1} = [zeros(9,1);0;0;0];
d2T_dxi_j_dxi_i{4,2} = [zeros(9,1);0;0;1];
d2T_dxi_j_dxi_i{4,3} = [zeros(9,1);0;-1;0];
d2T_dxi_j_dxi_i{4,4} = [0;-r21;-r31;0;-r22;-r32;0;-r23;-r33;0;-ty;-tz];
d2T_dxi_j_dxi_i{4,5} = [0;r11;0;0;r12;0;0;r13;0;0;tx;0];
d2T_dxi_j_dxi_i{4,6} = [0;0;r11;0;0;r12;0;0;r13;0;0;tx];

d2T_dxi_j_dxi_i{5,1} = [zeros(9,1);0;0;-1];
d2T_dxi_j_dxi_i{5,2} = [zeros(9,1);0;0;0];
d2T_dxi_j_dxi_i{5,3} = [zeros(9,1);1;0;0];
d2T_dxi_j_dxi_i{5,4} = [r21;0;0;r22;0;0;r23;0;0;ty;0;0];
d2T_dxi_j_dxi_i{5,5} = [-r11;0;-r31;-r12;0;-r32;-r13;0;-r33;-tx;0;-tz];
d2T_dxi_j_dxi_i{5,6} = [0;0;r21;0;0;r22;0;0;r23;0;0;ty];

d2T_dxi_j_dxi_i{6,1} = [zeros(9,1);0;1;0];
d2T_dxi_j_dxi_i{6,2} = [zeros(9,1);-1;0;0];
d2T_dxi_j_dxi_i{6,3} = [zeros(9,1);0;0;0];
d2T_dxi_j_dxi_i{6,4} = [r31;0;0;r32;0;0;r33;0;0;tz;0;0];
d2T_dxi_j_dxi_i{6,5} = [0;r31;0;0;r32;0;0;r33;0;0;tz;0];
d2T_dxi_j_dxi_i{6,6} = [-r11;-r21;0;-r12;-r22;0;-r13;-r23;0;-tx;-ty;0];


d2T_dxi_j_dxi_i_sym = cell(6,6);

for i=1:6
    for j=1:i
        
        if (i==j)
            d2T_dxi_j_dxi_i_sym{i,j} = d2T_dxi_j_dxi_i{i,j};
        else
            d2T_dxi_j_dxi_i_sym{i,j} = ( d2T_dxi_j_dxi_i{i,j} + d2T_dxi_j_dxi_i{j,i} ) / 2;
            d2T_dxi_j_dxi_i_sym{j,i} = ( d2T_dxi_j_dxi_i{i,j} + d2T_dxi_j_dxi_i{j,i} ) / 2;
        end
        
    end
end







