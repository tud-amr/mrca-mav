function dx = bebop_dynamics(x, u)

    % Bebop 2 dynamics model
    % 
    % state:   x, y, z, vx, vy, vz, phi, theta, psi
    % control: phi_c, theta_c, vz_c, psi_rate_c
    % 
    % f:
    %	\dot( x ) == vx
    %	\dot( y ) == vy
    %   \dot( z ) == vz
    %   \dot( vx) == (cos(psi)*tan(theta)/cos(phi) + sin(psi)*tan(phi))*g - kD_x*vx
    %   \dot( vy) == (sin(psi)*tan(theta)/cos(phi) - cos(psi)*tan(phi))*g - kD_y*vy
    %   \dot( vz) == (k_vz*vz_c - vz) / tau_vz
    %   \dot(phi) == (k_phi*phi_c - phi) / tau_phi
    %   \dot(theta) == (k_theta*theta_c - theta) / tau_theta
    %   \dot(psi) == psi_rate_c
    % 
    %   g           =   9.81
    %   kD_x        =   0.25
    %   kD_y        =   0.33
    %   k_vz        =   1.2270
    %   tau_vz      =   0.3367
    %   k_phi       =   1.1260
    %   tau_phi     =   0.2368
    %   k_theta     =   1.1075
    %   tau_theta   =   0.2318
    %
    % (c) Hai Zhu, TU Delft, 2019, h.zhu@tudelft.nl
    %

    %% Model parameters
    g           =   9.81;
    kD_x        =   0.25;
    kD_y        =   0.33;
    k_vz        =   1.2270;
    tau_vz      =   0.3367;
    k_phi       =   1.1260;
    tau_phi     =   0.2368;
    k_theta     =   1.1075;
    tau_theta   =   0.2318;

    %% control inputs
    phi_c       =   u(1);
    theta_c     =   u(2);
    vz_c        =   u(3);
    psi_rate_c  =   u(4);

    %% position dynamics
    vx          =   x(4);
    vy          =   x(5);
    vz          =   x(6);

    phi         =   x(7);
    theta       =   x(8);
%   psi         =   x(9);   % in this way, should be more accurate? 

%     ax = (cos(psi)*tan(theta)/cos(phi) + sin(psi)*tan(phi))*g - kD_x*vx;
%     ay = (sin(psi)*tan(theta)/cos(phi) - cos(psi)*tan(phi))*g - kD_y*vy;
    
    % in this way, might be computationally easier
    ax =  tan(theta)*g - kD_x*vx;
    ay = -tan(phi)*g - kD_y*vy;
    
    az = (k_vz*vz_c - vz) / tau_vz;

    %% attitude dynamics
    dphi    = (k_phi*phi_c - phi) / tau_phi;
    dtheta  = (k_theta*theta_c - theta) / tau_theta;
    dpsi    = psi_rate_c;

    %% output
    dx = [vx; vy; vz; ax; ay; az; dphi; dtheta; dpsi];


end
