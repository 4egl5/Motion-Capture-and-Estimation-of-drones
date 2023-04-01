function Fyp8    
    % Connect natnet client to optic-track motion capture system
    n=natnet;
    %n.HostIP='192.168.0.102';
    %n.ClientIP='192.168.0.102';
    n.HostIP='127.0.0.1';
    n.ClientIP='127.0.0.1';
    
    n.ConnectionType='Multicast';
    n.connect;
    if(n.IsConnected==0)
        fprintf('Network issue\n');
        return
    end
    if(n.getModelDescription.RigidBodyCount<1)
        fprint('No rigid body detected');
        return
    end

    % variable initialization
    % set up loop frequency
    dt=0.1;%loop period
    r=rateControl(1/dt);%Run Loop In Constant Rate
    drone=1;
    sigma_a=0.5; % standard deviation in Acceleration
    sigma_x=2.5;
    sigma_y=2.5;
    sigma_z=2.5;

    % Initialize Kalman Filter constant / variable
    % State Transition Matrix , F
    F=[
    1 0 0 dt 0 0 dt^2/2 0 0;
    0 1 0 0 dt 0 0 dt^2/2 0;
    0 0 1 0 0 dt 0 0 dt^2/2;
    0 0 0 1 0 0 dt 0 0;
    0 0 0 0 1 0 0 dt 0;
    0 0 0 0 0 1 0 0 dt;
    0 0 0 0 0 0 1 0 0;
    0 0 0 0 0 0 0 1 0;
    0 0 0 0 0 0 0 0 1;
    ];
    % Initial State, x
    x=[
        double(n.getFrame.RigidBodies(drone).x);
        double(n.getFrame.RigidBodies(drone).y);
        double(n.getFrame.RigidBodies(drone).z);
        0;
        0;
        0;
        0;
        0;
        0;
    ];

    % Covariance Matrix, P
    P=[
        5 0 0 0 0 0 0 0 0;
        0 5 0 0 0 0 0 0 0;
        0 0 5 0 0 0 0 0 0; 
        0 0 0 5 0 0 0 0 0;
        0 0 0 0 5 0 0 0 0;
        0 0 0 0 0 5 0 0 0;
        0 0 0 0 0 0 5 0 0;
        0 0 0 0 0 0 0 5 0;
        0 0 0 0 0 0 0 0 5;
    ];

    % Process noise Matrix, Q
    Q=[
        dt^4/4 0 0 dt^3/2 0 0 dt^2/2 0 0;
        0 dt^4/4 0 0 dt^3/2 0 0 dt^2/2 0;
        0 0 dt^4/4 0 0 dt^3/2 0 0 dt^2/2;
        dt^3/2 0 0 dt^2 0 0 dt 0 0;
        0 dt^3/2 0 0 dt^2 0 0 dt 0;
        0 0 dt^3/2 0 0 dt^2 0 0 dt;
        dt^2/2 0 0 dt 0 0 1 0 0;
        0 dt^2/2 0 0 dt 0 0 1 0;
        0 0 dt^2/2 0 0 dt 0 0 1;
    ]*sigma_a^2;

    % Observation matrix, H
    H=[
        1 0 0 0 0 0 0 0 0;
        0 1 0 0 0 0 0 0 0;
        0 0 1 0 0 0 0 0 0; 
    ];

    % Measurement Covariance Matrix R
    % Random Variance in position x,y,z measurement.

    R=[
        sigma_x^2 0 0;
        0 sigma_y^2 0;
        0 0 sigma_z^2;
    ];    
    %Identity matrix
    I=eye(9);

    time=0; % Plot time axis        
    lm=[0;0;0;]; % measurement state at time step n-1, for velocity calculation
    l2m=[0;0;0;];% measurement state at time step n-2, for acceleration calculation
    v=[0;0;0;];  % measured velocity at time step n, for plot graph
    a=[0;0;0;];  % measured acceleration at time step n, for plot graph

    tmp=1;       % counter

    while true
        %tic;
        data=n.getFrame;

        %1: Measurement
        % measurement vector
        z=[
            double(data.RigidBodies(drone).x);
            double(data.RigidBodies(drone).y);
            double(data.RigidBodies(drone).z);
        ];
        if(tmp==1)
            lm=z;
        end
        if(tmp>1)
            a=(z-2*lm+l2m)/(dt^2);
            v=(z-lm)/dt;
            l2m=lm;
            lm=z;
            disp([z;v;a;]);
        end

        %2: Update
        % Kalman Gain
        K=P*transpose(H)/(H*P*transpose(H)+R);
        % Estimate current state
        x=x+K*(z-H*x);
        % Update Current estimate Uncertainty
        P=(I-K*H)*P*transpose(I-K*H)+K*R*transpose(K);
        disp(x);

        %3: Predict
        time=time+dt;
        % constant acceleration model, no acceleration control input.
        x=F*x;%+G*u; 
        P=F*P*transpose(F)+Q;
        disp(x);
        waitfor(r);
        %toc;
    end
