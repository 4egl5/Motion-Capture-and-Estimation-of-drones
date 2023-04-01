function Fyp7
    
    % Connect natnet client to optic-track motion capture system
    n=natnet;
    n.HostIP='192.168.0.102';
    n.ClientIP='192.168.0.102';
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
    

    % set up loop frequency
    dt=0.8;%loop period
    r=rateControl(1/dt);%Run Loop In Constant Rate

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
        double(n.getFrame.RigidBodies(2).x);
        double(n.getFrame.RigidBodies(2).y);
        double(n.getFrame.RigidBodies(2).z);
        0;
        0;
        0;
        0;
        0;
        0;
    ];


    % Control Matrix, G
    G=[
        dt^2/2 0 0;
        0 dt^2/2 0;
        0 0 dt^2/2;
        dt 0 0;
        0 dt 0;
        0 0 dt;
        1 0 0;
        0 1 0;
        0 0 1;
    ];

    % Covariance Matrix, P
    P=[
        1 0 0 0 0 0 0 0 0;
        0 1 0 0 0 0 0 0 0;
        0 0 1 0 0 0 0 0 0; 
        0 0 0 1 0 0 0 0 0;
        0 0 0 0 1 0 0 0 0;
        0 0 0 0 0 1 0 0 0;
        0 0 0 0 0 0 1 0 0;
        0 0 0 0 0 0 0 1 0;
        0 0 0 0 0 0 0 0 1;
    ];

    % Process noise Matrix, Q
    sigma_a=2; % Random Variance in Acceleration
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
    % Random Variance in position x,y,z measurement
    sigma_x=50;
    sigma_y=50;
    sigma_z=50;
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
       
    mean=[0;0;0];%0;0;0;0;0;0];
    standard_deviation=[0;0;0];%0;0;0;0;0;0];
    % Plot graph
    
  %{
    
    figure(1);
    title('tragectory');
    xlabel('m');
    ylabel('m');
    %zlabel('m');
    hAxes(1)=gca;
    mlp=animatedline(hAxes(1));
    mlp.Marker='.';
    clp=animatedline(hAxes(1));
    clp.Marker='.';
    plp=animatedline(hAxes(1));
    plp.Marker='.';
    mlp.Color=[0 1 0];
    clp.Color=[0 0 1];
    plp.Color=[1 0 0];
    legend('Measured','Corrected','Predicted');
    %view(3); 
    grid;
    %}
    figure(2);
    %fig.WindowStyle='docked';
    subplot(3,3,1);
    title('Px');
    xlabel('time(s)');
    ylabel('m');
    hAxes(8)=gca;
    mlpx=animatedline(hAxes(8));
    clpx=animatedline(hAxes(8));
    plpx=animatedline(hAxes(8));
    mlpx.Color=[0 1 0];
    clpx.Color=[0 0 1];
    plpx.Color=[1 0 0];
    mlpx.Marker='.';
    clpx.Marker='.';
    plpx.Marker='.';
    legend('Measured','Corrected','Predicted');
    
    subplot(3,3,2);
    title('Py');
    xlabel('time(s)');
    ylabel('m');
    hAxes(9)=gca;
    mlpy=animatedline(hAxes(9));
    clpy=animatedline(hAxes(9));
    plpy=animatedline(hAxes(9));
    mlpy.Color=[0 1 0];
    clpy.Color=[0 0 1];
    plpy.Color=[1 0 0];
    mlpy.Marker='.';
    clpy.Marker='.';
    plpy.Marker='.';
    legend('Measured','Corrected','Predicted');
    axis auto;
    
    subplot(3,3,3);
    title('Pz');
    xlabel('time(s)');
    ylabel('m');
    hAxes(10)=gca;
    mlpz=animatedline(hAxes(10));
    clpz=animatedline(hAxes(10));
    plpz=animatedline(hAxes(10));
    mlpz.Color=[0 1 0];
    clpz.Color=[0 0 1];
    plpz.Color=[1 0 0];
    mlpz.Marker='.';
    clpz.Marker='.';
    plpz.Marker='.';
    legend('Measured','Corrected','Predicted');
    axis auto;
    
    
    subplot(3,3,4);
    %figure(2);
    title('Vx');
    xlabel('time(s)');
    ylabel('m/s');
    hAxes(2)=gca;
    mlvx=animatedline(hAxes(2));
    clvx=animatedline(hAxes(2));
    plvx=animatedline(hAxes(2));
    mlvx.Marker='.';
    clvx.Marker='.';
    plvx.Marker='.';
    mlvx.Color=[0 1 0];
    clvx.Color=[0 0 1];
    plvx.Color=[1 0 0];
    legend('Measured','Corrected','Predicted');
    axis auto;
    
    subplot(3,3,5);
    %figure(3);
    title('Vy');
    hAxes(3)=gca;
    xlabel('time(s)')
    ylabel('m/s')
    mlvy=animatedline(hAxes(3));
    clvy=animatedline(hAxes(3));
    plvy=animatedline(hAxes(3));
    mlvy.Marker='.';
    clvy.Marker='.';
    plvy.Marker='.';
    mlvy.Color=[0 1 0];
    clvy.Color=[0 0 1];    
    plvy.Color=[1 0 0];    
    legend('Measured','Corrected','Predicted');
    axis auto;
    
    subplot(3,3,6);   
    %figure(4);
    title('Vz');
    ylabel('m/s')
    xlabel('time(s)')
    hAxes(4)=gca;
    mlvz=animatedline(hAxes(4));
    clvz=animatedline(hAxes(4));
    plvz=animatedline(hAxes(4));
    mlvz.Marker='.';
    clvz.Marker='.';
    plvz.Marker='.';
    mlvz.Color=[0 1 0];
    clvz.Color=[0 0 1];
    plvz.Color=[1 0 0];
    legend('Measured','Corrected','Predicted');
    axis auto;
    
    subplot(3,3,7);
    %figure(5);
    hAxes(5)=gca;
    xlabel('time(s)');
    ylabel('m/s^2');
    title('Ax');
    mlax=animatedline(hAxes(5));
    clax=animatedline(hAxes(5));
    plax=animatedline(hAxes(5));
    mlax.Marker='.';
    clax.Marker='.';
    plax.Marker='.';
    mlax.Color=[0 1 0];
    clax.Color=[0 0 1];
    plax.Color=[1 0 0];
    legend('Measured','Corrected','Predicted');
    axis auto;
    
    %figure(6);
    subplot(3,3,8);
    xlabel('time(s)');
    ylabel('m/s^2');
    title('Ay');
    hAxes(6)=gca;
    mlay=animatedline(hAxes(6));
    clay=animatedline(hAxes(6));
    play=animatedline(hAxes(6));
    mlay.Marker='.';
    clay.Marker='.';
    play.Marker='.';
    mlay.Color=[0 1 0];
    clay.Color=[0 0 1];    
    play.Color=[1 0 0];    
    legend('Measured','Corrected','Predicted');
    axis auto;
    
    subplot(3,3,9);
    %figure(7);
    ylabel('m/s^2');
    xlabel('time(s)');
    title('Az');
    hAxes(7)=gca;
    mlaz=animatedline(hAxes(7));
    claz=animatedline(hAxes(7));
    plaz=animatedline(hAxes(7));
    mlaz.Marker='.';
    claz.Marker='.';
    plaz.Marker='.';
    mlaz.Color=[0 1 0];
    claz.Color=[0 0 1];
    plaz.Color=[1 0 0];
    legend('Measured','Corrected','Predicted');
    axis auto;
    

    
    %for c=1:(10/dt)
    while true
        tic;
        data=n.getFrame;

        %1: Measurement
        % measurement vector
        z=[
            double(data.RigidBodies(2).x);
            double(data.RigidBodies(2).y);
            double(data.RigidBodies(2).z);
        ];
        
        if(tmp==1)
            %v=(z-lm)/dt;
            %addpoints(mlvx,time,v(1));
            %addpoints(mlvy,time,v(2));       
            %addpoints(mlvz,time,v(3));
            lm=z;
        end
        if(tmp>1)
            a=(z-2*lm+l2m)/(dt^2);
            if(tmp>2)
                addpoints(mlax,time,a(1));
                addpoints(mlay,time,a(2));
                addpoints(mlaz,time,a(3));
            end

            v=(z-lm)/dt;
            
            addpoints(mlvx,time,v(1));
            addpoints(mlvy,time,v(2));       
            addpoints(mlvz,time,v(3));

            l2m=lm;
            lm=z;
            %addpoints(mlp,z(1),z(2));
            addpoints(mlpx,time,z(1));
            addpoints(mlpy,time,z(2));
            addpoints(mlpz,time,z(3));
        end
        drawnow()
        tmp=tmp+1;
       
        %2: Update
        % Kalman Gain
        K=P*transpose(H)/(H*P*transpose(H)+R);
        %time=time+dt;
        % Estimate current state
        x=x+K*(z-H*x);
        % Update Current estimate Uncertainty
        P=(I-K*H)*P*transpose(I-K*H)+K*R*transpose(K);

        %mean=((tmp-1)*mean+z)/tmp;
        %standard_deviation=sqrt(((tmp-1)/tmp)*standard_deviation.^2+(1/(tmp-1))*(z-mean).^2);
        %display(mean);
        %display(standard_deviation);
        %addpoints(clp,x(1),x(2));
        
        
        addpoints(clvx,time,x(4));

        addpoints(clvy,time,x(5));
        addpoints(clvz,time,x(6));

        
        addpoints(clax,time,x(7));

        addpoints(clay,time,x(8));
        addpoints(claz,time,x(9));

       
        addpoints(clpx,time,x(1));

       
        addpoints(clpy,time,x(2));
        addpoints(clpz,time,x(3));


        %display(P);

        %3: Predict
        time=time+dt;
        %u=[x(7);x(8);x(9);];

        % constant acceleration model, no acceleration control input.
        x=F*x;%+G*u; 
        P=F*P*transpose(F)+Q;
        %addpoints(plp,x(1),x(2));
        
        
        addpoints(plvx,time,x(4));

        addpoints(plvy,time,x(5));
        addpoints(plvz,time,x(6));

        addpoints(plax,time,x(4));

        addpoints(play,time,x(5));
        addpoints(plaz,time,x(6));

   
        addpoints(plpx,time,x(1));

        addpoints(plpy,time,x(2));
        addpoints(plpz,time,x(3));
        
        drawnow;


       
       % display(P);
        
        % wait
        %display(tmp);
        %display(tmp*dt);

        waitfor(r);
        toc;
    end
end