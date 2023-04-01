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
    delay=10/dt;
    r=rateControl(1/dt);%Run Loop In Constant Rate
    drone=1;
    sigma_a=0.5; % standard deviation in Acceleration
    sigma_x=2.5;
    sigma_y=2.5;
    sigma_z=2.5;
    %standard deviation of measurement
    record_time=30;% second
    % Initialize Kalman Filter constant / variable
    mpx=[];
    mvx=[];
    max=[];
    ppx=[];
    pvx=[];
    pax=[];
    cpx=[];
    cvx=[];
    cax=[];
    mpy=[];
    mvy=[];
    may=[];
    ppy=[];
    pvy=[];
    pay=[];
    cpy=[];
    cvy=[];
    cay=[];
    mpz=[];
    mvz=[];
    maz=[];
    ppz=[];
    pvz=[];
    paz=[];
    cpz=[];
    cvz=[];
    caz=[];
    
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
    KGp=[];
    KGv=[];
    KGa=[];
    time=0; % Plot time axis        
    lm=[0;0;0;]; % measurement state at time step n-1, for velocity calculation
    l2m=[0;0;0;];% measurement state at time step n-2, for acceleration calculation
    v=[0;0;0;];  % measured velocity at time step n, for plot graph
    a=[0;0;0;];  % measured acceleration at time step n, for plot graph
    tmp=1;       % counter
    Mean=[0;0;0;0;0;0;0;0;0];
    standard_deviation=[0;0;0;0;0;0;0;0;0];

    
    % Plot graph
    m=[];
    s=[];
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
    
    cipxt=animatedline(hAxes(8));
    cipxb=animatedline(hAxes(8));
    cipxt.Color=[0 0.5 0];
    cipxb.Color=[0 0.5 0];
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
    
    cipyt=animatedline(hAxes(9));
    cipyb=animatedline(hAxes(9));
    cipyt.Color=[0 0.5 0];
    cipyb.Color=[0 0.5 0];
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
    
    cipzt=animatedline(hAxes(10));
    cipzb=animatedline(hAxes(10));
    cipzt.Color=[0 0.5 0];
    cipzb.Color=[0 0.5 0];
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
    civxt=animatedline(hAxes(2));
    civxb=animatedline(hAxes(2));
    civxt.Color=[0 0.5 0];
    civxb.Color=[0 0.5 0];
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
    
    civyt=animatedline(hAxes(3));
    civyb=animatedline(hAxes(3));
    civyt.Color=[0 0.5 0];
    civyb.Color=[0 0.5 0];
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
    civzt=animatedline(hAxes(4));
    civzb=animatedline(hAxes(4));
    civzt.Color=[0 0.5 0];
    civzb.Color=[0 0.5 0];
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
    
    ciaxt=animatedline(hAxes(5));
    ciaxb=animatedline(hAxes(5));
    ciaxt.Color=[0 0.5 0];
    ciaxb.Color=[0 0.5 0];
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
    ciayt=animatedline(hAxes(6));
    ciayb=animatedline(hAxes(6));
    ciayt.Color=[0 0.5 0];
    ciayb.Color=[0 0.5 0];
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
    ciazt=animatedline(hAxes(7));
    ciazb=animatedline(hAxes(7));
    ciazt.Color=[0 0.5 0];
    ciazb.Color=[0 0.5 0];
    mlaz.Marker='.';
    claz.Marker='.';
    plaz.Marker='.';
    mlaz.Color=[0 1 0];
    claz.Color=[0 0 1];
    plaz.Color=[1 0 0];
    legend('Measured','Corrected','Predicted');
    axis auto;
    
    for c=1:(record_time/dt)
    %while true
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
            if (tmp>2)
                max=[max a(1)];
                may=[may a(2)];
                maz=[maz a(3)];
            end
        mpx=[mpx z(1)];
        mvx=[mvx v(1)];
        
        mpy=[mpy z(2)];
        mvy=[mvy v(2)];
        
        mpz=[mpz z(3)];
        mvz=[mvz v(3)];
        
        
        end

        %2: Update
        % Kalman Gain
        K=P*transpose(H)/(H*P*transpose(H)+R);
        %disp(K);
        KGp=[KGp K(1)];
        KGv=[KGv K(4)];
        KGa=[KGa K(7)];
        % Estimate current state
        x=x+K*(z-H*x);
        % Update Current estimate Uncertainty
        P=(I-K*H)*P*transpose(I-K*H)+K*R*transpose(K);

        ppx=[ppx x(1)];
        pvx=[pvx x(4)];
        pax=[pax x(7)];
        
        ppy=[ppy x(2)];
        pvy=[pvy x(5)];
        pay=[pay x(8)];
        
        ppz=[ppz x(3)];
        pvz=[pvz x(6)];
        paz=[paz x(9)];
        
        if(tmp>delay)
            if(tmp-delay==1)
                Mean=x;%tmp=51 delay=50
                standard_deviation=[0;0;0;0;0;0;0;0;0];
            else
                %last_Mean=Mean; N-1        N
                Mean=((tmp-delay-1)*Mean+x)/(tmp-delay); % tmp=52 delay=50
                standard_deviation=sqrt(((tmp-delay-1)/(tmp-delay))*standard_deviation.^2+(1/(tmp-delay-1))*(x-Mean).^2);
            end
        end
        tmp=tmp+1;
        m=[m Mean];
        s=[s standard_deviation];
        
        %3: Predict
        time=time+dt;
        %u=[x(7);x(8);x(9);];

        % constant acceleration model, no acceleration control input.
        x=F*x;%+G*u; 
        P=F*P*transpose(F)+Q;
        cpx=[cpx x(1)];
        cvx=[cvx x(4)];
        cax=[cax x(7)];
        
        cpy=[cpy x(2)];
        cvy=[cvy x(5)];
        cay=[cay x(8)];
        
        cpz=[cpz x(3)];
        cvz=[cvz x(6)];
        caz=[caz x(9)];

        waitfor(r);
        %toc;
    end
    %display(s);
    %display(max);
    %display(mpx);
    for c=1:size(mpx,2)
        if (c<size(max,2))
           addpoints(clax,dt*c,cax(c));
           addpoints(clay,dt*c,cay(c));
           addpoints(claz,dt*c,caz(c));
         
           addpoints(mlax,dt*c,max(c));
           addpoints(plax,dt*c,pax(c));
           addpoints(mlay,dt*c,may(c));
           addpoints(play,dt*c,pay(c));
           addpoints(mlaz,dt*c,maz(c));
           addpoints(plaz,dt*c,paz(c));

        end
       addpoints(mlpx,dt*c,mpx(c));
       addpoints(plpx,dt*c,ppx(c));
       addpoints(clpx,dt*c,cpx(c));

       addpoints(mlvx,dt*c,mvx(c));
       addpoints(plvx,dt*c,pvx(c));
       addpoints(clvx,dt*c,cvx(c));
      
       addpoints(mlpy,dt*c,mpy(c));
       addpoints(plpy,dt*c,ppy(c));
       addpoints(clpy,dt*c,cpy(c));

       addpoints(mlvy,dt*c,mvy(c));
       addpoints(plvy,dt*c,pvy(c));
       addpoints(clvy,dt*c,cvy(c));
         
       addpoints(mlpz,dt*c,mpz(c));
       addpoints(plpz,dt*c,ppz(c));
       addpoints(clpz,dt*c,cpz(c));

       addpoints(mlvz,dt*c,mvz(c));
       addpoints(plvz,dt*c,pvz(c));
       addpoints(clvz,dt*c,cvz(c));

        addpoints(mlp,mpx(c),mpy(c));
        addpoints(plp,ppx(c),ppy(c));
        addpoints(clp,cpx(c),cpy(c));
        


        if(c<size(s,2)&& c>delay)
            
           addpoints(cipxt,dt*c,m(1,c)+(1.96*s(1,c)));
           addpoints(cipxb,dt*c,m(1,c)-(1.96*s(1,c)));

           addpoints(cipyt,dt*c,m(2,c)+(1.96*s(2,c)));
           addpoints(cipyb,dt*c,m(2,c)-(1.96*s(2,c)));


           addpoints(cipzt,dt*c,m(3,c)+(1.96*s(3,c)));
           addpoints(cipzb,dt*c,m(3,c)-(1.96*s(3,c)));


           addpoints(civxt,dt*c,m(4,c)+(1.96*s(4,c)));
           addpoints(civxb,dt*c,m(4,c)-(1.96*s(4,c)));

           addpoints(civyt,dt*c,m(5,c)+(1.96*s(5,c)));
           addpoints(civyb,dt*c,m(5,c)-(1.96*s(5,c)));

           addpoints(civzt,dt*c,m(6,c)+(1.96*s(6,c)));
           addpoints(civzb,dt*c,m(6,c)-(1.96*s(6,c)));

           addpoints(ciaxt,dt*c,m(7,c)+(1.96*s(7,c)));
           addpoints(ciaxb,dt*c,m(7,c)-(1.96*s(7,c)));
           addpoints(ciayt,dt*c,m(8,c)+(1.96*s(8,c)));
           addpoints(ciayb,dt*c,m(8,c)-(1.96*s(8,c)));
           addpoints(ciazt,dt*c,m(9,c)+(1.96*s(9,c)));
           addpoints(ciazb,dt*c,m(9,c)-(1.96*s(9,c)));
        end
    end


    figure (3)
    subplot(3,1,1)

    %line([min(x) max(x)],[0 dt*size(mpz(1))], 'LineWidth', dt, 'color',[1 1 0])
    plot(sqrt((mpz-Mean(3)).^2),'LineWidth',0.1,'color',[0 1 0])
    hold on
    plot(sqrt((ppz-Mean(3)).^2),'LineWidth',0.1,'color',[1 0 0])
    plot(sqrt((cpz-Mean(3)).^2),'LineWidth',0.1,'color',[0 0 1])
    legend('Measurement','Predicted','Corrected')
    title('Position error between true value')
    xlabel('measurement')
    ylabel('m')
    

    subplot(3,1,2)
    %line([min(x) max(x)],[0 dt*size(mpz(1))], 'LineWidth', dt, 'color',[1 1 0])
    plot(sqrt((mvz-Mean(6)).^2),'LineWidth',0.1,'color',[0 1 0])
    hold on
    plot(sqrt((pvz-Mean(6)).^2),'LineWidth',0.1,'color',[1 0 0])
    plot(sqrt((cvz-Mean(6)).^2),'LineWidth',0.1,'color',[0 0 1])
    legend('Measurement','Predicted','Corrected')
    title('Velocity error between true value')
    xlabel('measurement')
    ylabel('m/s')
    
    subplot(3,1,3)
    %line([min(x) max(x)],[0 dt*size(mpz(1))], 'LineWidth', dt, 'color',[1 1 0])
    plot(sqrt((maz-Mean(9)).^2),'LineWidth',0.1,'color',[0 1 0])
    hold on
    plot(sqrt((paz-Mean(9)).^2),'LineWidth',0.1,'color',[1 0 0])
    plot(sqrt((caz-Mean(9)).^2),'LineWidth',0.1,'color',[0 0 1])
    legend('Measurement','Predicted','Corrected')
    title('Acceleration error between true value')
    xlabel('measurement')
    ylabel('m/s^2')
    hold off

    figure(4)
    plot(KGp,'color',[0.5,0,0])
    hold on;
    plot(KGv,'color',[0,0,0.5])
    plot(KGa,'color',[0,0.5,0])
    legend('Position Kalman Gain','Velocity Kalman Gain','Acceleration Kalman Gain')
    title("Kalman Gain")
    xlabel('measurement')
    ylabel('Kalman Gain')

disp([std(mpx);std(mpy);std(mpz);std(mvx);std(mvy);std(mvz);std(max);std(may);std(maz);]);
drawnow();
   
end