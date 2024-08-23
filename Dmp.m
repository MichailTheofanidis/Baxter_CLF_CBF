%% Dynamic Movement Primitives Class
%% by Michail Theofanidis
%% Modified to work on cartesian trajectories

classdef Dmp
    
    properties
        
        a % Gain a of the transformation system
        b % Gain b of the transformation system
        as % Degredation of the canonical system
        ng % Number of gaussians
        stb % Stabilization term
        obs % Obstacle position
        obs_gamma % Obstacle avoidance gain
        obs_radius % Obstacle radius

    end
    
    methods
        
        % Constructor method for the transformation system
        function obj = Dmp(obs_param)

            if nargin==0
                obj.a = 25;
                obj.b = obj.a/4;
                obj.as = obj.a/3;
                obj.ng = 100;
                obj.stb = 0;
            end
            
            if nargin==1
                obj.a = 25;
                obj.b = obj.a/4;
                obj.as = obj.a/3;
                obj.ng = 100;
                obj.stb = 0;
                obj.obs = [obs_param(1), obs_param(2), obs_param(3)];
                obj.obs_gamma = obs_param(4);
                obj.obs_radius = obs_param(5);
            end
                        
        end
        
        % Create the phase of the system
        function s = phase(obj,time)
            r=exp(-1*[obj.as]*linspace(0,1,length(time)));
            s=r';
        end
        
        % Gaussian distributions
        function psv=distributions(obj,s)
            
            % centers of gaussian are placed even in s time.
            incr=(s(1)-s(end))/(obj.ng-1);
            c=s(end):incr:s(1);
            lrc=fliplr(c);
            d=diff(c);
            c=c/d(1);
            
            % calculate every gaussian
            h=1;
            psv=zeros(obj.ng,length(s));
            
            for i=1:obj.ng
                for j=1:length(s)
                    psv(i,j)=psiF(h,c(i),s(j)/d(1));
                end
            end
            
        end

        % Immitation learning
        function [ftarget,w]=immitate(obj,x,dx,ddx,time,s,psv)
            
            % Define some variables
            dim=size(x);

            g=x(end,:);
            x0=x(1,:);
            tau=time(end);
            
            % Initialize ftarget and variables for regression
            ftarget=zeros(length(time),dim(2));
            sigma=zeros(length(time),dim(2));
            w=zeros(obj.ng,dim(2));
            
            % Compute ftarget
            for j=1:dim(2)
                for i=1:length(time)
                    
                    % Add the sabilization term
                    if obj.stb==1
                        mod=obj.b*(g(j)-x0(j))*s(i);
                        sigma(i,j)=s(i)*(g(j)-x0(j));
                    else
                        mod=0;
                        sigma(i,j)=s(i);
                    end
                    
                    ftarget(i,j)=tau^2*ddx(i,j)-obj.a*(obj.b*(g(j)-x(i,j))-tau*dx(i,j)+mod);
                                    
                end
            end
            
            % Regression
            for j=1:dim(2)
                for i=1:obj.ng
                    
                    %Locally Weighted Learning
                    w(i,j)=(transpose(sigma(:,j))*diag(psv(i,:))*ftarget(:,j))/(transpose(sigma(:,j))*diag(psv(i,:))*sigma(:,j));
                       
                end
            end
            
        end
        
        % Generate trajectory
        function[x,dx,ddx]=generate(obj,w,x0,g,t,s,psV)

            dim=size(w);
            
            ddx=zeros(length(s),dim(2));
            dx=zeros(length(s),dim(2));
            x=zeros(length(s),dim(2));

            f_rep=zeros(length(s),dim(2));
            
            tau=t(end);
            
            ddx_r=zeros(1,dim(2));
            dx_r=zeros(1,dim(2));
            x_r=x0;
            
            for i=1:length(t)
                
                psum_x=zeros(1,dim(2));
                pdiv_x=zeros(1,dim(2));
                
                if i==1
                    dt=t(i);
                else
                    dt=t(i)-t(i-1);
                end
                
                % Add the sabilization term
                if obj.stb==1
                    mod=obj.b*(g-x0)*s(i);
                    sigma(i)=s(i)*(g-x0);
                else
                    mod=0;
                    sigma(i)=s(i);
                end
                
                for k=1:dim(2)
                    for j=1:obj.ng
                        
                        % Normalizing the center of gaussians
                        psum_x(k)=psum_x(k)+psV(j,i)*w(j,k);
                        pdiv_x(k)=pdiv_x(k)+psV(j,i);
                        
                    end
                end

                % Generate the new trajectories according to the new control input
                f_rep(i,:)=(psum_x./pdiv_x)*sigma(i);

                if any(obj.obs)
                    if i>1
                        p=avoid_obstacles(x(i-1,:), dx(i-1,:), g, obj.obs,obj.obs_gamma,obj.obs_radius,dt);
                    else
                        p=0;
                    end
                else
                    p=0;
                end
                
                ddx_r=(obj.a*(obj.b*(g-x_r)-tau*dx_r)+f_rep(i,:)+mod+p)/tau^2;
                dx_r= dx_r+(ddx_r*dt);
                x_r= x_r+(dx_r*dt);
                                
                ddx(i,:)=ddx_r;
                dx(i,:)=dx_r;
                x(i,:)=x_r;
                
            end
            
        end
                       
        % Adapt trajectory with reinforcement learning
        function[x_r,dx_r,ddx_r]=adapt(obj,w,x0,g,t,s,psV,samples,rate)

            % Initialize action variables
            actions=ones(samples,obj.ng);
            expl=zeros(samples,obj.ng);
            a=w';
            
            % Flag for stopping condition
            flag=0;
            
            while flag==0
                
                for i=1:samples
                    for j=1:obj.ng
                        expl(i,j)=normrnd(0,std2(psV(j,:)*w(j)));
                    end
                end
                                
                % Sample all the possible actions
                for i=1:samples
                    actions(i,:)=a+expl(i,:);
                end
                               
                % Generate new rollouts
                x=zeros(length(t),samples);
                dx=zeros(length(t),samples);
                ddx=zeros(length(t),samples);
                
                for i=1:samples
                    [x(:,i),dx(:,i),ddx(:,i)]=generate(obj,actions,x0,g,t,s,psV);
                end
                
                 % Estimate the Q values
                Q=zeros(1,samples);

                for i=1:samples
                    sum=0;
                    for j=1:length(t)
                        sum=sum+Reward(g,x(j,i),t(j));
                    end
                    Q(i)=sum;
                end
                
                % Sample the highest Q values to aptade the action parameters
                high=floor(rate*samples);
                [Q,I] = sort(Q,'descend');
                
                % Update the action parameters
                sumQ_y=0;
                sumQ_x=0;
                for i=1:high
                    sumQ_y=sumQ_y+Q(i);
                    sumQ_x=sumQ_x+expl(I(i),:).*Q(i);
                end

                a=a+(sumQ_x)/(sumQ_y);
                
                if(abs(x(end,I(1))-g)<0.1)
                    flag=1;
                end
                
            end
            
            x_r=x(:,I(1));
            dx_r=x(:,I(1));
            ddx_r=x(:,I(1));
            
        end
            
    end
    
end

% Obstacle avoidance
function p = avoid_obstacles(x, dx, g, o,gamma,radius,dt)

    p = zeros(size(g));
    
    if norm(dx) > 0.00005
        if norm(o - x) < radius
            p=gamma*(1/radius-1/norm(o-x))*(1/(norm(o-x)^2))*((o-x)./dt);
            
        end
    end

end

%% My Gaussian function
function r=psiF(h, c, s)
    r=exp(-h*(s-c)^2);
end

%% My Reward function
function [rwd]=Reward(target,position,time)

    w=0.5;
    thres=0.01; 
    temp=target-position; 
    tau=time(end);

    if(abs(time-tau)<thres)
        rwd=(w)*exp(-sqrt(temp*temp'));
    else
        rwd=(1-w)*exp(-sqrt(temp*temp'))/tau;
    end

end
