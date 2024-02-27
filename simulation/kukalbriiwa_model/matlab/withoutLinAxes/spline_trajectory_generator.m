%% simulink block

function q_d = fcn(t,start,stop,robot_model,duration,param_trajectory_generator_spline)
  persistent last_start;
  persistent last_stop;
  persistent started;
  persistent t_0;
  persistent q__0;
  persistent T;
  
  if(isempty(last_start))
    last_start=start;
  end
  if(isempty(last_stop))
    last_stop=stop;
  end
  if(isempty(started))
    started=0;
  end
  if(isempty(t_0))
    t_0=0;
  end
  if(isempty(q__0))
    q__0=robot_model.q;
  end
  if(isempty(T))
    T=duration;
  end
  
  if(started==1 && last_stop~=stop && stop==1)
    started=0;
    %Hold last position
  elseif(started==0 && last_start~=start && start==1)
    started=1;
    t_0=t;
    T=duration;
  end
  last_start=start;
  last_stop=stop;
  
  theta=(t-t_0)/T;
  theta_p=1/T;
  theta_pp=0;
  theta_limit=max(min(theta,1-1e-6),0);

  if(theta==theta_limit && started)
    degree=3;
    %B-Spline basis functions
    i=bspline_findspan(theta_limit,param_trajectory_generator_spline);
    N=bspline_basisfunction(theta_limit,i,2,param_trajectory_generator_spline);
    q=sum(param_trajectory_generator_spline.position(i:i+degree,:).*repmat(N(1,1:degree+1)',[1,7]),1)';
    dq=sum(param_trajectory_generator_spline.position(i:i+degree,:).*repmat(N(2,1:degree+1)',[1,7]),1)';
    ddq=sum(param_trajectory_generator_spline.position(i:i+degree,:).*repmat(N(3,1:degree+1)',[1,7]),1)';
    
    q__0=q;
  else
    started=0;
    q=q__0;
    dq=zeros(size(q__0));
    ddq=zeros(size(q__0));
  end
  
  q_d=[q;
    dq*theta_p;
    ddq*theta_p^2+dq*theta_pp];
end
