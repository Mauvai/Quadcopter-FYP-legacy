close all
x =[transpose(get(X_PositionSampled,'data')); transpose(get(Y_PositionSampled,'data')); transpose(get(HeightSampled,'data'))];
theta = [transpose(get(RollSampled,'data')); transpose(get(PitchSampled,'data')); transpose(get(YawSampled,'data'))];
vel = [transpose(get(XRateSampled,'data')); transpose(get(YRateSampled,'data')); transpose(get(HeightRateSampled,'data'))];
angvel = [transpose(get(RollRateSampled,'data')); transpose(get(PitchRateSampled,'data')); transpose(get(HeightRateSampled,'data'))];
time1 = transpose(get(X_PositionSampled,'time')); 

input = [transpose(get(PWM3,'data')); transpose(get(PWM4,'data')); transpose(get(PWM1,'data')); transpose(get(PWM2,'data'))];
data = struct('x',x,'theta',theta,'vel',vel,'angvel',angvel,'t',time1,'dt',0,'input',input);
visualize(data)
legend('Roll','Pitch','Yaw','Location','Best')