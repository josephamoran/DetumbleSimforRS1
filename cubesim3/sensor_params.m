%%%Update Rate
nextSensorUpdate = 5;

MagscaleBias = (4e-7)*0; %nT
MagFieldBias = MagscaleBias*(2-rand()-1)*0; %%% -1 to 1

MagscaleNoise = (1e-5); %T
MagFieldNoise = MagscaleNoise*(2*rand()-1)*0;

AngscaleBias = 0.01; %rad/s
AngFieldBias = AngscaleBias*(2*rand()-1)*0;

AngscaleNoise = 0.001; %rad/s
AngFieldNoise = AngscaleNoise*(2*rand()-1)*0;