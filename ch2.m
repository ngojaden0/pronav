clear all;
n=0;
% velocity of missile and target
VM = 3000.;
VT = 900.;

% acceleration magnitude of target
g = 32.2; %ft /s^2
XNT = -g;

% initial heading error
HEDEG = 10.;

% acceleration magnitude of missile
XNP = 5.;

% initial positions of missile and target
RM1 = 0.;
RM2 = 10000.;
RT1 = 40000.;
RT2 = 10000.;

% initial target angle from reference frame
BETA=pi/6;

% velocity components of target
VT1=-VT*cos(BETA);
VT2=VT*sin(BETA);

% convert degrees to radians
HE=HEDEG/57.3;

% time 
T=0.;
S=0.;

% x and y distance between missile and target
RTM1=RT1-RM1;
RTM2=RT2-RM2;

% magnitude distance between missile and target
RTM=sqrt(RTM1*RTM1+RTM2*RTM2);

% angle between missile and target respect to earth frame
XLAM=atan2(RTM2,RTM1);

% missile lead angle, initial launch angle, correct on collision triangle
XLEAD=asin(VT*sin(BETA+XLAM)/VM);

% missile angle respect to earth frame
THET=XLAM+XLEAD;

% componentize missile velocity
VM1=VM*cos(THET+HE);
VM2=VM*sin(THET+HE);

% difference between missle and target velocities componentized
VTM1 = VT1 - VM1;
VTM2 = VT2 - VM2;

% closing velocity, -RdotTM
VC=-(RTM1*VTM1 + RTM2*VTM2)/RTM;

% when Vc changes direction
while VC >= 0

	if RTM < 1000
		% integration step size change
		H=.0002;
		else
		H=.01;
	end

	% set previous variables to current ones
	BETAOLD=BETA;
	RT1OLD=RT1;
	RT2OLD=RT2;
	RM1OLD=RM1;
	RM2OLD=RM2;
	VM1OLD=VM1;
	VM2OLD=VM2;

	STEP=1;
	FLAG=0;
	while STEP <=1 % 2nd order runge kutta
		if FLAG==1 % 1st step of runge kutta
			STEP=2;
			BETA=BETA+H*BETAD;
			RT1=RT1+H*VT1;
			RT2=RT2+H*VT2;
			RM1=RM1+H*VM1;
			RM2=RM2+H*VM2;
			VM1=VM1+H*AM1;
			VM2=VM2+H*AM2;
			T=T+H;
		end
		% solve for variables to find missile acceleration and beta dot
		RTM1=RT1-RM1;
		RTM2=RT2-RM2;
		RTM=sqrt(RTM1*RTM1+RTM2*RTM2);
		VTM1=VT1-VM1;
		VTM2=VT2-VM2;
		VC=-(RTM1*VTM1+RTM2*VTM2)/RTM;
		XLAM=atan2(RTM2,RTM1);
		XLAMD=(RTM1*VTM2-RTM2*VTM1)/(RTM*RTM);
		XNC=XNP*VC*XLAMD;
		AM1=-XNC*sin(XLAM);
		AM2=XNC*cos(XLAM);
		VT1=-VT*cos(BETA);
		VT2=VT*sin(BETA);
		BETAD=XNT/VT;
		FLAG=1;
	end
	% 2nd step of runge kutta
	FLAG=0;
	BETA=.5*(BETAOLD+BETA+H*BETAD);
	RT1=.5*(RT1OLD+RT1+H*VT1);
	RT2=.5*(RT2OLD+RT2+H*VT2);
	RM1=.5*(RM1OLD+RM1+H*VM1);
	RM2=.5*(RM2OLD+RM2+H*VM2);
	VM1=.5*(VM1OLD+VM1+H*AM1);
	VM2=.5*(VM2OLD+VM2+H*AM2);
	S=S+H;
	% plot every 0.1 seconds for computation sake
	if S >=.09999
		S=0.;
		n=n+1;
		ArrayT(n)=T;
		ArrayRT1(n)=RT1;
		ArrayRT2(n)=RT2;
		ArrayRM1(n)=RM1;
		ArrayRM2(n)=RM2;
		ArrayXNCG(n)=XNC/32.2;
		ArrayRTM(n)=RTM;

		% store heading over time
		ArrayMissileAngle(n)=rad2deg(XLAM+atan2(VM2,VM1));
		ArrayMissileY(n)=VM2;
	end
end
RTM;
figure
plot(ArrayRT1,ArrayRT2,ArrayRM1,ArrayRM2),grid
title('Two-dimensional tactical missile-target engagement simulation')
xlabel('Downrange (Ft) ')
ylabel('Altitude (Ft)')
% figure
% plot(ArrayT,ArrayXNCG),grid
% title('Two-dimensional tactical missile-target engagement simulation')
% xlabel('Time (sec)')
% ylabel('Acceleration of missle (G)')
% figure
% plot(ArrayT, ArrayMissileAngle), grid
% title('Missile Velocity Angle relative to earth frame')
% xlabel('Time (sec)')
% ylabel('Angle of missile velocity (deg)')
% figure
% plot(ArrayT, ArrayMissileY), grid
% title('Missile Y Velocity')
% xlabel('Time (sec)')
% ylabel('missile velocity Y (deg)')
clc
output=[ArrayT',ArrayRT1',ArrayRT2',ArrayRM1',ArrayRM2',ArrayXNCG',ArrayRTM' ];
