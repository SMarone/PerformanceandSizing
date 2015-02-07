%%%%%%%%% First Order Sizing Code %%%%%%%%%
% Approximation for the tilt rotor vehicle
% Based on the hybrid electric configuration
% Evaluation of the vehicle performance and convergence on the vehicle TOGW
% (for loop formulation)
% Etienne Demers Bouchard
% 02/07/2015
%%%%%%%%% XXXXXXXXXXXXXXXXXXXXXXX %%%%%%%%%

%Atmosphere
rho = 1.25 ;

%Vehicle
Payload = 15 ;%kg
Phi = 0.8 ; %Empty Wt fraction
Fuel = 1;% kg
TOGW = (Payload + Fuel) / (1-Phi);
for i = 1:10
    Fuel=FuelTotal;
    TOGW = (Payload + Fuel) / (1-Phi);
    %Mission
    Hover = 120; %s
    CruiseDistance = 30; %km
    CruiseSpeed = 22.5; %m/s
    CruiseTime = CruiseDistance * 1000/CruiseSpeed ; %seconds
    
    %Wing
    b = 6 ; %m
    c = 0.75 ; %m
    e = 0.8 ; %Oeswald efficiency
    Cd0 = 0.06 ; %Parasitic Drag
    AR = b / c; %Aspect Ratio
    
    %Rotor
    Nhover = 4; %Number of Rotor used in hover
    NCruise= 2; % Number of rotor used in forward flight
    R = 0.3; % radius of rotor, in meter
    M = 0.8; % Propulsive efficiency (tbd)
    %wing
    CL = TOGW * 9.81 / (0.5 * rho * CruiseSpeed.^2 * b * c); %Lift Coeficient
    Cdi = CL.^2 / (pi * e * AR); %Induced drag coefficient
    CD = Cdi + Cd0; %Total drag Coefficient
    D = 0.5 * rho * CruiseSpeed.^2 * b * c * CD; % Newton of Drag
    
    %Power Required in hover
    DiskAreaHover = Nhover * pi * R.^2; %m^2
    PHover = 1/M* sqrt ( (TOGW * 9.81).^3 / (2* rho * DiskAreaHover ) ); % W
    
    %Power Required in Forward Flight
    Thrust = D / NCruise; %Thrsut per propeller
    PForwardFlight = NCruise .*  0.5 * Thrust .* CruiseSpeed .* (sqrt((Thrust /( pi * R^2 * CruiseSpeed * rho /2)) + 1) + 1); %W
    
    %Engine
    BFSC = 8.5e-8 ; %kg/(W*s)
    FuelHover = PHover * Hover *BFSC; %kg
    FuelCruise = PForwardFlight * CruiseTime *BFSC; %kg
    FuelTotal = FuelHover + FuelCruise;
end
TOGW