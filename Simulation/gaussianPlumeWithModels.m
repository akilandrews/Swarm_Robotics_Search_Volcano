clear C z0 z1
dimx = 1:100:1000; dimy = -500:100:500; dimz = 1:10:100;
[C, z0] = gaussianPlume(500, 5, 50, 'h_ref', 10, 'reflection', false ...
    ,'deposition', false, 'amb_pres', 1103, 'amb_temp', 22 ...
    ,'X', dimx, 'Y', dimy, 'Z', dimz);
close all
figure;
set(gca, 'NextPlot', 'replace', 'Visible', 'off');
for z = 1:length(dimz)
    surf(C(:, :, z), 'EdgeColor', 'none', 'FaceColor', 'interp');
    colormap(jet);
    colorbar;
    axis tight;
    view(0, 90);
    xlabel('X'); ylabel('Y'); title(sprintf('height=%d',dimz(z)));
    pause(1.0);
end

% NOTE: {'amb_pres', Pa} and {'Pa',Pa} not used in calculations
function [C,z0]=gaussianPlume(Q, u_ref, h, varargin)
%gaussianPlume  Steady-state gaussian plume distribution model
%
%   gaussianPlume models the dispersion of a continuous point source, i.e.
%   plume, in various conditions and terrains. The output of gaussianPlume
%   is a 3-dimensional matrix containing the concentrations of the emitted
%   substance over a field with the first dimension (y) representing the
%   cross-wind axis, the second dimension (x) the downwind distance, and
%   the third dimension (z) the vertical axis. The origin is set at the
%   base of the stack unless Briggs plume rise model is used in which case
%   the horizontal (x axis) coordinate is offset such that x=0 marks the 
%   maximum rise of the plume downwind. All units are in <mgs> (meters,
%   grams, seconds) except where noted.
%
%   Most of the equations were taken from the ISC3 User's Manual, Volume
%   II, available online at
%   http://www.epa.gov/scram001/userg/regmod/isc3v2.pdf
%
%   The EPA now has newer software for modeling atmospheric dispersion
%   including AERMOD and CALPUFF
%   http://www.epa.gov/scram001/dispersion_prefrec.htm#aermod as these are
%   their preferred/recommended models.
%
%   C = gaussianPlume(Q) returns the steady-state Gaussian distribution
%   model of a single, continuous point source emitting at a rate of Q
%   grams per second for a 50m physical stack height with no calculations
%   for plume rise, in rural terrain with stability class "F" in the 
%   Guifford-Pasquiill scale. Wind speed is assumed to be 1m/s at the stack
%   tip (50m).
%
%   C = gaussianPlume(Q, u_ref) sets the wind at 1m to be u_ref
%
%   C = gaussianPlume(Q, u_ref, h) sets the stack height to h
%
%   [C, z0] = gaussianPlume(...) returns the effective stack height in
%       meters as z0.
%
%   C = gaussianPlume(Q, u_ref, h, ...) allows you to set certain options.
%   Options are set by 'option', <option value> pairs. The list of
%   permissible options are listed below:
%
%       {'h_ref', h_ref} Sets the reference height for the wind speed u_ref
%           in meters. Default is 1m.
%       {'stability', 'class'}    Sets Guifford-Pasquill stability class to
%           class 'class'. 'class' must be one of 'A', 'B', 'C', 'D', 'E', 
%           or 'F'. Default is 'F'.
%       {'terrain', 't_type'}   Sets the terrain to one of 'rural' or
%           'urban'. Default is 'urban'
%       {'p', p}    Sets the scaling factor for change in wind as a
%           function of altitude to p. Default is 0.4.
%       {'plum_rise_model', 'model'}  Sets the model to use for calculation
%           of plume rise. 'model' is one of 'none', 'CONCAWE',
%           'CarlsonMoses', 'Holland', or 'Briggs'. Default is 'none'.
%       {'mw', mw}  Sets the molecular weight of the exhaust for plume rise
%           calculations. Specify in atomic mass units. Default is average
%           of air, 30g/mole.
%       {'amb_temp', Ta} Sets the ambient temperature in degrees Celsius.
%           Default is 25C. Alternatively, you may specify {'Ta',Ta}.
%       {'stack_temp', Ts} Sets the stack temperature in degrees Celsius.
%           Default is 200C. Alternatively, you may specify {'Ts',Ts}.
%       {'stack_diameter', ds} Sets the stack diameter (in meters). Default
%           is 10m. Alternatively, you may specify {'ds',ds}.
%       {'specific_heat', Cp} Sets the specific heat of the exhaust gas in
%           J/degree Celsius/g. Default is 1.020 (constant pressure Cp for
%           dry air). Alternatively, you may specify {'Cp',Cp}.
%       {'amb_pres', Pa} Sets ambient pressure in millibars. Default is
%           1010mb. Alternatively, you may specify {'Pa',Pa}.
%       {'stack_pres', Ps} Sets stack tip pressure in millibars. Default is
%           1010mb. Alternatively, you may specify {'Ps',Ps}.
%       {'stack_velocity', vs} Sets the stack exit velocity in m/s. Default
%           is 1m/s. Alternativelt, you may specify {'vs',vs}.
%       {'lapse', eta} Sets the lapse rate for stable conditions (class E or
%           F). Alternatively, you may specify {'eta',eta}.
%       {'reflection', true} enables ground reflection whereas
%       {'reflection', false} disables ground reflection
%       {'deposition', true} models dry deposition whereas
%       {'deposition', false} disables dry deposition modeling
%       {'term_velocity', vt} Sets terminal/settling velocity in m/s.
%           Default is settling velocity for PM2.5, 0.5cm/s. Synonyms
%           include {'settling_velocity', vt} or simply {'vt', vt}.
%       {'X', x} Sets sampling points at the downwind distances. x is a
%           vector specifying the downwind distances to evaluate for C in
%           meters. Default is 1 to 5km in 100m resolution.
%       {'Y', y} Same as above except for the y-axis (cross-wind)
%       {'Z', z} Same as above except for the vertical axis
%
%   Example
%   ------------------------------
%   C=gaussianPlume(500, 5, 50, 'h_ref', 10, 'reflection', false, ...
%       'deposition', false, 'amb_pres', 1103, 'amb_temp', 22);
%       Computes the steady-state Gaussian-distribution of an emission at a
%       rate of 500g/s with average wind speed of 5m/s at 10m from a
%       physical stack height of 50m. No plume rise, reflection, nor
%       deposition will be modeled. The data will be sampled in 100m
%       resolution across the downwind and crosswind axes from 1 to 5km and
%       1 to 1km in the vertical axis.


% Check for missing arguments. Set to empty so we don't get undefined
% errors.
if(nargin<2)
    u_ref=[];
end

if(nargin<3)
    h=[];
end

if(isempty(u_ref))
    % Default wind speed is 1m/s
    u_ref=1;
end

if(isempty(h))
    % Default stack height is 50m
    h=50;
end

% Set default values
stability='F';              % Guifford-Pasquill stability class
plume_rise_model='none';    % Plume rise models: 'none', 'CONCAWE', 'Briggs'
reflection=false;           % Model ground reflection?
deposition=false;           % Model dry deposition?
terrain='urban';            % Terrain type, 'urban' or 'rural'
p=0.4;                      % Wind speed variation as a function of altitude, from 0.07 to 0.6
mw=30;                      % Average molecular weight of the efflux, typically air (g/mol)
Ta=25;                      % Ambient temperature (degrees C)
Ts=200;                     % Stack tip temperature (degrees C)
ds=10;                      % Stack tip diameter (meters)
Cp=1.020;                   % Specific heat of efflux (J/g degrees Kelvin). 1.020 is for air (http://www.efunda.com/materials/common_matl/show_gas.cfm?MatlName=Air0C)
Pa=1010;                    % Ambient pressure (millibars = 100 N/m^2 = 100 [kg*m/s^2]/m^2 = 1e5 [g/m/s^2])
Ps=1010;                    % Stack tip pressure (millibar)
x_grid=[1 100:100:5e3];     % Downwind distance of each sampled point (m)
y_grid=[1 100:100:5e3];     % Crosswind distance of each sampled point (m)
z_grid=[1 100:100:1e3];     % Altitude above ground level of each sampled point (m)
vt=0.005;                   % Settling (terminal) velocity in m/s
vs=1.5;                     % Stack exit velocity (m/s)
h_ref=1;                    % Altitude above ground level of measured windspeed (u_ref) (in m)
eta=0.035;                  % Lapse rate (degrees C/m)

% The rest of the arguments are set by optional 'string' 'value' pairs
for argnum=1:2:length(varargin)
    switch(varargin{argnum})
        case 'stability'
            stability=varargin{argnum+1};            
        case 'plume_rise_model'
            plume_rise_model=varargin{argnum+1};            
        case 'reflection'
            reflection=varargin{argnum+1};        
        case 'deposition'
            deposition=varargin{argnum+1};
        case 'terrain'
            terrain=varargin{argnum+1};        
        case 'p'
            p=varargin{argnum+1};            
        case 'mw'
            mw=varargin{argnum+1};            
        case 'amb_temp'
            Ta=varargin{argnum+1};            
        case 'Ta'
            Ta=varargin{argnum+1};            
        case 'stack_temp'
            Ts=varargin{argnum+1};            
        case 'Ts'
            Ts=varargin{argnum+1};            
        case 'stack_diameter'
            ds=varargin{argnum+1};            
        case 'ds'
            ds=varargin{argnum+1};            
        case 'specific_heat'
            Cp=varargin{argnum+1};            
        case 'Cp'
            Cp=varargin{argnum+1};            
        case 'amb_pres'
            Pa=varargin{argnum+1};
        case 'Pa'
            Pa=varargin{argnum+1};
        case 'stack_pres'
            Ps=varargin{argnum+1};            
        case 'Ps'
            Ps=varargin{argnum+1};            
        case 'X'
            x_grid=varargin{argnum+1};            
        case 'Y'
            y_grid=varargin{argnum+1};            
        case 'Z'
            z_grid=varargin{argnum+1};            
        case 'term_velocity'
            vt=varargin{argnum+1};            
        case 'vt'
            vt=varargin{argnum+1};            
        case 'settling_velocity'
            vt=varargin{argnum+1};            
        case 'stack_velocity'
            vs=varargin{argnum+1};            
        case 'vs'
            vs=varargin{argnum+1};            
        case 'h_ref'
            h_ref=varargin{argnum+1};            
        case 'lapse'
            eta=varargin{argnum+1};            
        case 'eta'
            eta=varargin{argnum+1};            
        otherwise
            warn('gaussianPlume:args', ['Unrecognized option: ', varargin{argnum}]);
    end    
end

% Setup some physical constants
Rc=8.3144e3;  % Universal gas constant in mJ/mol/K = [g*m^2/s^2/mol/K]
gc=9.80616;     % Graviational acceleration, m/s^2

% For internal calculations, convert stack and ambient temperatures 
% into degrees Kelvin
Ta=Ta+273;
Ts=Ts+273;

% Force x_grid, y_grid, and z_grid to be row vectors
if(size(x_grid, 1)==1)
    x_grid=x_grid';
end
if(size(y_grid, 1)==1)
    y_grid=y_grid';
end
if(size(z_grid, 1)==1)
    z_grid=z_grid';
end

% Determine wind velocity (us) at tip of stack using power law for wind
% speed measured at height h_ref (m) to be u_ref (m/s)
us=((h/h_ref)^p)*u_ref; % in m/s

% ISC3 manual warns of issues with us<1m/s
if(us<1)
    warning('gaussianPlume:badus','Stack height wind speed, u_s, is %.3fm/s which is less than 1m/s.', us);
end

% Compute mass flow using ideal gas law assumption
% PV=nRT (ideal gas law) <==> n=PV/RT
% m_dot=[m^2]*[m/s]*[g/m/s^2]*[g/mol]/[(mJ/mol/K)*K]
%      =[m^3][1/s]*[g/m/s^2]*[g/mol]/[g*m^2/s^2/mol]
%      =[g/s]
m_dot=pi*(ds/2)^2*vs*(Ps*1e5)*mw/(Rc*Ts);   % g/s
% Now compute heat flux
% Qh=[g/s]*[J/(g*degK)]*degK
%   =[J/s]
Qh=m_dot*Cp*(Ts-Ta);    % J/s

% Compute plume rise
switch(plume_rise_model)
    case 'none'
        % The rise is then set to zero
        plume_rise=0;
        x_f=0;
    case 'CONCAWE'
        % TODO: Verify
        plume_rise=4.71*(Qh^0.44/us^0.694);
        x_f=0;
    case 'Holland'
        % TODO: Verify
        plume_rise=vs*ds/us*(1.5+0.01*Qh/(vs*ds));
        x_f=0;
    case 'CarlsonMoses'
        % TODO: Verify
        plume_rise=0.029*vs*ds/us+2.62*(Qh^0.5/us);
        x_f=0;
    case 'Briggs'
        % Compute buoyancy and momentum factors
        % Fb=[m/s^2]*[m/s]*[m^2]*[degK]/[degK]
        %   =[m^4]/[s^3]
        Fb=gc*vs*ds^2*(Ts-Ta)/(4*Ts);
        % Fm=[m^2/s^2]*[m^2]*[degK]/[degK]
        %   =[m^4]/[s^2]
        Fm=vs^2*ds^2*Ta/(4*Ts);
        % Compute stability parameter if stable
        if(stability=='E' || stability=='F')
            % s = [m/s^2]*[degK/m]/[degK]
            %   = [1/s^2]
            s=gc*eta/Ta; % 1/s^2
            % Can't perform unit analysis, factor 0.019... is unknown
            dTc=0.019582*Ts*vs*sqrt(s);
            % Check if buoyancy dominated (thermal) or momentum (kinetic)
            if( (Ts-Ta)>=dTc)
                % Buoyancy dominates
                x_f=2.0715*us/sqrt(s);
                plume_rise=2.6*(Fb/(us*s))^(1/3);
            else
                x_f=0;
                % Note that we also evaluate unstable/neutral and select
                % the lower value
                plume_rise=min(1.5*(Fm/(us*sqrt(s)))^(1/3), 3*ds*vs/us);
            end
        else
            % Unstable or neutral              
            if(Fb<55)
                % Check for buoyancy dominated or momentum
                dTc=0.0297*Ts*vs^(1/3)/ds^(2/3);
                if( (Ts-Ta)>=dTc )
                    % Buoyancy dominated
                    x_f=49*Fb^(5/8);
                    plume_rise=21.425*Fb^(3/4)/us;
                else
                    % Momentum dominated
                    x_f=0;
                    plume_rise=3*ds*vs/us;
                    % Check
                    if(vs/us<=4)
                        warning('gaussianPlume:Briggs', 'Momentum rise model selected but may not be accurate due to low stack exit velocity');
                    end
                end
            else
                % Brigg's equations seem to diverage at Fb==55
                dTc=0.00575*Ts*vs^(2/3)/ds^(1/3);
                if( (Ts-Ta)>=dTc )
                    % Offset horizontal downwind distance for distance to
                    % max height (rise)
                    x_f=119*Fb^(2/5);
                    plume_rise=38.71*Fb^(3/5)/us;
                else
                    x_f=0;
                    plume_rise=3*ds*vs/us;
                    if(vs/us<=4)
                        warning('gaussianPlume:Briggs', 'Momentum rise model selected but may not be accurate due to low stack exit velocity');
                    end
                end
            end
        end            
    otherwise
        warning('gaussianPlume:riseModel', ['Unrecognized plume rise model specified: ', plume_rise_model]);
        plume_rise=0;
        x_f=0;
end

% Set effective stack height
z0=h+plume_rise;

% Offset the downwind direction to take into account distance to max rise
% TODO: Substitute appropriate equations for distance less than final rise
% per ISC3v2 manual for buoyancy dominated conditions
x_grid=x_grid-x_f;

% Compute the dispersion coefficients
switch(terrain)
    case 'rural'
        % Pasquill-Gifford curves
        switch(stability)            
            case 'A'
                % [c, d] coefficients
                coeffs_y=[24.1670, 2.5334];
                % [x a b] matrix
                coeffs_z=[0.10 122.800 0.94470;
                          0.16 158.080 1.05420;
                          0.21 170.220 1.09320;
                          0.26 179.520 1.12620;
                          0.31 217.410 1.26440;
                          0.41 258.890 1.40940;
                          0.51 346.750 1.72830;
                          3.11 453.850 2.11660;
                          inf  nan     nan];
            case 'B'
                coeffs_y=[18.3330, 1.8096];
                coeffs_z=[0.20 90.673 0.93198;
                          0.40 98.483 0.98332;
                          inf  109.300 1.09710];
            case 'C'
                coeffs_y=[12.5000, 1.0857];
                coeffs_z=[inf 61.141 0.91465];
            case 'D'
                coeffs_y=[8.3330, 0.72382];
                coeffs_z=[0.31 34.459 0.86974;
                          1.01 32.093 0.81066;
                          3.01 32.093 0.64403;
                          10.01 33.504 0.60486;
                          30.00 36.650 0.56589;
                          inf   44.053 0.51179];
            case 'E'
                coeffs_y=[6.2500, 0.54287];
                coeffs_z=[0.10 24.260 0.83660;
                          0.31 23.331 0.81956;
                          1.01 21.628 0.75660;
                          2.01 21.628 0.63077;
                          4.01 22.534 0.57154;
                          10.01 24.703 0.50527;
                          20.01 26.970 0.46713;
                          40.00 35.420 0.37615;
                          inf 47.618 0.29592];                      
            case 'F'
                coeffs_y=[4.1667, 0.36191];
                coeffs_z=[0.21 15.209 0.81558;
                          0.71 14.457 0.78407;
                          1.01 13.953 0.68465;
                          2.01 13.953 0.63227;
                          3.01 14.823 0.54503;
                          7.01 16.187 0.46490;
                          15.01 17.836 0.41507;
                          30.01 22.651 0.32681;
                          60.00 27.074 0.27436;
                          inf   34.219 0.21716];
            otherwise
                error('gaussianPlume:stability', ['Unknown stability class ', stability]);                
        end
        % Construct sigma_y vector along the x-axis
        % Note that x should be in kilometers (x_grid/1.e3)
        sigma_y=465.11628.*(x_grid./1e3).*tan(0.017453293.*(coeffs_y(1)-coeffs_y(2).*log(x_grid./1e3))); % m
        % Construct sigma_z vector along the x-axis
        prev_boundary=0; % in km    
        % Pre-allocate (should be same size as x_grid since all tables end
        % with 'inf')
        sigma_z=nan(size(x_grid));
        for section=1:size(coeffs_z, 1)
            idx=find(prev_boundary<=(x_grid./1e3) & (x_grid./1e3)<coeffs_z(section, 1));
            sigma_z(idx)=coeffs_z(section, 2).*(x_grid(idx)./1e3).^coeffs_z(section, 3);
            prev_boundary=coeffs_z(section, 1);
        end
        % Move the vector into the proper dimension (cols=x)
        sigma_y=shiftdim(sigma_y, -1);
        % Clip all values>5e3 for stability classes A-C
        switch(stability)
            case {'a', 'b', 'c'}
                sigma_y(sigma_y>5e3)=5e3;
        end        
        %sigma_z=shiftdim(sigma_z, -1);        
    case 'urban'
        % Pasquill-Gifford with urban fit (McElroy-Pooler)
        switch(stability)
            case 'A'
                coeffs_y=0.32;
                coeffs_z=[0.24 1 0.001 0.5];
            case 'B'
                coeffs_y=0.32;
                coeffs_z=[0.24 1 0.001 0.5];
            case 'C'
                coeffs_y=0.22;
                coeffs_z=[0.20 1 0 0];
            case 'D'
                coeffs_y=0.16;
                coeffs_z=[0.14 1 0.0003 -0.5];
            case 'E'
                coeffs_y=0.11;
                coeffs_z=[0.08 1 0.0015 -0.5];
            case 'F'
                coeffs_y=0.11;
                coeffs_z=[0.08 1 0.0015 -0.5];
            otherwise
                error('gaussianPlume:stability', ['Unrecognized stability class ', stability]);
        end
        % Construct sigma_y along x-axis
        sigma_y=coeffs_y(1).*x_grid.*(1+0.0004.*x_grid).^(-0.5);
        sigma_y=shiftdim(sigma_y, -1);
        % Construct sigma_z along x-axis
        sigma_z=coeffs_z(1).*x_grid.*(coeffs_z(2)+coeffs_z(3).*x_grid).^coeffs_z(4);        
        sigma_z=shiftdim(sigma_z, -1);
    otherwise
        error('gaussianPlume:terrain', ['Unrecognized terrain option ', terrain]);        
end

% Now we have a choice when we compute Gaussian distribution. We
% can either save memory and use a for-loop, or speed the code up
% at the expense of more memory requirement. Since memory is cheap
% these days, we'll go ahead and optimize for speed in favor of
% size.
        
% Now replicate sigma vectors into 3D matrices
sigma_y=repmat(sigma_y, [length(y_grid) 1 length(z_grid)]);
sigma_z=repmat(sigma_z, [length(y_grid) 1 length(z_grid)]);

% Create wind matrix (scaled according to altitude)
u_matrix=((z_grid./h_ref).^p).*u_ref; % [m/s]
% Move over into z-dimension
u_matrix=shiftdim(u_matrix, -2);
u_matrix=repmat(u_matrix, [length(y_grid) length(x_grid) 1]);

% At this point, we will scale x_grid/y_grid/z_grid into full 3-D matrices
% Because we now manipulate them, first save their original lengths
sx=size(x_grid, 1);
sy=size(y_grid, 1);
sz=size(z_grid, 1);
x_grid=repmat(shiftdim(x_grid, -1), [sy 1 sz]);
y_grid=repmat(y_grid, [1 sx sz]);
z_grid=repmat(shiftdim(z_grid, -2), [sy sx 1]);

% Setup deposition
if(deposition)
    % Note here we had to massage x_grid into full 3-D matrix
    dz_dep=vt.*x_grid./u_matrix;
else
    dz_dep=zeros(size(x_grid));
end

if(reflection)
    % Reflection scales everything by 2
    r=2;
else
    r=1;
end

% Now compute steady-state Gaussian dispersion
% (Note everything inside the exp() becomes unitless)
% C=[g/s]/[m/s*m*m]=[g/m^3]
C=r.*Q./ ...
    (2.*pi.*u_matrix.*sigma_y.*sigma_z).*exp( ...
        (-y_grid.^2)./(2.*sigma_y.^2) - ((z_grid-z0-dz_dep).^2)./(2.*sigma_z.^2));
end
