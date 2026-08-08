%% TinyMPC initialization for PX4/Gazebo prototype

clear prob

%% ------------------------------------------------------------------------
% TinyMPC paths
%% ------------------------------------------------------------------------

Ts = 0.02;              % 50 Hz outer loop
currentFile = mfilename('fullpath');
repoRoot = fileparts(currentFile);
srcDir = fullfile(repoRoot,'src');
if exist(srcDir,'dir')
    addpath(srcDir);
end

tinympcMatlabPath = getenv('TINY_MPC_MATLAB_PATH');
if ~isempty(tinympcMatlabPath) && exist(tinympcMatlabPath,'dir')
    addpath(genpath(tinympcMatlabPath));
end

buildDir = fullfile(repoRoot,'build');

mexFile = ['tinympc_matlab.' mexext];
mexDirs = {...
    buildDir,...
    fullfile(buildDir,'Debug'),...
    fullfile(buildDir,'Release')};

for k = 1:numel(mexDirs)
    if exist(fullfile(mexDirs{k},mexFile),'file')
        addpath(mexDirs{k});
        break
    end
end

hasTinyMPCMatlab = exist('TinyMPC','class') == 8 || exist('TinyMPC','file') == 2;

%% ------------------------------------------------------------------------
% Vehicle model
%
% Replace these matrices later with the identified X500 model.
%% ------------------------------------------------------------------------

Adyn = eye(12);

% Position integrates velocity
Adyn(1,7) = Ts;
Adyn(2,8) = Ts;
Adyn(3,9) = Ts;

Bdyn = zeros(12,4);

% TinyMPC is used as the outer loop. Its control output is a PX4
% trajectory-setpoint-style acceleration feed-forward plus yaw speed:
%
%   u = [ax; ay; az; yawspeed]
%
% PX4 consumes these through TrajectorySetpoint and keeps its
% acceleration-to-attitude conversion, attitude controller, and mixer.
Bdyn(1,1) = 0.5*Ts^2;
Bdyn(2,2) = 0.5*Ts^2;
Bdyn(3,3) = 0.5*Ts^2;

Bdyn(7,1) = Ts;
Bdyn(8,2) = Ts;
Bdyn(9,3) = Ts;

Bdyn(6,4) = Ts;

%% ------------------------------------------------------------------------
% Controller settings
%% ------------------------------------------------------------------------
N  = 25;                % 0.5 second prediction horizon

rho = 5.0;

Q = diag([...
    100 ...     % x
    100 ...     % y
    150 ...     % z
    0 ...       % roll
    0 ...       % pitch
    5 ...       % yaw
    15 ...      % vx
    15 ...      % vy
    20 ...      % vz
    0 ...       % p
    0 ...       % q
    0]);        % r

R = diag([...
    1
    1
    1
    1]);

nx = size(Adyn,1);
nu = size(Bdyn,2);

Xref = zeros(nx,N);
Uref = zeros(nu,N-1);

%% ------------------------------------------------------------------------
% Build MATLAB solver if the TinyMPC MATLAB interface is available.
%
% The Simulink PX4 demo calls the C++ wrapper directly through coder.ceval, so
% tinympc-matlab is helpful for MATLAB-only experiments but is not required to
% load or build the demo model.
%% ------------------------------------------------------------------------

if hasTinyMPCMatlab
    prob = TinyMPC();

    prob.setup(...
        Adyn,...
        Bdyn,...
        Q,...
        R,...
        N,...
        'rho',rho,...
        'verbose',false,...
        'adaptive_rho',false);

    prob.set_x_ref(Xref);
    prob.set_u_ref(Uref);
else
    prob = [];
    warning(['TinyMPC MATLAB interface was not found on the MATLAB path. ',...
        'Continuing with the C++ wrapper demo path. Set TINY_MPC_MATLAB_PATH ',...
        'or install tinympc-matlab for MATLAB-only solver experiments.']);
end

%% ------------------------------------------------------------------------
% Export variables
%% ------------------------------------------------------------------------

assignin('base','prob',prob);
assignin('base','N',N);
assignin('base','nx',nx);
assignin('base','nu',nu);
assignin('base','Ts',Ts);

% Build-time demo selection. Stateflow declares these as non-tunable chart
% parameters, so the generated PX4 app contains no environment-variable or
% string parsing. Unknown values fail closed to the verified hover/guidance
% configuration.
scenarioName = lower(strtrim(getenv('TINY_MPC_SCENARIO')));
switch scenarioName
    case {'virtual_wall','wall'}
        TinyMPCScenario = int32(1);
    case 'corridor'
        TinyMPCScenario = int32(2);
    case {'reduced_authority','reduced'}
        TinyMPCScenario = int32(3);
    case {'figure_eight_soc','figure8_soc','figure_eight'}
        TinyMPCScenario = int32(5);
    case {'figure_eight_box','figure8_box'}
        TinyMPCScenario = int32(6);
    otherwise
        TinyMPCScenario = int32(0);
        scenarioName = 'hover';
end

outputModeName = lower(strtrim(getenv('TINY_MPC_OUTPUT_MODE')));
if strcmp(outputModeName,'direct') || strcmp(outputModeName,'acceleration')
    TinyMPCOutputMode = int32(1);
    outputModeName = 'direct';
else
    TinyMPCOutputMode = int32(0);
    outputModeName = 'guidance';
end

assignin('base','TinyMPCScenario',TinyMPCScenario);
assignin('base','TinyMPCOutputMode',TinyMPCOutputMode);

fprintf('TinyMPC scenario: %s (%d)\n',scenarioName,TinyMPCScenario);
fprintf('TinyMPC output mode: %s (%d)\n',outputModeName,TinyMPCOutputMode);

disp('TinyMPC initialized.')
