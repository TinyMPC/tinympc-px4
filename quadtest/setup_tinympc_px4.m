function status = setup_tinympc_px4()
%SETUP_TINYMPC_PX4 Check MATLAB/Simulink/PX4 prerequisites for quadtest.

quadtestRoot = fileparts(mfilename('fullpath'));
cd(quadtestRoot);

status = struct();
status.quadtestRoot = quadtestRoot;
status.repoRoot = fileparts(quadtestRoot);
status.model = fullfile(quadtestRoot,'quadtest.slx');
status.nativeTinyMpcLibrary = fullfile(quadtestRoot,'tinympc','TinyMPC','build','src','tinympc','libtinympcstatic.a');
status.px4FirmwareRoot = getenv('PX4_DIR');
if isempty(status.px4FirmwareRoot)
    status.px4FirmwareRoot = fullfile(status.repoRoot,'third_party','PX4-Autopilot');
end
status.px4CmakeConfig = 'px4_sitl_default';

fprintf('TinyMPC-PX4 quadtest root: %s\n',quadtestRoot);
fprintf('MATLAB release: %s\n',version('-release'));

addpath(quadtestRoot);
addpath(fullfile(quadtestRoot,'wrapper'));

tinympcMatlabPath = getenv('TINY_MPC_MATLAB_PATH');
if ~isempty(tinympcMatlabPath) && exist(tinympcMatlabPath,'dir')
    addpath(genpath(tinympcMatlabPath));
    fprintf('Added TINY_MPC_MATLAB_PATH: %s\n',tinympcMatlabPath);
end

candidateTinyMpcMatlabRoots = {...
    fullfile(quadtestRoot,'tinympc-matlab'),...
    fullfile(fileparts(quadtestRoot),'tinympc-matlab'),...
    fullfile(fileparts(fileparts(quadtestRoot)),'tinympc-matlab')};

for k = 1:numel(candidateTinyMpcMatlabRoots)
    if exist(candidateTinyMpcMatlabRoots{k},'dir')
        addpath(genpath(candidateTinyMpcMatlabRoots{k}));
        fprintf('Added TinyMPC MATLAB candidate path: %s\n',candidateTinyMpcMatlabRoots{k});
    end
end

status.products = checkProducts({...
    'MATLAB',...
    'Simulink',...
    'MATLAB Coder',...
    'Simulink Coder',...
    'Embedded Coder',...
    'UAV Toolbox'});

status.hasTinyMpcMatlab = exist('TinyMPC','class') == 8 || exist('TinyMPC','file') == 2;
status.hasPx4UorbLibrary = exist('px4uORBlib','file') == 4 || exist('px4uORBlib','file') == 2;
status.hasPx4CoderTarget = ~isempty(which('codertarget.pixhawk.internal.getPX4BaseDir'));
status.hasNativeTinyMpcLibrary = exist(status.nativeTinyMpcLibrary,'file') == 2;
status.requestedPx4Version = 'v1.15.3';

printCheck('TinyMPC MATLAB interface',status.hasTinyMpcMatlab);
printCheck('PX4 uORB Simulink library',status.hasPx4UorbLibrary);
printCheck('PX4 coder target hooks',status.hasPx4CoderTarget);
printCheck('Linux TinyMPC static library',status.hasNativeTinyMpcLibrary);
fprintf('Pinned PX4 firmware version: %s\n',status.requestedPx4Version);

status.px4PreferencesConfigured = configurePx4Preferences(status);
[status.tinyMpcScenario,status.tinyMpcOutputMode] = ...
    configureBuildSelectionFromEnvironment();

try
    load_system(status.model);
    status.replacedAerospaceQuatBlock = replaceAerospaceQuatBlockIfNeeded('quadtest');
    status.patchedTinyMpcControllerBlock = patchTinyMpcControllerBlockIfNeeded('quadtest');
    status.patchedTrajectoryOutput = patchTrajectoryOutputIfNeeded('quadtest');
    configureModelCustomCode(status);
    if strcmp(get_param('quadtest','Dirty'),'on')
        save_system('quadtest');
        fprintf('[ok] Saved model after applying setup changes.\n');
    end
    status.modelLoads = true;
    fprintf('[ok] Loaded model: %s\n',status.model);
    bdclose('quadtest');
catch err
    status.modelLoads = false;
    fprintf('[missing] Model load failed: %s\n',err.message);
end
end

function patched = patchTinyMpcControllerBlockIfNeeded(model)
patched = false;
block = [model '/MATLAB Function'];

rt = sfroot;
chart = rt.find('-isa','Stateflow.EMChart','Path',block);
if isempty(chart)
    return;
end

script = tinyMpcControllerScript();
scriptPatched = false;
dataPatched = configureTinyMpcChartData(chart);

if ~strcmp(chart.Script,script)
    chart.Script = script;
    scriptPatched = true;
end

patched = scriptPatched || dataPatched;

if patched
    fprintf('[ok] Patched TinyMPC controller block (script=%d, data=%d).\n',...
        scriptPatched,dataPatched);
end
end

function patched = configureTinyMpcChartData(chart)
patched = false;

outputData = chart.find('-isa','Stateflow.Data','Scope','Output');
for k = 1:numel(outputData)
    if strcmp(outputData(k).Name,'traj_sp')
        if ~strcmp(outputData(k).Props.Array.Size,'[1 11]')
            outputData(k).Props.Array.Size = '[1 11]';
            patched = true;
        end
    end
end

parameterSpecs = {...
    'TinyMPCScenario','int32';...
    'TinyMPCOutputMode','int32'};

for k = 1:size(parameterSpecs,1)
    name = parameterSpecs{k,1};
    dataType = parameterSpecs{k,2};
    data = chart.find('-isa','Stateflow.Data','Name',name);
    if isempty(data)
        data = Stateflow.Data(chart);
        data.Name = name;
        patched = true;
    else
        data = data(1);
    end
    if ~strcmp(data.Scope,'Parameter')
        data.Scope = 'Parameter';
        patched = true;
    end
    if ~strcmp(data.DataType,dataType)
        data.DataType = dataType;
        patched = true;
    end
    if data.Tunable
        data.Tunable = false;
        patched = true;
    end
end

end

function script = tinyMpcControllerScript()
script = sprintf([...
    'function traj_sp = TinyMPC_Controller(x, xref, TinyMPCScenario, TinyMPCOutputMode)\n' ...
    '%%#codegen\n' ...
    '%% TinyMPC constrained outer loop.\n' ...
    '%%\n' ...
    '%% x is [x y z roll pitch yaw vx vy vz p q r] in PX4 NED/local frame.\n' ...
    '%% The C++ wrapper latches an engagement-relative origin and owns the\n' ...
    '%% selected scenario reference, hard state/input bounds, diagnostics,\n' ...
    '%% and fallback policy.\n' ...
    '%%\n' ...
    '%% MPC_Step_Scenario returns u0 = [ax ay az yawspeed] plus a state\n' ...
    '%% about 200 ms ahead. Guidance mode publishes the MPC plan as\n' ...
    '%% [position(1x3), velocity(1x3), acceleration(1x3), yaw, yawspeed],\n' ...
    '%% feed-forward. Direct mode publishes NaN position/velocity and u0\n' ...
    '%% acceleration, bypassing PX4 position/velocity feedback while keeping\n' ...
    '%% its acceleration-to-attitude conversion and inner loops downstream.\n' ...
    '\n' ...
    'persistent last_traj mpc_initialized\n' ...
    'if isempty(last_traj)\n' ...
    '    last_traj = zeros(1,11,''single'');\n' ...
    'end\n' ...
    'if isempty(mpc_initialized)\n' ...
    '    mpc_initialized = false;\n' ...
    'end\n' ...
    '\n' ...
    'mpc_u = zeros(4,1,''single'');\n' ...
    'x_next = zeros(12,1,''single'');\n' ...
    'diagnostics = zeros(8,1,''single'');\n' ...
    'x_single = single(x);\n' ...
    'xref_in = single(xref);\n' ...
    '\n' ...
    '%% Keep xref connected for code-generation shape inference and a future\n' ...
    '%% external-reference mode; scenarios currently own their references.\n' ...
    'if false\n' ...
    '    x_single = xref_in;\n' ...
    'end\n' ...
    '\n' ...
    'coder.cinclude(''tinympc_interface.h'');\n' ...
    'if ~mpc_initialized\n' ...
    '    coder.ceval(''MPC_Reset'');\n' ...
    '    mpc_initialized = true;\n' ...
    'end\n' ...
    '\n' ...
    'coder.ceval(''MPC_Step_Scenario'', ...\n' ...
    '            coder.rref(x_single), ...\n' ...
    '            int32(TinyMPCScenario), ...\n' ...
    '            coder.wref(mpc_u), ...\n' ...
    '            coder.wref(x_next), ...\n' ...
    '            coder.wref(diagnostics));\n' ...
    '\n' ...
    'traj_single = zeros(1,11,''single'');\n' ...
    'if TinyMPCOutputMode == int32(1)\n' ...
    '    traj_single(1:6) = single(NaN);\n' ...
    'else\n' ...
    '    traj_single(1) = x_next(1);\n' ...
    '    traj_single(2) = x_next(2);\n' ...
    '    traj_single(3) = x_next(3);\n' ...
    '    traj_single(4) = x_next(7);\n' ...
    '    traj_single(5) = x_next(8);\n' ...
    '    traj_single(6) = x_next(9);\n' ...
    'end\n' ...
    'traj_single(7) = min(max(mpc_u(1), single(-4.0)), single(4.0));\n' ...
    'traj_single(8) = min(max(mpc_u(2), single(-4.0)), single(4.0));\n' ...
    'traj_single(9) = min(max(mpc_u(3), single(-4.0)), single(4.0));\n' ...
    'traj_single(10) = x_next(6);\n' ...
    'traj_single(11) = min(max(mpc_u(4), single(-1.0)), single(1.0));\n' ...
    '\n' ...
    '%% NaN position/velocity is intentional in direct acceleration mode.\n' ...
    'valid_traj = all(isfinite(traj_single));\n' ...
    'if TinyMPCOutputMode == int32(1)\n' ...
    '    valid_traj = all(isnan(traj_single(1:6))) && ...\n' ...
    '                 all(isfinite(traj_single(7:11)));\n' ...
    'end\n' ...
    'if valid_traj\n' ...
    '    last_traj = traj_single;\n' ...
    'else\n' ...
    '    traj_single = last_traj;\n' ...
    'end\n' ...
    '\n' ...
    'traj_sp = double(traj_single);\n' ...
    '\n' ...
    'end\n']);
end

function patched = patchTrajectoryOutputIfNeeded(model)
patched = false;
sys = [model '/PX4 Low Level Controller'];

if getSimulinkBlockHandle(sys) < 0
    return;
end

messageBlock = findOneBlock(sys,'MaskType','PX4 uORB Message');
writeBlock = findOneBlock(sys,'MaskType','PX4 uORB Write');
busAssignment = findOneBlock(sys,'BlockType','BusAssignment');
demux = findOneBlock(sys,'BlockType','Demux');
inport = findOneBlock(sys,'BlockType','Inport');

if isempty(messageBlock) || isempty(writeBlock) || isempty(busAssignment) || ...
        isempty(demux) || isempty(inport)
    return;
end

patched = setParamIfDifferent(messageBlock,'uORBMsg','TrajectorySetpoint') || patched;
patched = setParamIfDifferent(messageBlock,'uORBTopicName','trajectory_setpoint') || patched;
patched = setParamIfDifferent(writeBlock,'uORBMsg','TrajectorySetpoint') || patched;
patched = setParamIfDifferent(writeBlock,'uORBTopicName','trajectory_setpoint') || patched;
patched = setParamIfDifferent(busAssignment,'AssignedSignals',...
    'position,velocity,acceleration,yaw,yawspeed') || patched;
patched = setParamIfDifferent(demux,'Outputs','[3 3 3 1 1]') || patched;

dtcNames = {...
    'Data Type Conversion13',...
    'Data Type Conversion14',...
    'Data Type Conversion15',...
    'Data Type Conversion16',...
    'Data Type Conversion17'};
dtcPositions = {...
    [190 70 250 100],...
    [190 125 250 155],...
    [190 180 250 210],...
    [190 235 250 265],...
    [190 290 250 320]};
dtcBlocks = cell(1,numel(dtcNames));

for k = 1:numel(dtcNames)
    [dtcBlocks{k},blockChanged] = ensureDataConversionBlock(...
        sys,dtcNames{k},dtcPositions{k});
    patched = blockChanged || patched;
    patched = setParamIfDifferent(dtcBlocks{k},'OutDataTypeStr','single') || patched;
end

if ~hasExpectedTrajectoryWiring(inport,demux,messageBlock,busAssignment,...
        dtcBlocks,writeBlock)
    deleteSubsystemLines(sys);

    connectBlocks(inport,1,demux,1);
    connectBlocks(messageBlock,1,busAssignment,1);

    for k = 1:numel(dtcBlocks)
        connectBlocks(demux,k,dtcBlocks{k},1);
        connectBlocks(dtcBlocks{k},1,busAssignment,k + 1);
    end

    connectBlocks(busAssignment,1,writeBlock,1);
    patched = true;
end

terminator = [sys '/Terminator5'];
if getSimulinkBlockHandle(terminator) >= 0
    delete_block(terminator);
    patched = true;
end

if patched
    fprintf('[ok] Configured PX4 Low Level Controller to publish TinyMPC TrajectorySetpoint commands.\n');
end
end

function block = findOneBlock(sys,varargin)
blocks = find_system(sys,'SearchDepth',1,varargin{:});
if isempty(blocks)
    block = '';
else
    block = blocks{1};
end
end

function changed = setParamIfDifferent(block,param,value)
changed = false;
try
    current = get_param(block,param);
catch
    current = '';
end

if ~strcmp(string(current),string(value))
    set_param(block,param,value);
    changed = true;
end
end

function [block,changed] = ensureDataConversionBlock(sys,name,position)
block = [sys '/' name];
changed = false;
if getSimulinkBlockHandle(block) < 0
    add_block('simulink/Signal Attributes/Data Type Conversion',block,...
        'Position',position);
    changed = true;
else
    changed = setParamIfDifferent(block,'Position',position);
end
end

function ok = hasExpectedTrajectoryWiring(inport,demux,messageBlock,...
        busAssignment,dtcBlocks,writeBlock)
ok = blocksConnected(inport,1,demux,1) && ...
     blocksConnected(messageBlock,1,busAssignment,1) && ...
     blocksConnected(busAssignment,1,writeBlock,1);

for k = 1:numel(dtcBlocks)
    ok = ok && blocksConnected(demux,k,dtcBlocks{k},1) && ...
         blocksConnected(dtcBlocks{k},1,busAssignment,k + 1);
end
end

function connected = blocksConnected(srcBlock,srcPort,dstBlock,dstPort)
srcPorts = get_param(srcBlock,'PortHandles');
dstPorts = get_param(dstBlock,'PortHandles');
line = get_param(srcPorts.Outport(srcPort),'Line');

if line == -1
    connected = false;
    return;
end

dstHandles = get_param(line,'DstPortHandle');
connected = any(dstHandles == dstPorts.Inport(dstPort));
end

function deleteSubsystemLines(sys)
lines = get_param(sys,'Lines');
for k = 1:numel(lines)
    if lines(k).Handle ~= -1
        delete_line(lines(k).Handle);
    end
end
end

function connectBlocks(srcBlock,srcPort,dstBlock,dstPort)
srcPorts = get_param(srcBlock,'PortHandles');
dstPorts = get_param(dstBlock,'PortHandles');
add_line(get_param(srcBlock,'Parent'),...
    srcPorts.Outport(srcPort),...
    dstPorts.Inport(dstPort),...
    'autorouting','on');
end

function replaced = replaceAerospaceQuatBlockIfNeeded(model)
replaced = false;
block = [model '/PX4 State Estimator and Trajectory Reader /Subsystem/PX4 State Estimate/Quaternions to Rotation Angles'];

if getSimulinkBlockHandle(block) < 0
    return;
end

if ~strcmp(get_param(block,'BlockType'),'EmptyBlock')
    return;
end

parent = get_param(block,'Parent');
position = get_param(block,'Position');
ports = get_param(block,'PortHandles');
inLine = get_param(ports.Inport(1),'Line');
outLine = get_param(ports.Outport(1),'Line');
srcPort = get_param(inLine,'SrcPortHandle');
dstPorts = get_param(outLine,'DstPortHandle');

delete_line(inLine);
delete_line(outLine);
delete_block(block);

add_block('simulink/User-Defined Functions/MATLAB Function',block,...
    'Position',position);

rt = sfroot;
chart = rt.find('-isa','Stateflow.EMChart','Path',block);
chart.Script = localQuatToEulerScript();

newPorts = get_param(block,'PortHandles');
add_line(parent,srcPort,newPorts.Inport(1),'autorouting','on');
for k = 1:numel(dstPorts)
    add_line(parent,newPorts.Outport(1),dstPorts(k),'autorouting','on');
end

fprintf('[ok] Replaced Aerospace Blockset Quat2Angle proxy with local MATLAB Function block.\n');
replaced = true;
end

function script = localQuatToEulerScript()
script = sprintf([...
    'function eul = fcn(q)\n' ...
    '%%#codegen\n' ...
    '%% q is PX4 vehicle_attitude.q ordered as [w x y z].\n' ...
    'qw = q(1);\n' ...
    'qx = q(2);\n' ...
    'qy = q(3);\n' ...
    'qz = q(4);\n' ...
    '\n' ...
    'sinr_cosp = 2.0 * (qw*qx + qy*qz);\n' ...
    'cosr_cosp = 1.0 - 2.0 * (qx*qx + qy*qy);\n' ...
    'roll = atan2(sinr_cosp, cosr_cosp);\n' ...
    '\n' ...
    'sinp = 2.0 * (qw*qy - qz*qx);\n' ...
    'if sinp >= 1.0\n' ...
    '    pitch = pi / 2.0;\n' ...
    'elseif sinp <= -1.0\n' ...
    '    pitch = -pi / 2.0;\n' ...
    'else\n' ...
    '    pitch = asin(sinp);\n' ...
    'end\n' ...
    '\n' ...
    'siny_cosp = 2.0 * (qw*qz + qx*qy);\n' ...
    'cosy_cosp = 1.0 - 2.0 * (qy*qy + qz*qz);\n' ...
    'yaw = atan2(siny_cosp, cosy_cosp);\n' ...
    '\n' ...
    'eul = [roll; pitch; yaw];\n']);
end

function ok = configurePx4Preferences(status)
ok = false;

if ~exist(status.px4FirmwareRoot,'dir')
    fprintf('[missing] PX4 firmware root: %s\n',status.px4FirmwareRoot);
    return;
end

px4BuildRoot = fullfile(status.px4FirmwareRoot,'build',status.px4CmakeConfig);
if ~exist(px4BuildRoot,'dir')
    fprintf('[missing] PX4 build directory: %s\n',px4BuildRoot);
    return;
end

setpref('ThirdPartyTool','FirmwarePath',status.px4FirmwareRoot);
setpref('ThirdPartyTool','CMake',status.px4CmakeConfig);
setpref('MW_PX4_AutoPilot','AutoPilotName','PX4 Host Target');

maxUorbInstances = detectMaxUorbInstances(status.px4FirmwareRoot);
setpref('MW_PX4_AutoPilot','MaxUorbInstances',maxUorbInstances);

fprintf('[ok] PX4 firmware root: %s\n',status.px4FirmwareRoot);
fprintf('[ok] PX4 CMake config: %s\n',status.px4CmakeConfig);
fprintf('[ok] PX4 max uORB instances: %d\n',maxUorbInstances);
ok = true;
end

function maxUorbInstances = detectMaxUorbInstances(px4FirmwareRoot)
maxUorbInstances = 10;
uorbHeader = fullfile(px4FirmwareRoot,'platforms','common','uORB','uORB.h');

if exist(uorbHeader,'file') ~= 2
    return;
end

contents = fileread(uorbHeader);
tokens = regexp(contents,...
    '#if\s+defined\(CONSTRAINED_MEMORY\)\s*\n[#\s]*define\s+ORB_MULTI_MAX_INSTANCES\s+(\d+)\s*\n#else\s*\n[#\s]*define\s+ORB_MULTI_MAX_INSTANCES\s+(\d+)',...
    'tokens','once');

if ~isempty(tokens)
    maxUorbInstances = str2double(tokens{2});
end
end

function configureModelCustomCode(status)
includeDirs = sprintf(['./wrapper\n' ...
    './tinympc/TinyMPC/src\n' ...
    './tinympc/TinyMPC/src/tinympc\n' ...
    './tinympc/TinyMPC/include/Eigen']);
sourceFile = sprintf(['wrapper/tinympc_interface.cpp\n' ...
    'tinympc/TinyMPC/src/tinympc/admm.cpp\n' ...
    'tinympc/TinyMPC/src/tinympc/tiny_api.cpp\n' ...
    'tinympc/TinyMPC/src/tinympc/rho_benchmark.cpp']);

set_param('quadtest',...
    'SimUserSources',sourceFile,...
    'SimUserIncludeDirs',includeDirs,...
    'SimUserLibraries','',...
    'CustomSource',sourceFile,...
    'CustomInclude',includeDirs,...
    'CustomLibrary','');

fprintf('[ok] Applied platform custom-code settings to loaded model.\n');
end

function [scenario,outputMode] = configureBuildSelectionFromEnvironment()
scenarioName = lower(strtrim(getenv('TINY_MPC_SCENARIO')));
switch scenarioName
    case {'virtual_wall','wall'}
        scenario = int32(1);
    case 'corridor'
        scenario = int32(2);
    case {'reduced_authority','reduced'}
        scenario = int32(3);
    case {'figure_eight_soc','figure8_soc','figure_eight'}
        scenario = int32(5);
    case {'figure_eight_box','figure8_box'}
        scenario = int32(6);
    otherwise
        scenario = int32(0);
end

outputModeName = lower(strtrim(getenv('TINY_MPC_OUTPUT_MODE')));
if strcmp(outputModeName,'direct') || strcmp(outputModeName,'acceleration')
    outputMode = int32(1);
else
    outputMode = int32(0);
end

assignin('base','TinyMPCScenario',scenario);
assignin('base','TinyMPCOutputMode',outputMode);
end

function products = checkProducts(names)
installed = ver;
installedNames = {installed.Name};
products = struct();

for k = 1:numel(names)
    fieldName = matlab.lang.makeValidName(names{k});
    products.(fieldName) = any(strcmp(installedNames,names{k}));
    printCheck(names{k},products.(fieldName));
end
end

function printCheck(label,ok)
if ok
    fprintf('[ok] %s\n',label);
else
    fprintf('[missing] %s\n',label);
end
end
