function status = run_tinympc_px4_demo(action)
%RUN_TINYMPC_PX4_DEMO Configure and run the TinyMPC-PX4 Simulink demo.
%
% action:
%   "update"  update/compile the Simulink diagram (default)
%   "build"   generate and install PX4 target module sources with slbuild
%   "sim"     run a short normal-mode simulation
%   "open"    configure and open the model

if nargin < 1 || isempty(action)
    action = "update";
else
    action = string(action);
end

quadtestRoot = fileparts(mfilename('fullpath'));
cd(quadtestRoot);

status = setup_tinympc_px4();
model = 'quadtest';

if ~status.modelLoads
    error('TinyMPCPX4:ModelLoadFailed','Model did not load during setup_tinympc_px4.');
end

load_system(model);
init_tinympc_quad;
configureDemoTiming(model);
if strcmp(get_param(model,'Dirty'),'on')
    save_system(model);
end

switch lower(action)
    case "open"
        open_system(model);
        fprintf('[ok] Opened %s.\n',model);

    case "update"
        set_param(model,'SimulationCommand','update');
        fprintf('[ok] Updated %s diagram.\n',model);

    case "sim"
        simOut = sim(model,'StopTime','0.1'); %#ok<NASGU>
        fprintf('[ok] Ran short normal-mode simulation for %s.\n',model);

    case "build"
        slbuild(model);
        fprintf('[ok] Generated PX4 target module sources for %s.\n',model);

    otherwise
        error('TinyMPCPX4:UnknownAction',...
            'Unknown action "%s". Use update, build, sim, or open.',action);
end

status.action = action;
end

function configureDemoTiming(model)
sampleTime = '0.02';
if evalin('base','exist(''Ts'',''var'')') == 1
    sampleTime = num2str(evalin('base','Ts'),'%.15g');
end

set_param(model,...
    'StopTime','inf',...
    'SolverType','Fixed-step',...
    'FixedStep',sampleTime);

uorbBlocks = find_system(model,...
    'LookUnderMasks','all',...
    'FollowLinks','on',...
    'RegExp','on',...
    'MaskType','PX4 uORB (Read|Message)');

for k = 1:numel(uorbBlocks)
    params = get_param(uorbBlocks{k},'DialogParameters');
    if isfield(params,'SampleTime')
        set_param(uorbBlocks{k},'SampleTime',sampleTime);
    end
end

fprintf('[ok] Configured fixed step and PX4 uORB sample times to %s s.\n',sampleTime);
end
