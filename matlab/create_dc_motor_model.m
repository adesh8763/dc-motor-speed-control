%% DC Motor Simulink Model Builder


function create_dc_motor_model_main()

    model_name = 'DC_Motor_Simulink';

    % Close any existing copy cleanly
    if ~isempty(find_system('type','block_diagram','Name',model_name))
        close_system(model_name, 0);
    end

    new_system(model_name);
    open_system(model_name);
    set_param(model_name, 'Solver','ode45', 'StopTime','5');

    %--------------------------------------------------------------
    % Add all top-level blocks
    %--------------------------------------------------------------

    % Reference step input
    add_block('simulink/Sources/Step', [model_name '/Ref_Speed']);
    set_step_params([model_name '/Ref_Speed'], '0', '0', '100');

    % Error summer  (+ref, -feedback)
    add_block('simulink/Math Operations/Sum', [model_name '/Err_Sum']);
    set_param([model_name '/Err_Sum'], 'Inputs','+-');

    % PID controller subsystem
    create_pid_subsystem(model_name);

    % DC motor subsystem
    create_motor_subsystem(model_name);

    % Load disturbance step
    add_block('simulink/Sources/Step', [model_name '/Load_Torque']);
    set_step_params([model_name '/Load_Torque'], '2', '0', '0.5');

    % Disturbance summer (control + disturbance)
    add_block('simulink/Math Operations/Sum', [model_name '/Dist_Sum']);
    set_param([model_name '/Dist_Sum'], 'Inputs','++');

    % Mux to combine signals for scope
    add_block('simulink/Signal Routing/Mux', [model_name '/Mux']);
    set_param([model_name '/Mux'], 'Inputs','2');

    % Scope (single input via Mux - avoids NumInputPorts version issues)
    add_block('simulink/Sinks/Scope', [model_name '/Scope']);

    %--------------------------------------------------------------
    % Layout
    %--------------------------------------------------------------
    try
        set_param([model_name '/Ref_Speed'],  'Position',[ 30 130 110 160]);
        set_param([model_name '/Err_Sum'],    'Position',[170 155 210 195]);
        set_param([model_name '/PID_Ctrl'],   'Position',[260 130 380 230]);
        set_param([model_name '/Load_Torque'],'Position',[260 280 360 310]);
        set_param([model_name '/Dist_Sum'],   'Position',[420 155 460 195]);
        set_param([model_name '/DC_Motor'],   'Position',[500 130 620 230]);
        set_param([model_name '/Mux'],        'Position',[670 140 700 200]);
        set_param([model_name '/Scope'],      'Position',[730 155 770 195]);
    catch
    end

    %--------------------------------------------------------------
    % Wire top-level connections
    %--------------------------------------------------------------
    connect(model_name, 'Ref_Speed/1',  'Err_Sum/1');
    connect(model_name, 'DC_Motor/1',   'Err_Sum/2');
    connect(model_name, 'Err_Sum/1',    'PID_Ctrl/1');
    connect(model_name, 'PID_Ctrl/1',   'Dist_Sum/1');
    connect(model_name, 'Load_Torque/1','Dist_Sum/2');
    connect(model_name, 'Dist_Sum/1',   'DC_Motor/1');
    connect(model_name, 'DC_Motor/1',   'Mux/1');
    connect(model_name, 'Ref_Speed/1',  'Mux/2');
    connect(model_name, 'Mux/1',        'Scope/1');

    try, set_param(model_name,'SimulationCommand','update'); catch, end

    save_system(model_name);
    fprintf('\nModel "%s" created successfully.\n', model_name);
    fprintf('  PID gains : Kp=10, Ki=5, Kd=0.1\n');
    fprintf('  Motor     : electrical 1/(0.5s+1), mechanical 1/(0.01s+0.1)\n');
    fprintf('  Disturbance step at t=2s\n');
    fprintf('Run simulation: sim(''%s'')\n\n', model_name);
end

%===========================================================
function create_pid_subsystem(model_name)
% Builds:  u = Kp*e + Ki*integral(e) + Kd*derivative(e)
%===========================================================
    p = [model_name '/PID_Ctrl'];
    add_block('simulink/Ports & Subsystems/Subsystem', p);
    clean_subsystem(p);   % remove default In1/line/Out1

    add_block('simulink/Ports & Subsystems/In1',  [p '/e']);
    add_block('simulink/Ports & Subsystems/Out1', [p '/u']);

    add_block('simulink/Math Operations/Gain',    [p '/Kp']);
    set_param([p '/Kp'], 'Gain','10');

    add_block('simulink/Continuous/Integrator',   [p '/Int']);
    add_block('simulink/Math Operations/Gain',    [p '/Ki']);
    set_param([p '/Ki'], 'Gain','5');

    add_block('simulink/Continuous/Derivative',   [p '/Der']);
    add_block('simulink/Math Operations/Gain',    [p '/Kd']);
    set_param([p '/Kd'], 'Gain','0.1');

    add_block('simulink/Math Operations/Sum',     [p '/PID_Sum']);
    set_param([p '/PID_Sum'], 'Inputs','+++');

    try
        set_param([p '/e'],       'Position',[ 30 173  60 207]);
        set_param([p '/Kp'],      'Position',[130  70 180 100]);
        set_param([p '/Int'],     'Position',[130 150 180 180]);
        set_param([p '/Ki'],      'Position',[230 150 280 180]);
        set_param([p '/Der'],     'Position',[130 230 180 260]);
        set_param([p '/Kd'],      'Position',[230 230 280 260]);
        set_param([p '/PID_Sum'], 'Position',[330 120 365 270]);
        set_param([p '/u'],       'Position',[420 183 450 217]);
    catch
    end

    connect(p, 'e/1',      'Kp/1');
    connect(p, 'e/1',      'Int/1');
    connect(p, 'e/1',      'Der/1');
    connect(p, 'Int/1',    'Ki/1');
    connect(p, 'Der/1',    'Kd/1');
    connect(p, 'Kp/1',    'PID_Sum/1');
    connect(p, 'Ki/1',    'PID_Sum/2');
    connect(p, 'Kd/1',    'PID_Sum/3');
    connect(p, 'PID_Sum/1','u/1');
end

%===========================================================
function create_motor_subsystem(model_name)
% DC motor: electrical + mechanical TF with back-EMF loop
%===========================================================
    p = [model_name '/DC_Motor'];
    add_block('simulink/Ports & Subsystems/Subsystem', p);
    clean_subsystem(p);   % remove default In1/line/Out1

    add_block('simulink/Ports & Subsystems/In1',  [p '/V_in']);
    add_block('simulink/Ports & Subsystems/Out1', [p '/omega']);

    add_block('simulink/Math Operations/Sum',     [p '/V_sum']);
    set_param([p '/V_sum'], 'Inputs','+-');

    add_block('simulink/Continuous/Transfer Fcn', [p '/Elec']);
    set_param([p '/Elec'], 'Numerator','[1]', 'Denominator','[0.5 1]');

    add_block('simulink/Math Operations/Gain',    [p '/Kt']);
    set_param([p '/Kt'], 'Gain','0.01');

    add_block('simulink/Continuous/Transfer Fcn', [p '/Mech']);
    set_param([p '/Mech'], 'Numerator','[1]', 'Denominator','[0.01 0.1]');

    add_block('simulink/Math Operations/Gain',    [p '/Kb']);
    set_param([p '/Kb'], 'Gain','0.01');

    try
        set_param([p '/V_in'],  'Position',[ 30 113  60 137]);
        set_param([p '/V_sum'], 'Position',[110 105 145 145]);
        set_param([p '/Elec'],  'Position',[185  95 255 145]);
        set_param([p '/Kt'],    'Position',[295  95 345 135]);
        set_param([p '/Mech'],  'Position',[385  95 460 145]);
        set_param([p '/omega'], 'Position',[550 108 580 132]);
        set_param([p '/Kb'],    'Position',[385 200 445 240]);
    catch
    end

    connect(p, 'V_in/1', 'V_sum/1');
    connect(p, 'Kb/1',   'V_sum/2');       % back-EMF into negative port
    connect(p, 'V_sum/1','Elec/1');
    connect(p, 'Elec/1', 'Kt/1');
    connect(p, 'Kt/1',   'Mech/1');
    connect(p, 'Mech/1', 'omega/1');       % forward: speed output
    connect(p, 'Mech/1', 'Kb/1');          % branch: back-EMF feedback
end

%===========================================================
function clean_subsystem(p)

    % Step 1 – delete the feedthrough line by handle (avoids string-path issues)
    try
        line_handles = get_param(p, 'Lines');
        for k = 1:length(line_handles)
            delete_line(line_handles(k));
        end
    catch
    end

    % Step 2 – delete default port blocks
    try, delete_block([p '/In1']);  catch, end
    try, delete_block([p '/Out1']); catch, end
end

%===========================================================
function connect(sys, src, dst)
% Thin wrapper around add_line that gives a clear error message
% identifying exactly which connection failed.
%===========================================================
    try
        add_line(sys, src, dst, 'autorouting','on');
    catch ME
        error('Failed to connect %s -> %s in %s:\n%s', src, dst, sys, ME.message);
    end
end

%===========================================================
% Version-safe Step block parameter setter.
%   Older Simulink : 'Time' / 'Before' / 'After'
%   Newer Simulink : 'StepTime' / 'InitialValue' / 'FinalValue'
%===========================================================
function set_step_params(blk, t_step, v_init, v_final)
    names = fieldnames(get_param(blk, 'ObjectParameters'));
    if any(strcmp(names, 'StepTime'))
        set_param(blk, 'StepTime',t_step, 'InitialValue',v_init, 'FinalValue',v_final);
    else
        set_param(blk, 'Time',t_step, 'Before',v_init, 'After',v_final);
    end
end

% Entry point
fprintf('Creating DC Motor Simulink Model...\n');
create_dc_motor_model_main();
fprintf('Done!\n');