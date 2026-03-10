%% Manual PID Tuning Function

function tune_pid_manually(motor_tf)
    figure('Name', 'Manual PID Tuning', 'NumberTitle', 'off', 'Position', [100, 100, 800, 600]);
    Kp = 10;
    Ki = 5;
    Kd = 0.1;
    
  
    uicontrol('Style', 'text', 'Position', [50, 500, 100, 20], 'String', 'Kp:');
    Kp_slider = uicontrol('Style', 'slider', 'Position', [150, 500, 200, 20], ...
        'Min', 0, 'Max', 50, 'Value', Kp, 'Callback', @update_plot);
    
    uicontrol('Style', 'text', 'Position', [50, 450, 100, 20], 'String', 'Ki:');
    Ki_slider = uicontrol('Style', 'slider', 'Position', [150, 450, 200, 20], ...
        'Min', 0, 'Max', 50, 'Value', Ki, 'Callback', @update_plot);
    
    uicontrol('Style', 'text', 'Position', [50, 400, 100, 20], 'String', 'Kd:');
    Kd_slider = uicontrol('Style', 'slider', 'Position', [150, 400, 200, 20], ...
        'Min', 0, 'Max', 5, 'Value', Kd, 'Callback', @update_plot);
    
  
    ax = axes('Position', [0.1, 0.2, 0.8, 0.5]);
    
   
    update_plot();
    
    function update_plot(~, ~)
      
        Kp = get(Kp_slider, 'Value');
        Ki = get(Ki_slider, 'Value');
        Kd = get(Kd_slider, 'Value');
        
     
        C = pid(Kp, Ki, Kd);
        
      
        closed_loop = feedback(C * motor_tf, 1);
        
       
        step(closed_loop, 5);
        grid on;
        title(sprintf('PID Response: Kp=%.2f, Ki=%.2f, Kd=%.2f', Kp, Ki, Kd));
        xlabel('Time (seconds)');
        ylabel('Speed (rad/s)');
    end
end