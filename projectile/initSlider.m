
function initSlider(hplot, proj)
    % angle
    hAngle = uicontrol('style','slider','units','pixel', 'position',[20 5 250 20]);
    labelAngle = uicontrol('style', 'text','String', 'Angle (deg)', 'position',[20 30 250 15]);

    % velocity
    hVel = uicontrol('style','slider','units','pixel', 'position',[280 5 250 20]);
    labelVel = uicontrol('style', 'text','String', 'Velocity (m/s)', 'position',[280 30 250 15]);


    addlistener(hAngle,'ContinuousValueChange',@(hObject, event) cbAngle(hObject, event, hplot, proj, hVel));
    addlistener(hVel,'ContinuousValueChange',@(hObject, event) cbVel(hObject, event, hplot, proj, hAngle));
end


%% Angle Slider
function cbAngle(hObject, event, hplot, p, hVel)
    v = get(hVel, 'Value') * 5;
    theta = get(hObject,'Value') * 90;
    

    xi = [0, 0.4];
    vi = [v*cosd(theta), v*sind(theta)];
    [x, y, t] = p.simulatep(xi, vi, 4);

    disp(['V = ', num2str(v)]);
    disp(['Theta = ', num2str(theta)]);

    set(hplot,'xdata', x);
    set(hplot, 'ydata', y);
    drawnow;
end

%% Velocity slider
function cbVel(hObject, event, hplot, p, hAngle)
    v = get(hObject,'Value') * 5;
    theta = get(hAngle,'Value') * 90;

    xi = [0, 0.4];
    vi = [v*cosd(theta), v*sind(theta)];
    [x, y, t] = p.simulatep(xi, vi, 4);

    disp(['V = ', num2str(v)]);
    disp(['Theta = ', num2str(theta)]);

    set(hplot,'xdata', x);
    set(hplot, 'ydata', y);
    drawnow;
end