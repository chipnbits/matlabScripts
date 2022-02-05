% A simulation environment to test out different configurations for the
% Mech 223 orbital launcher design project

%Board Dimensions and constants
BD.area = [0 7 0 2.5]; %m
BD.earthRad = .2; %m
BD.earthLPos = [BD.earthRad ,.5]; %m
BD.earthRPos = [7-BD.earthRad,.5]; %m
BD.titanPos = [3.5 1.9]; %m
BD.titanRad = .45; %m
BD.titanInnerRad = .15; %m 
BD.exitHoleRad = .025; %m
BD.g = 9.81; %m/s^2
BD.mball = .025; %mass of rolling ball in kilos
BD.funnelFric = .04;
BD.orbiterBrakeFriction = .2; %friction coefficient for braking action of orbiter
BD.orbiterBrakeDistance = 8;  %distance at which to brake orbiter

% Orbiter parameters
OB.Lvel = 1; %m/s exit speed relative to the orbiter
OB.m = 0.8; %kg
OB.dim = [.25 .15]; %dimensions in meters, xy
OB.pos0 = [BD.earthLPos+[BD.earthRad -.5*OB.dim(2)] , OB.dim]; %m
OB.cannonOffset = [.5*OB.dim(1) 0]; %offset of the cannon gfx from the orbiter position
OB.pathMarkerTimeDistance = .2; %The distance in seconds between orbiter path markers

%Differentiate h(r) w.r.t. r for second function inside funnel
% This is the origin of the funnelOde equations
syms r
z1 = -(r^2-0.913*r+0.209)/(4.93*r^2+r+.36);
dz1dr = diff(z1);
ddz1dr = diff(dz1dr);

% Instantiate a GUI interface and store details in GUI structure
GUI = buildUI(BD, OB);

% Play with the GUI in main with button/field listeners for user input
% Pass the GUI, BD, OB data and handles, and pointers for landers and
% orbiters.  
% landers - array of all the lander objects that have been placed
% orbiters - array of all the orbiters that have been tracked
main(GUI,BD, OB, [], []);

function main(GUI,BD, OB, landers, orbiters)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% BUTTON CONTROLS %%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Create a function execution for the lines toggle button
GUI.button.lines.ValueChangedFcn = @toggleLines;
% Create a function execution for the reset button
GUI.button.reset.ButtonPushedFcn = @resetBoard; 
% Create a function execution for the launch button
GUI.button.launch.ButtonPushedFcn = @launchLander; 
% Create a function execution for the play button
GUI.button.play.ButtonPushedFcn = @playback;
% Create a function execution for the orbiter button
GUI.button.orbiter.ButtonPushedFcn = @launchOrbiter;

%Set the cursor to automatically update when text fields are changed
GUI.field.xpos.ValueChangedFcn = @updateCursor;
GUI.field.ypos.ValueChangedFcn = @updateCursor;
GUI.field.vel.ValueChangedFcn = @updateCursor;
GUI.field.ang.ValueChangedFcn = @updateCursor;

    %Calculate and scale the new cursor vector to display the user selected
    %fields
    function updateCursor(btn, event)
        % scaling on the vector arrow
        cursorScale = .5;
        
        %update arrow information
        x = GUI.field.xpos.Value; y = GUI.field.ypos.Value;
        v = GUI.field.vel.Value; ang = GUI.field.ang.Value;
        u = cosd(ang)*v;
        w = sind(ang)*v;
        
        set(GUI.World.gfx2d.cursor, 'XData', x, 'YData', y);
        set(GUI.World.gfx2d.cursor, 'UData', u*cursorScale,...
            'VData', w*cursorScale);
        % redraw the figure with updated cursor
        drawnow
    end

    %Turn the board guidelines on or/off as desired for both 3d and 2d mesh
    function toggleLines(btn, event)
        if (btn.Value == true)
            set(GUI.World.gfx2d.lineHandles, 'Visible', 'on')
            set(GUI.World.gfx3d.lineHandles, 'Visible', 'on')
        else
            set(GUI.World.gfx2d.lineHandles, 'Visible', 'off')
            set(GUI.World.gfx3d.lineHandles, 'Visible', 'off')
        end    
    end

    % Reset the board by removing lander plots from board but retain the
    % lander telemetry data in memory
    function resetBoard(btn, event)
        for L = landers
            set(L.plot2d,'Visible','off');
            set(L.plot3d,'Visible','off');
        end        
        for O = orbiters
          set(O.plot2d.line,'Visible','off');  
          set(O.plot2d.markers,'Visible','off');  
        end        
        set(GUI.World.gfx2d.orbiter, 'Visible', 'off');  
        set(GUI.World.gfx2d.landerTube, 'Visible', 'off'); 
    end    

    % Calculate the trajectory of a lander using ode45, plot the
    % information in an overlay on GUI, store the telemetry data to
    % workspace
    function launchLander(btn, event)
        makeLander(GUI.field.xpos.Value, GUI.field.ypos.Value,...
            GUI.field.vel.Value*cosd(GUI.field.ang.Value),...
            GUI.field.vel.Value*sind(GUI.field.ang.Value), 0);
    end
    function makeLander(x0, y0, vx0,vy0,t0)
        L.x0 = x0;
        L.y0 = y0;
        L.vx0 = vx0;
        L.vy0 = vy0;
        L.t0 = t0;
        L.status = "active";
        L.surface = "Stayed in contact";
        L.funnelentry = [];
        L.telemetry = [];
        [L.theta0, L.r0] = cart2titan(L.x0,L.y0);
        
        % Recursively calculate the trajectory until boundary termination 
        % conditions have been met (out of bounds or a long time)
        % ** - Store the trajectory in the telemetry information, where
        % L.telemetry (time, vx, vy, vz, x, y, z) cartesian position and velocity
        if (L.r0 > BD.titanRad)
            tableTrajectory(L.x0, L.y0, L.vx0, L.vy0, L.t0)
        else
            funnelTrajectory(L.x0, L.y0, L.vx0, L.vy0, L.t0)
        end        
        
        %Store the aqcuired telemetry to the lander handle
        L.telemetry(:,7)= boardSurf(rTitan(L.telemetry(:,5),L.telemetry(:,6),BD));
        
        % Plot the trajectory over the surface
        plotTrajectory();  
        
        % Warn user if the lander left the surface of the board
        if (L.surface ~= "Stayed in contact")
            disp("Warning: the lander became airborne") 
        end
               
        % Add any particular details about new features plotted in the GUI
        % to an array object holding all the instantiated Landers
        % (unpolished object oriented programming)
        landers = [landers L];
        assignin('base','landerData',landers);
        
        function tableTrajectory(x0, y0, vx0, vy0, t0)
            % Use ode45 to find a trajectory with a helper function to make
            % ode45 stop when a termination condition has been reached  
            % Set the ode45 function with parameters:
            %  tableOde() - A vector function 
            %         u' = [ax, ay, az, vx, vy, vz] = F(vx, vy, vz, px, py, pz)
            %  tspan - time over which to analyse solution
            %  u0 - the initial starting parameters (given from UI fields)

            % Set the ode45 options (bounds and termination condition)
            opt = odeset('Events', @outOfBounds); 
            % Set the time interval over which to evaluate
            tmax = 30;
            tspan = linspace(t0,tmax,1000);             
            % Place the lander in an initial x,y position on the board surface
            % With velocity vx vy and zero vertical velocity
            u0 = [vx0; vy0; 0; x0; y0; 0];        
            % Calculate trajectory and return u = [vx, vy, vz, px, py, pz]
            [T, U] = ode45(@(t,u) tableOde(t,u), tspan, u0, opt); 

            %Check if path ran into funnel and truncate at the funnel
            %before passing-- this is because ode45 does not properly check
            %for seperate event cases and can backtrack its calculation to a
            %point that misses the fact that the object has entered the
            %funnel system
            R = rTitan(U(:,4),U(:,5),BD);
            index = find(R < .45,1);
            if(index > 1)
                T = T(1:index);
                U = U(1:index,:);
            end
            
            % Append the new trajectory data onto any prexisting path
            % information
            L.telemetry = [L.telemetry ; T U];

            % Check the conditions on which the trajectory ended to
            % decide the next step to take, record the exit position
            % and velocity in exitData to pass to next function.
            % exitData: x, y, vx, vy 
            exitData = [U(end,4) U(end,5) U(end,1) U(end,2)];
            updateStatus(exitData)            
            
            if (L.status == "active")
               %Remove last line of telemetry which is used as the seed
               %for the next trajectory (exitData) -- z=0 and vz=0
               %assumed
               funnelTrajectory(exitData(1), exitData(2),...
                   exitData(3), exitData(4), T(end)) 
            end

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %Kinematic description of flat table  %%%
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            function dudt = tableOde(t,u)
                %No acceleration on table top                    
                a = [0 0 0];                    
                dudt = [  a(1) a(2) a(3) u(1) u(2) u(3)]';                    
            end

            % A function to determine if the lander has gone out of bounds
            % from table top area or stopped
            function [value, isterminal, direction] = outOfBounds(t, u)
                x = u(4); y=u(5); vx = u(1); vy = u(2);
                [th, r] = cart2titan(x,y);
                % set trigger if x y value is outside of board area or
                % stopped, or has fallen into funnel 
                % Check the three termination cases, integration will
                % halt when value = 0 and isterminal = 1
                value = ~(offtable(x,y,vx,vy) |...
                        (norm(vx,vy) <1e-4)|...
                        r < BD.titanRad);
                isterminal = 1;
                direction = 0;
            end    

            function updateStatus(data)
                x = data(1); y=data(2); vx = data(3); vy = data(4);
                
                if (offtable(x,y,vx,vy))
                    L.status = "offtable";
                elseif (norm([vx,vy]) <1e-4) %Slowed to a stop
                    L.status = "stopped";
                elseif (T(end)>=tmax)
                    L.status = "overtime";
                end
            end

            function bool = offtable(x,y,vx,vy)
                epsilon = .01;
                bool = x < (BD.area(1)-epsilon) | x > (BD.area(2)+epsilon)...
                | y < (BD.area(3)-epsilon) | y> (BD.area(4)+epsilon);               
            end
        end
        
        function funnelTrajectory(x0, y0, vx0, vy0, t0)
            % Determine all of the entry conditions to the funnel, record
            % to L data
            p = x0 - BD.titanPos(1); %convert to distances from titan center
            q = y0 - BD.titanPos(2);
            [theta0, r0] = cart2titan(x0,y0);
            dtheta0 = 1/r0^2*(-q*vx0+p*vy0);
            dr0 = 1/r0*(p*vx0+q*vy0);
            z0 = 0;
            dz0 = 0;
            
            vars.m = 7/5*BD.mball; %adjusting for a rolling ball Hamiltonian
            vars.g = BD.g; %use system setting for gravity
            
            %calculate initial angular momentum
            lang0 = vars.m*r0^3*dtheta0;
            
            %Record entrance conditions and details to L
            L.funnelentry.theta0 = theta0;
            L.funnelentry.r0     = r0;
            L.funnelentry.dtheta0 = dtheta0;
            L.funnelentry.dr0     = dr0;
            L.funnelentry.lang0   = lang0;
            
            % Calculate the difeq constants
            c1 = 5*vars.g/7;
            c2 = (lang0/vars.m)^2;
            vars.c1 = c1;
            vars.c2 = c2;
            vars.l = lang0;
            
            %The solution is a second order differential equation
            % Use ode45 to find a trajectory with a helper function to make
            % ode45 stop when a termination condition has been reached  
            % Set the ode45 function with parameters:
            %  vectorField() - A vector function 
            %         u' = [r', r'', th', th''] = F(r, r', th, th')
            %  u1 = r and u2 = r'
            %  tspan - time over which to analyse solution
            %  u0 - the initial starting parameters (given from UI fields)
            tmax = 30;
            tspan = linspace(t0,tmax,10000);  
            u0 = [r0, dr0, theta0, dtheta0];
            % Set the ode45 options (bounds and termination condition)
            opt = odeset('Events', @outOfBounds); 

            [T, U] = ode45(@(t,u) funnelOde(t,u, vars), tspan, u0, opt);
            R = U(:,1); DR = U(:,2); TH = U(:,3); DTH = U(:,4);
            
            % solve for height
            Z = boardSurf(R);

            %Convert trajectory back to x y z coordinates
            [X,Y] = pol2cart(TH,R);
            DX = (DR.*cos(TH) - R.*sin(TH).*DTH);
            DY = (DR.*sin(TH) + R.*cos(TH).*DTH);            
            DZ = zeros(size(DX));

            %convert back to board coordinates
            X = X+BD.titanPos(1);
            Y = Y+BD.titanPos(2);
            
            path = [T,DX,DY,DZ,X,Y,Z];

             L.telemetry = [L.telemetry ; path];

            % Check the conditions on which the trajectory ended to
            % decide the next step to take, record the exit position
            % and velocity in exitData to pass to next function.
            % exitData: x, y, vx, vy 
            exitData = [path(end,5) path(end,6) path(end,2) path(end,3)];
            updateStatus(exitData)
            if (L.status == "active")
               %Remove last line of telemetry which is used as the seed
               %for the next trajectory (exitData) -- z=0 and vz=0
               %assumed
               tableTrajectory(exitData(1), exitData(2),...
                   exitData(3), exitData(4), T(end)) 
            end

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %Kinematic description of funnel system %
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            function dudt = funnelOde(t,u, vars)
                r=u(1);
                dr=u(2);
                th=u(3);
                dth=u(4);

                %case 1
                if (r < .15)
                    dzdr=2.03;
                    ddzdr=0;
                else

                    dzdr = (((493*r)/50 + 1)*(r^2 - (913*r)/1000 + 209/1000))...
                        /((493*r^2)/100 + r + 9/25)^2 - ...
                        (2*r - 913/1000)/((493*r^2)/100 + r + 9/25);
                    ddzdr = (493*(r^2 - (913*r)/1000 + 209/1000))/...
                        (50*((493*r^2)/100 + r + 9/25)^2) -...
                        2/((493*r^2)/100 + r + 9/25) + ...
                        (2*((493*r)/50 + 1)*(2*r - 913/1000))/((493*r^2)/100 +...
                        r + 9/25)^2 - (2*((493*r)/50 + 1)^2*...
                        (r^2 - (913*r)/1000 + 209/1000))/((493*r^2)/100 + r + 9/25)^3;
                end  
                
                %Complete frictionless calcultions then adjust for friction
                
                %Friction added attempt
                m=vars.m;
                
                ddth = -2*dr/r*dth;
                ddr = 1/(1+dzdr^2)*(-dzdr*ddzdr*dr^2 - BD.g*dzdr + r*dth^2);                 
                ddzdt = ddzdr*dr^2+dzdr*ddr;
                
                %Find v-unit vector
                vr = dr;
                vth = r*th;
                vz = dzdr*dr;
                v = [vr vth vz];
                vunit = v/norm(v);                
                
                %From equations of motion
                ar = ddr-r*dth^2;
                ath = r*ddth + 2*dr*dth;
                az = ddzdt; 
                a = [ar ath az];               
                
                %add friction
                mu = .04;
                
                N = m*norm(a + [0 0 BD.g]);
                fric = mu*N*(-vunit);
                
                %recalculate with friction added
                ddth=1/r*(fric(2)/m-2*dr*dth);
                
                sinphi = dzdr/(dzdr^2+1)^.5;
                ddr = 1/m*(-N*sinphi+1*fric(1))+r*dth^2;
                
                if (ddzdt<=-BD.g)
                    L.surface = "Left the surface";
                end
                         
%                 %other model
%                 %add rolling friction
%                 % calculate normal force first, find magnitude of
%                 % acceleration, to get 
%                 mu = .04;                
%                 a = norm([ddth ddr ddzdt]);                
%                 aFricMag = mu*a; %accel due to friction magnitude
%                 
%                 % since the friction opposes the direction of motion we
%                 % need to find the normalized velocity vector in
%                 % cylindrical coordinates [th, r, z]'
%                 dzdt=dzdr*dr;
%                 
%                 vunit = [dth dr dzdt]/norm([dth dr dzdt]);
%                 aFric = -aFricMag*vunit;  %opposing velocity
% 
%                 %only r and th are tracked due to surface constraint
%                 ddr = ddr + aFric(2);
%                 ddth = ddth + aFric(1);               

                dudt = [dr ddr dth ddth]'; 
                 
                 
            end

            % A function to determine if the lander has gone out of bounds
            % from table top area or stopped
            function [value, isterminal, direction] = outOfBounds(t, u)
                r = u(1); 

                % set trigger if the ball leaves the funnel through the
                % objective landing hole or out the top
                value = and(r < BD.titanRad, r > BD.exitHoleRad);
                isterminal = 1;
                direction = 0;
                               
            end    

            function updateStatus(data)
                x = data(1); y=data(2); vx = data(3); vy = data(4);
                [th,r] = cart2titan(x,y);
                if (r >= BD.titanRad)
                    L.status = "active";
                else
                    L.status = "landed";
                end
            end
         
            function [th,r] = cart2titan(a,b)
            [th,r] = cart2pol(a-BD.titanPos(1),b-BD.titanPos(2));
        end
        end
  
        function plotTrajectory()
            X = L.telemetry(:,5); Y = L.telemetry(:,6);
            R = rTitan (X,Y,BD);
            offset = .002; %lift the thicker line partially off surface for visibility
            Z = boardSurf(R)+offset;
                
        %plot the info over the gui interface, save plot handle to L for later
        %deletion if required
        hold( GUI.board, 'on' )
        axis( GUI.board,'manual')
        L.plot2d = plot(GUI.board, X,Y, '-o'); 
        hold( GUI.board, 'off' )
        
        % layover surface 3d plot
        hold( GUI.surf, 'on' )
        axis( GUI.surf,'manual')
        L.plot3d = plot3(GUI.surf, X,Y, Z);
        L.plot3d.LineWidth = 5;
        hold( GUI.surf, 'off' )            
        end
        
        function [th,r] = cart2titan(a,b)
            [th,r] = cart2pol(a-BD.titanPos(1),b-BD.titanPos(2));
        end
    end

    % Animate a playback for the trajectory from the lander telemtry    
    function playback(btn, event)
        
    if (isempty(landers))
            disp("No Landers Launched")            
    else
        L = landers(end);        
        T = L.telemetry(:,1);
        X = L.telemetry(:,5);
        Y = L.telemetry(:,6);
        Z = L.telemetry(:,7);
        Vx = L.telemetry(:,2);
        Vy = L.telemetry(:,3);
        oLaunch = false;  %Boolean to classify as an orbiter launch
        
        %Check for launch from orbiter (t0 will be > 0)
        if (T(1) ~=0 )
            oLaunch = true;
            O = orbiters(end);
            oT = O.path(:,1);
            oVx = O.path(:,2);
            oVy = O.path(:,3);
            oX = O.path(:,4);
            oY = O.path(:,5);
         end

        % Determine the indices for evenly spaced timeframes at fps frames/second
        % and store them in frame Index to get normalized linear time spacing for
        % animating at a constant speed of time
        timescaling = 1;
        fps = 30;
        fps = fps*timescaling;
        delt = 1/fps; %time step size between frames
 
        % Reduce the telemetry arrays to increments of a fixed
        % time interval use average value of the two neares valuest on each 
        % side of the time marker to determine a weighted average for the
        % checkpoint.

        if(oLaunch)
            [oT, oPos] = makeFrameDeck(oT, [oX oY]);
        end
        [T, LPos] =  makeFrameDeck(T, [X Y Z Vx Vy]);
        X = LPos(:,1);
        Y = LPos(:,2);
        Z = LPos(:,3);
        Vx = LPos(:,4);
        Vy = LPos(:,5);
        
        %Find normalized vector lengths and velocity value
        vnet = sqrt(Vx.^2+Vy.^2);
        scaleFactor = .1;
        normXPrime = scaleFactor*Vx./vnet;
        normYPrime =  scaleFactor*Vy./vnet; 

        % layover the UI panel
        hold( GUI.surf, 'on' )
        axis( GUI.surf,'manual')
        hold( GUI.board, 'on' )
        axis( GUI.board,'manual')
        
        % Create a 3d quiver object to track the motion of the lander
        q3 = quiver3(GUI.surf, NaN, NaN, NaN, NaN, NaN, 0);
        q3.Marker='.';
        q3.MarkerSize=30;
        q3.LineWidth=3;
              
        % Create a 2d quiver for the board view
        q2 = quiver(GUI.board, NaN, NaN, NaN, NaN);
        q2.Marker='.';
        q2.MarkerSize=30;
        q2.LineWidth=3;

        % Play the frames of the object in motion in a smooth fashion
        maxIndex = T(end);
        if (oLaunch)
            maxIndex = max(T(end), oT(end));
        end
     
        orbgfx = GUI.World.gfx2d.orbiter;
        orbcannon = GUI.World.gfx2d.landerTube;
        
        tic;
    for k = 1:maxIndex
 
        while (toc < k*delt-.5*delt)
        end
        if(oLaunch )
            if(k<oT(end))
                %Update orbiter position and lander tube mount
                newPos =  oPos(k,:)-.5*OB.dim;
                orbgfx.Position(1:2)=newPos;
                orbcannon.XData = newPos(1)+OB.cannonOffset(1);    
                orbcannon.YData = newPos(2)+OB.cannonOffset(2); 
            end            
        end
        if(k<T(end))
            %Update postion and velocity 3d
        q3.XData=X(k); q3.YData=Y(k); q3.ZData=Z(k);
        q3.UData=normXPrime(k); q3.VData=normYPrime(k);
        %Update postion and velocity 2d
        q2.XData=X(k); q2.YData=Y(k); 
        q2.UData=normXPrime(k); q2.VData=normYPrime(k);
        end 
        
        %redraw GUI
        drawnow
    end  
        delete(q3);
        delete(q2);
        hold( GUI.surf, 'off' )
        hold( GUI.board, 'off' )
    end
        
        
        function [Tindex Pos] = makeFrameDeck(T, data)
            dataIndex = 2;
            %make an index of frame numbers starting from t=0 -> frame 0
            Tindex=[floor(T(1)/delt):1:floor(T(end)/delt)];
            Pos(1,:) = data(1,:);
            
            for step = Tindex(2:end)
                t = step*delt; %which time value to search for
                while (t > T(dataIndex))
                    %advance the data index to the next frame after passing
                    %desired time value for frames
                    dataIndex = dataIndex + 1;
                end
                %take the weighted average of the the values to get the
                %intermediate frame for the fps time value
               twidth = T(dataIndex)-T(dataIndex-1); 
               posval = (1-abs((T(dataIndex) - t))/twidth ).*data(dataIndex,:) +...
                       (1-abs((T(dataIndex-1) - t))/twidth ).*data(dataIndex-1,:); 
                   Pos(step, :) = posval;  %record the position values                 
            end
        end
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% ORBITER MECHANICS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
    % Calculate orbiter path and any launch events and show path
    function launchOrbiter(btn, event)
        Orb.triggerType = GUI.dd.trigger.Value;
        Orb.triggerVal = GUI.field.td.Value;
        disp('Launching Orbiter');
        
        %Clear all previous orbiters from gfx field
        clrOrb();
        
        Orb.path = orbiterPath();        
        plotOrbiter();
        %Check if path crossed the boundary condition for releasing a
        %launcher. launch the launcher at the correct timestamp if it
        %the event is triggered by the orbiter trajectory
        %find when trigger in the orb path is above the trigger val
        
        if(Orb.triggerType == "Distance")
            lindex = find(Orb.path(:,4)>Orb.triggerVal ,1);
        elseif(Orb.triggerType == "Time")
            lindex = find(Orb.path(:,1)>Orb.triggerVal ,1);
        end
        
        %Check if lander launch was triggered and send an orbital launch if
        %a launch index was found
        if(~isempty(lindex))
            disp("Lander Launched")
            %pass the launch condition values
            orbitalLaunch(Orb.path(lindex,:));
        end       
        
        % Add any particular details about new features plotted in the GUI
        % to an array object holding all the instantiated Orbiters
        % (unpolished object oriented programming)
        orbiters = [orbiters Orb];
        assignin('base','orbiterData',orbiters);  
        set(GUI.World.gfx2d.orbiter, 'Visible', 'on');
        
        %Adjust the launch tube angle to the user value
        [uvec, vvec] = pol2cart((-GUI.field.lang.Value+90)*pi/180, 1);
        GUI.World.gfx2d.landerTube.UData=uvec;
        GUI.World.gfx2d.landerTube.VData=vvec;    
        
        set(GUI.World.gfx2d.landerTube, 'Visible', 'on');
        
        %Returns an orbiter trajectory [ T Vx Vy X Y]
        function path = orbiterPath()
            
            %u = [v x] u' = [a v]  
            tstep = .01;
            tspan = [0 : tstep : 10];
            u0 = [GUI.field.orbvel.Value OB.pos0(1)+OB.dim(1)*.5];
            [T U] = ode45(@(t,u) orbiterMotion(t,u), tspan ,u0);
            
            X = U(:,2);
            VY = zeros(size(X));
            Y = (OB.pos0(2)+.5*OB.dim(2)).*ones(length(T),1);
            path = [T U(:,1) VY U(:,2) Y ];  

            function dudt = orbiterMotion(t,u)
                v = u(1); x = u(2);
                mu = GUI.field.fric.Value;
                ax=0;
                %rolling friction opposite to direction of motion
                if( abs(v) < .0001)
                    v=0;
                end

                % Braking action that can be set in the .m global variable
                % in the header
                if ( abs(v) > 0)   
                    ax = -mu*BD.g*sign(v);
                    if (u(2) >= BD.orbiterBrakeDistance)   

                    ax = -BD.orbiterBrakeFriction*BD.g*sign(v);                   
                    end
                end              

                dudt = [ax u(1)]';                
            end          
            
            function [value, isterminal, direction] = outOfBounds(t, u)
                vx = u(1); x = u(2);
                % Check the three termination cases, integration will
                % halt when value = 0 and isterminal = 1
                value = ~(offtable(x, 1)|(vx <1e-3)); %placeholder val for y
                isterminal = 1;
                direction = 0;
            end    
            %Check if orbiter is still on the table
            function bool = offtable(x, y)
                epsilon = .01;
                bool = x < (BD.area(1)-epsilon) | x > (BD.area(2)+epsilon)...
                | y < (BD.area(3)-epsilon) | y> (BD.area(4)+epsilon);             
            end  
        end         
        function plotOrbiter()
            X = Orb.path(:,4);
            Y = Orb.path(:,5);
            T = Orb.path(:,1);

            % The orbiter speed checkpoints should be shown graphically on
            % the board, the array can be reduced to increments of a fixed
            % time interval to show this type of velocity information, use
            % the average value of the two neares values on each side of
            % the time marker to determine a weighted average for the
            % checkpoint.
            markers = [X(1) Y(1)]; %plot + marker locations
            tstep = OB.pathMarkerTimeDistance;
            
            timeIndex = 1;
            for i = 2:length(T)
                tdiff = T(i) - tstep*timeIndex;
                if (tdiff >= 0)
                   twidth = T(i)-T(i-1);
                   xyval = (1-tdiff/twidth)*[X(i) Y(i)] +...
                       (1-abs((T(i-1) - tstep*timeIndex))/twidth)*[X(i-1) Y(i-1)];
                   markers = [markers; xyval];
                   timeIndex=timeIndex+1;
                end                 
            end
            
            %plot the info over the gui interface, save plot handle to L for later
            %deletion if required            
            hold( GUI.board, 'on' )
            axis( GUI.board,'manual')
            color = [140 90 90]/255;
            Orb.plot2d.line = plot(GUI.board, X,Y, '-',...
                'Color', color,...
                'LineWidth',1); 
            Orb.plot2d.markers = plot(GUI.board, markers(:,1),markers(:,2), '+',...
                'Color', color,...
                'LineWidth',2); 
            hold( GUI.board, 'off' )                   
        end
        
        % input data: [T vx vy x y]
        function orbitalLaunch(data)
            %convert the field values entered by user into cartesian vector
            %components
            [vx, vy] = pol2cart((-GUI.field.lang.Value+90)*pi/180, GUI.field.lvel.Value);
            
            n1 = data(4);
            n2 = data(5);
            % add the orbiter velocity to the launch tube velocity
            % (relative motion)
            n3 = data(2) + vx;
            n4 = data(3) + vy;
            n5 = data(1);
            makeLander(n1, n2, n3, n4, n5);
        end
        function clrOrb()
            if(~isempty(orbiters))
                for O = orbiters
                    set(O.plot2d.line,'Visible','off');  
                    set(O.plot2d.markers,'Visible','off');  
                end        
                set(GUI.World.gfx2d.orbiter, 'Visible', 'off'); 
                set(GUI.World.gfx2d.landerTube, 'Visible', 'off');
            end
        end
    end
end

%Build the GUI and base game surface visual features--
% Returns handles for figure, fields, buttons, grid layout, board, ect.
function GUI = buildUI(BD, OB)
% Create main window
GUI.fig = uifigure('Name', 'Space Simulator', 'Position',[0,0,1200,800] );

% Manage layout
GUI.grid = uigridlayout(GUI.fig);
GUI.grid.RowHeight = {20, 20, 20, 20, 20,...
                      20, 20,...
                      20, 20, 20, 20, 20, 20,...
                      20, 20, 20, 20, 20, '1x'};
GUI.grid.ColumnWidth = {100,50,'1x'};

%Make a grid for visuals panels to the right and add figures
GUI.World.graphpanel = uipanel(GUI.grid, 'Title','Visualization');
GUI.World.graphpanel.Layout.Row = [1 length(GUI.grid.RowHeight)]; %use all rows
GUI.World.graphpanel.Layout.Column = 3;
GUI.World.panelgrid = uigridlayout(GUI.World.graphpanel,[2 1]);
GUI.World.panelgrid.RowHeight = {'1x','1x'};

% Create UI components
% Text Field Labels Landers
lblxpos = uilabel(GUI.grid, 'Text', 'x Position');
lblypos = uilabel(GUI.grid, 'Text', 'y Position');
lblvel = uilabel(GUI.grid, 'Text', 'Velocity');
lblang = uilabel(GUI.grid, 'Text', 'Angle (Degrees)');
% Text Field Labels Orbiter

lbllvel = uilabel(GUI.grid, 'Text', 'Lander L Vel');
lbllang = uilabel(GUI.grid, 'Text', 'Launch Angle');
lblfric = uilabel(GUI.grid, 'Text', 'Friction');
lblorbvel = uilabel(GUI.grid, 'Text', 'Orbiter L Vel');
lbltd = uilabel(GUI.grid, 'Text', 'Trigger Value');
GUI.labels = [lblxpos lblypos lblvel lblang  lbllvel lbllang lblfric lblorbvel lbltd];

% Text Field Boxes Landers
GUI.field.xpos = uieditfield(GUI.grid,'numeric');
GUI.field.ypos = uieditfield(GUI.grid,'numeric');
GUI.field.vel = uieditfield(GUI.grid,'numeric');
GUI.field.ang = uieditfield(GUI.grid,'numeric');
% Text Field Boxes Orbiters
GUI.field.lvel = uieditfield(GUI.grid,'numeric');
GUI.field.lvel.Value= 1;
GUI.field.lang = uieditfield(GUI.grid,'numeric');
GUI.field.lang.Value= 0;
GUI.field.fric = uieditfield(GUI.grid,'numeric');
GUI.field.fric.Value = .005;
GUI.field.orbvel = uieditfield(GUI.grid,'numeric');
GUI.field.orbvel.Value = 1.1;
GUI.field.td = uieditfield(GUI.grid,'numeric');
GUI.field.td.Value = 2.0;

%Adjust position editfield to only include positions on the board
GUI.field.xpos.Limits = [BD.area(1) BD.area(2)];
GUI.field.ypos.Limits = [BD.area(3) BD.area(4)];

% Not sure if there is a better way to do this:
fields = [GUI.field.xpos  GUI.field.ypos GUI.field.vel GUI.field.ang...
          GUI.field.lvel GUI.field.lang GUI.field.fric GUI.field.orbvel GUI.field.td ];  
      
% Dropdown Boxes
GUI.dd.trigger = uidropdown(GUI.grid,'Items',{ 'Time', 'Distance',});
      
% Launch and line toggle buttons
GUI.button.launch = uibutton(GUI.grid, 'Text', 'Launch Lander');
GUI.button.play = uibutton(GUI.grid, 'Text', 'Playback');
GUI.button.orbiter = uibutton(GUI.grid, 'Text', 'Launch Orbiter');
GUI.button.lines = uibutton(GUI.grid,'state', 'Text', 'Lines On/Off');
GUI.button.reset = uibutton(GUI.grid, 'Text', 'Reset Sim');

buttons = [GUI.button.launch GUI.button.play  ...
    GUI.button.orbiter  GUI.button.lines GUI.button.reset];

% Lay out UI components
% Position labels
i = 2;
for lbl =  GUI.labels(1:4)
    lbl.Layout.Row = i;
    lbl.Layout.Column = 1;
    i = i + 1;
end
i = 9;
for lbl =  GUI.labels(5:9)
    lbl.Layout.Row = i;
    lbl.Layout.Column = 1;
    i = i + 1;
end
% Position data fields
i = 2;
for fld =  fields(1:4)
    fld.Layout.Row = i;
    fld.Layout.Column = 2;
    i = i + 1;
end
i = 9;
for fld =  fields(5:9)
    fld.Layout.Row = i;
    fld.Layout.Column = 2;
    i = i + 1;
end
% Position Buttons and Dropdown Box
i = 6;
for btn = buttons(1:2)
btn.Layout.Row = i;
btn.Layout.Column = [1 2];
i = i + 1;
end

GUI.dd.trigger.Layout.Row = 14;
GUI.dd.trigger.Layout.Column = [1 2];

i = 16;
for btn = buttons(3:5)
btn.Layout.Row = i;
btn.Layout.Column = [1 2];
i = i + 1;
end

%Create the GUI visual panels

% 3d Mesh Panel Axes Placement
GUI.surf = uiaxes(GUI.World.panelgrid);
GUI.surf.Layout.Row = 2;

% Simulation Board Panel Axes Placement
GUI.board = uiaxes(GUI.World.panelgrid);
GUI.board.Layout.Row = 1;

%Draw the mesh surface for analysis
GUI.World.gfx3d = drawSimMesh(BD, GUI.surf);

%Draw the playing field objects and return their handles
GUI.World.gfx2d = drawSimulationSurface(BD, OB, GUI.board);

function gfx3d = drawSimMesh(BD, ax)
    % set the mesh resolution (del) and the extention factor in terms of
    % radius (reach)
    del = .01;
    reach = 1.1;
    limits = [BD.titanPos(1)-BD.titanRad*reach , BD.titanPos(1)+BD.titanRad*reach,...
    BD.titanPos(2)-BD.titanRad*reach, BD.titanPos(2)+BD.titanRad*reach];
    % Build mesh over board surface
    [X,Y] = meshgrid(limits(1):del:limits(2) , limits(3):del:limits(4) );
    % Convert to Titan R coordinates
    R = rTitan(X,Y,BD);
    %Evaluate Z height and map the funnel
    Z = boardSurf(R);
    gfx3d.meshsurface = surf(ax, X,Y,Z, 'FaceAlpha',0.7);
    set(gfx3d.meshsurface,'linestyle','none');
    
    %Plot the inner diameter cutoff
    th = [0:.1:2*pi];
    X = BD.titanInnerRad*cos(th)+BD.titanPos(1);
    Y = BD.titanInnerRad*sin(th)+BD.titanPos(2);
    Z = -.1525*ones(size(X));
    hold( ax, 'on' )  
    plot3(ax, X,Y,Z, 'b', 'LineWidth', 3);
    hold(ax, 'off')
    
    %Flip colormap to make Titan yellow colored
    colormap(ax, flipud(parula))
    
    % use the command get(GUI.surf, 'View') to get data on which view the
    % window has to set preferred initial view, here the view is set
    view(ax, -10, 50);
    axis( GUI.surf,'manual')
    
    % create laser sights on 3d mesh
    gfx3d.lineHandles = setLineHandles(ax);       
end

function gfx2d = drawSimulationSurface(BD, OB, ax)
    
%Draw the playing surface and borders
gfx2d.top = rectangle(ax, 'Position',[0,0,7,2.5],'FaceColor',[250 227 201]/255,...
'EdgeColor',[177 105 16]/255,'LineWidth',2);

%Draw the planetary circles
gfx2d.earthL = rectangle(ax, 'Position', ...
    [BD.earthLPos-[BD.earthRad BD.earthRad] 2*[BD.earthRad BD.earthRad]],...
    'Curvature',[1 1],...
    'FaceColor','bl');
gfx2d.earthR = rectangle(ax, 'Position',...
    [BD.earthRPos-[BD.earthRad BD.earthRad] 2*[BD.earthRad BD.earthRad]],...
    'Curvature',[1 1],...
    'FaceColor','bl');
gfx2d.titan = rectangle(ax, 'Position',...
    [BD.titanPos-[BD.titanRad BD.titanRad] 2*[BD.titanRad BD.titanRad]],...
    'Curvature',[1 1],...
    'FaceColor',[182 180 0]/255);
gfx2d.titanInner = rectangle(ax, 'Position',...
    [BD.titanPos-[BD.titanInnerRad BD.titanInnerRad] 2*[BD.titanInnerRad BD.titanInnerRad]],...
    'Curvature',[1 1]);

gfx2d.orbiter = rectangle(ax, 'Position',...
    OB.pos0,...
    'Curvature',[.5 .5],...
    'FaceColor',[180 0 0]/255);
set(gfx2d.orbiter, 'Visible', 'off');

hold( ax, 'on' )   
axis( ax,'manual')
gfx2d.landerTube = quiver(ax, ...
    OB.pos0(1)+OB.cannonOffset(1), OB.pos0(2)+OB.cannonOffset(2),...
    0,1,.15,...
    'linewidth',4,...
    'Color',[250 100 30]/255);
set(gfx2d.landerTube, 'Visible', 'off');

%Draw the laser line of sights, set invisible until activated
gfx2d.lineHandles = setLineHandles(ax);

%Draw the launch cursor and arrow at the start position
gfx2d.cursor = quiver(ax, 0,0,0,0, 'AutoScale','off');
gfx2d.cursor.Marker='.';
gfx2d.cursor.MarkerSize=30;
gfx2d.cursor.LineWidth=2;
gfx2d.cursor.Color = 'k';
hold(ax, 'off')
end

function lineHandles = setLineHandles(ax)
        gate(1,:) = [.7 0 .7 2.5];
        gate(2,:) = [3.45 0 3.45 2.5]; 
        gate(3,:) = [3.55 0 3.55 2.5]; 
        gate(4,:) = [0 1.45 7 1.45];
        gate(5,:) = [0 1.35 7 1.35];
        gate(6,:) = [0 .025 7 .025];
        gate(7,:) = [0 2.4 7 2.4];
            for i = 1:size(gate,1)
                lineHandles(i) = line(ax, [gate(i,1) gate(i,3)],[gate(i,2) gate(i,4)],...
                    'Color', 'red');
            end
        set(lineHandles, 'Visible', 'off');
    end
end

 % Piecewise surface function describing the flat board and funnel
function z = boardSurf(R)
z = zeros(size(R));
        
    for i = 1:size(z,1)
        for j = 1:size(z,2)
            r = R(i,j);
            %Flat top by default
            z(i,j)=0;
            % Check if inside of titan's orbit
            if (r < .45)
                if (r <= .15)
                    % Center cone
                    z(i,j) = -.457 +2.03*r;                
                else
                    % Outer capture area
                    z(i,j) = - (r^2 - .913*r + .209)/(4.93*r^2 + r +.36); 
                end
            end
        end
    end
end

function r = rTitan(x,y,BD)
    r = ((x-BD.titanPos(1)).^2 + (y-BD.titanPos(2)).^2).^.5;
end
