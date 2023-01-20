%% SFU MATH 495 Project
% Student: Sanjay Alwani 
% Semseter: Fall 2022
%
% The following code simulates a two lane bi-directional traffic flow
%   It uses the NASCH Cellular automata model to simulate the flow of cars and
%    evaluates the effect of controlled and uncontrolled pedestrian crossing
%    at a fixed point (crosswalk vs cross signal)
%  The CA has an open boundary allowing arrival of cars and pedestrians.
%  Cars and pedestrian arrival times follow a Poisson Process.
%%


%% SIMULATION MASTER RUNNER CONSTANTS
% SIM_RUNS = 10;

%% SIMULATION INIT
% Parametrization
PWT_Parameter = [0, 5, 10, 20, 30, 40, 50, 60];
VAR_Parameter = 0:0.05:1;
Runs_per_parametrization = 200;

SIM_RUNS = length(PWT_Parameter) * length(VAR_Parameter) * Runs_per_parametrization;

Parameter_combo = cell(1, SIM_RUNS);

for i = 1:length(PWT_Parameter)
    for j = 1:length(VAR_Parameter)
        for k = 1:Runs_per_parametrization
            Parameter_combo{(i-1)*length(VAR_Parameter)*Runs_per_parametrization + (j-1)*Runs_per_parametrization + k} = [PWT_Parameter(i), VAR_Parameter(j)];
        end
    end
end

% Init simulation run data arrays
SimStart = cell(SIM_RUNS,1);
SimEnd = cell(SIM_RUNS,1);
PAR = zeros(SIM_RUNS,1);
PWT = zeros(SIM_RUNS,1);
PCT = zeros(SIM_RUNS,1);
VAR = zeros(SIM_RUNS,1);
AvgPedWaitTimesP90  = zeros(SIM_RUNS,1);
AvgPedWaitTimeMedian = zeros(SIM_RUNS,1);
AvgVehicleDensity    = zeros(SIM_RUNS,1);
AvgVehicleVelocity   = zeros(SIM_RUNS,1);
AvgVehicleFlow       = zeros(SIM_RUNS,1);
AvgVehicleStagnation = zeros(SIM_RUNS,1);
NumCrossedPeds       = zeros(SIM_RUNS,1);
NumArrivedVehicles   = zeros(SIM_RUNS,1);

% Sample time RV assuming mutual steady state start time
% Simulation time
SimDuration = 5000;

% Steady-state start and random vector sampler time
SteadyStateStart = 1000;

RandomSampleTime = SteadyStateStart + unidrnd(SimDuration - SteadyStateStart, SIM_RUNS, 1);

tic
%% SIMULATION MAIN
parfor sim_run_i = 1:SIM_RUNS
    if (mod(sim_run_i, 100) == 0)
        disp("Simulation runs complete:");
        disp(sim_run_i);
    end
    SimStart(sim_run_i) = cellstr(datetime);
    pwt_par = Parameter_combo{sim_run_i}(1);
    var_par = Parameter_combo{sim_run_i}(2);
    
    %% SIMULATION RUN CONSTANTS
    % Traffic infrastructure geometry
    RoadLen = 133;         % 7.5m * 133 = ~1km road
    RedLightLocation = 67; % mid-point of road

    % Speed limit
    vmax = 3;
    
    % Vehicle Parameters
    % We assume each lane has vehicle arrivals following a homogeneous Poisson Process
    VehicleArrivalPeriod = 1/var_par; % (avg units of time for 1 car arrival)
    Randomization = 0.3;

    % Pedestrian Parameters
    PedLightTime = 12;
    PedCrossingTime = 12;
    assert(PedCrossingTime <= PedLightTime);

    PedWaitTime = pwt_par;
    % Let 1 r.v. represent the combined exp. dist. arrival rates of both sides
    PedArrivalPeriod = 2*PedCrossingTime;

    %% SIMULATION RUN INITIALIZATION
    CSpace = zeros(2, RoadLen);
    VelocityField = zeros(2, RoadLen);

    AnimateCA = false;
    % Animation code
    animation = zeros(3, RoadLen);
    % Build out color dimension
    animation(:,:,2) = 0;
    animation(:,:,3) = 0;

    % Build out frame dimension
    animation(:,:,:,2) = 0;

    frame_idx = 1;
    %Draw one frame
    for j = 1:2
        for i = 1:RoadLen
            rgb = ones(1,3) .* CSpace(j, i);
            animation(j,i,:,frame_idx) = reshape(rgb, [1 1 3 1]);
        end
    end
    frame_idx = frame_idx + 1;

    %% SIMULATION RUN INSTRUMENTATION
    % Car arrival time vector
    arrivals_n = 0;
    arrivals_s = 0;

    % Pedestrians crossed count
    crossed_pedestrians = 0;

    % Pedestrian Stagnation = average time spent waiting
    pedestrian_timer_queue = timer_queue();

    % Vehicle Stagnation = time spent at velocity(0) before the crossing
    vehicle_stagnation_vector = zeros(1,0);
    StagnationTimeField = zeros(2, RoadLen);

    % Vehicle Velocity = average velocity per time step
    vehicle_velocity_vector = zeros(1,0);

    % Vehicle Density = num cars on road / length of road
    vehicle_density_vector = zeros(1,0);

    % Vehicle Flow = cars passing crossing per second
    vehicle_crossing_vector = zeros(1,0);


    %% SIMULATION RUN MAIN

    % Lane directionality state (for simplicity and by indepedence only anim.
    %   code is concerned with directionality)
    DirField = [-1, 1];

    % Random Variable Generation
    % We do not allow stacking of cars.
    next_arrival_north = max(round(exprnd(VehicleArrivalPeriod)), 1);
    next_arrival_south = max(round(exprnd(VehicleArrivalPeriod)), 1);
    % We allow stacking of pedestrians. Sample poisson dist. every time step
    ped_arrivals = poissrnd(1/PedArrivalPeriod);

    % Pedestrian traffic light state variables
    crossing = false;
    waiting = false;
    ped_light_countdown = PedLightTime;
    pedestrian_queue = 0;
    wait_endured = 0;

    % Simulation Run
    for t = 1:SimDuration
        pedestrian_timer_queue = pedestrian_timer_queue.wait(1);
        % Pedestrian has arrived at crossing
        if (ped_light_countdown == 0)
            crossing = false;
        end
        if (ped_arrivals)
            if (crossing == false)
                % Pedestrian has red light -> press button to wait if not done
                if (waiting == false)
                    waiting = true;
                end
            end
            % Pedestrian is enqueued at crossing
            pedestrian_queue = pedestrian_queue + ped_arrivals;
            pedestrian_timer_queue = pedestrian_timer_queue.addTimers(ped_arrivals);
        end
        if (waiting)        
            if (wait_endured >= PedWaitTime)
                crossing = true;
                ped_light_countdown = PedLightTime;
                waiting = false;
                wait_endured = 0;
            else
                wait_endured = wait_endured + 1;
            end
        end
        if (crossing)
            % If whomever is waiting at the crossing and they can cross in
            %   time, then cross and remove from queue
            if (pedestrian_queue && PedWaitTime == 0)
                ped_light_countdown = PedCrossingTime;
            end
            if (ped_light_countdown >= PedCrossingTime)
                crossed_pedestrians = crossed_pedestrians + pedestrian_queue;
                pedestrian_queue = 0;
                pedestrian_timer_queue = pedestrian_timer_queue.dequeTimers();
            end
            % Count down pedestrian crossing light and check for stop condition
            ped_light_countdown = ped_light_countdown - 1;
        end
        % Generate next arrival count
        ped_arrivals = poissrnd(1/PedArrivalPeriod);

        % Crossing state brings red light into existence
        redLight = (crossing)*RedLightLocation;

        % Get the gaps between cars in each lane
        DistanceField = getDistances(CSpace, redLight);

        % Get the new velocity for each car (in place)
        cars = {find(CSpace(1,:)), find(CSpace(2,:))};
        new_velocity = updateVelocity(cars, VelocityField, DistanceField, vmax, Randomization, Randomization);

        if (t == RandomSampleTime(sim_run_i))
            % Record flow at t
            numCrossing = getCrossing(new_velocity, RedLightLocation, vmax);
            AvgVehicleFlow(sim_run_i) = numCrossing;
            
            % Record avg velocity at t
            avgVelocity = getAvgVelocity(new_velocity, CSpace);
            AvgVehicleVelocity(sim_run_i) = avgVelocity;

            % Update stagnation percentage
            stagnation_pct = getStagnation(new_velocity, CSpace, RedLightLocation, RoadLen);
            AvgVehicleStagnation(sim_run_i) = stagnation_pct;
        end
        % Shift the position and velocity info to the new position of the cars
        [new_position, new_velocity] = updateSpace(CSpace, new_velocity);

        % Simulate new car arrivals
        if (next_arrival_south == t)
            % If first cell unoccupied, new car arrives with velocity 1
            if (new_position(2,1) ~= 1)
                new_position(2,1) = 1;
                new_velocity(2,1) = 2;
            end
            next_arrival_south = max(t + 1, t + round(exprnd(VehicleArrivalPeriod)));
            arrivals_s = arrivals_s + 1;
        end
        if (next_arrival_north == t)
            % If first cell unoccupied, new car arrives with velocity 2
            if (new_position(1, 1) ~= 1)
                new_position(1, 1) = 1;
                new_velocity(1, 1) = 2;
            end
            next_arrival_north = max(t + 1, t + round(exprnd(VehicleArrivalPeriod)));
            arrivals_n = arrivals_n + 1;
        end


        % Update space and velocity field
        CSpace = new_position;
        VelocityField = new_velocity;
        
        if (t == RandomSampleTime(sim_run_i))
            % Record vehicle density
            AvgVehicleDensity(sim_run_i) = mean(CSpace, [1,2]);
        end
        if AnimateCA
            %Draw one frame
            for j = 1:2
                index = 1:RoadLen;
                if (DirField(j) == -1)
                    index = RoadLen:-1:1;
                end
                for i = 1:length(index)
                    % Animate red light
                    if (index(i) == redLight)
                        animation(j,index(i),:,frame_idx) = reshape([1 0 0], [1 1 3 1]);
                        continue;
                    end
                    rgb = [1, 1, 0.5] .* CSpace(j, i);

                    animation(j,index(i),:,frame_idx) = reshape(rgb, [1 1 3 1]);
                end
            end
            % animate peds
            for i = 1:pedestrian_queue
                animation(3,RedLightLocation + i - 1,:,frame_idx) = reshape([0.2 0.3 1], [1 1 3 1]);
            end
            frame_idx = frame_idx + 1;
        end
    end

    if AnimateCA
        implay(animation);
    end

    % Calculate pedestrian wait time metrics after the fact
    
    % Grouped together for both sides, but wait time is location indep.
    [pwt_median, pwt_p90] = pedestrian_timer_queue.getAvgWaitPercentiles();
    AvgPedWaitTimeMedian(sim_run_i) = pwt_median;
    AvgPedWaitTimesP90(sim_run_i)  = pwt_p90;

    crossed_pedestrians;
    num_vehicles_arrived = arrivals_s + arrivals_n;
    
    NumArrivedVehicles(sim_run_i) = num_vehicles_arrived;
    NumCrossedPeds(sim_run_i) = crossed_pedestrians;
    
    PAR(sim_run_i) = 1/PedArrivalPeriod;
    VAR(sim_run_i) = 1/VehicleArrivalPeriod;
    PWT(sim_run_i) = PedWaitTime;
    PCT(sim_run_i) = PedCrossingTime;

    SimEnd(sim_run_i) = cellstr(datetime);
end
toc

% IMPORTANT CHANGE: The entries (X_i's) are now point samples. We have bumped up runs
% per parametrization and will be taking the means of the point sample for 
% iid property of the X_i's in our analysis.
T = table(SimStart, SimEnd, PAR, VAR, PWT, PCT, NumArrivedVehicles, NumCrossedPeds, ...
        AvgPedWaitTimeMedian, AvgPedWaitTimesP90, AvgVehicleStagnation, ...
        AvgVehicleVelocity, AvgVehicleFlow, AvgVehicleDensity);
    
writetable(T, 'project_data.csv','WriteMode','Append');

%% FUNCTIONS
% Takes in left to right velocity fields
function [distanceField] = getDistances(CSpace, redLight)
    RoadLen = size(CSpace, 2);
    distanceField = zeros(size(CSpace));
    for row = 1:size(CSpace, 1)
        % Calculate distances
        for i = 1:RoadLen
            if (CSpace(row, i) == 1)
                lookAhead = 1;
                while(i + lookAhead < RoadLen && CSpace(row, i + lookAhead) == 0)
                   % Search to next car or red light whichever is nearest
                   if(i + lookAhead == redLight)
                       break;
                   end
                   lookAhead = lookAhead + 1 ;
                end
                if(i + lookAhead >= RoadLen)
                    distanceField(row, i) = inf;
                else
                    distanceField(row, i) = lookAhead - 1; % lookAhead == empties + 1 found car
                end
            end
        end
    end
end

function [newVelocityField] = updateVelocity(cars, velocityField, distanceField, vMax, p, p_0)

    newVelocityField = zeros(size(velocityField));
    for row = 1:size(velocityField, 1)
        % step 1: v = min(v_old + 1, v_max)
        for i = cars{row}
            newVelocityField(row, i) = min(velocityField(row, i) + 1, vMax);
        end

        % step 2: v = min(v, gap)
        for i = cars{row}
            newVelocityField(row, i) = min(newVelocityField(row, i), distanceField(row, i));
        end 
        
        % step 3: v = max(0, v-1) with probability (p_0 if v=vmax else p)
        U = unifrnd(0, 1, length(cars{row}), 1);
        for i = 1:length(cars{row})
            car_pos = cars{row}(i);
            u = U(i);
            if (newVelocityField(row, car_pos) == vMax)
                if (u > p_0)
                    continue
                end
                newVelocityField(row, car_pos) = max(0, newVelocityField(row, car_pos) - 1);
            else
                if (u > p)
                    continue
                end
                newVelocityField(row, car_pos) = max(0, newVelocityField(row, car_pos) - 1);
            end
        end
    end
end

function [newCSpace, newVelocityField] = updateSpace(CSpace, velocityField)
    RoadLen = size(CSpace, 2);

    newCSpace = zeros(size(CSpace));
    newVelocityField = zeros(size(velocityField));

    for row = 1:size(CSpace, 1)
        for i = 1:RoadLen
            if (CSpace(row, i) == 1)
                new_i = i + velocityField(row, i);
                if (new_i >= RoadLen)
                    break
                end
                newCSpace(row, new_i) = 1;
                newVelocityField(row, new_i) = velocityField(row, i);
            end
        end
    end
end

% Instrumentation Functions
function numCrossings = getCrossing(NextVelocityField, RedLightLocation, vMax)
    numCrossings = 0;
    % Crossing defn: passed boundary at left of redLightLocation
    CellBeforeRedLight = RedLightLocation - 1;
    for j = 1:2
        for i = CellBeforeRedLight:-1:(CellBeforeRedLight - vMax)
            if (i + NextVelocityField(j, i) >= RedLightLocation)
                numCrossings = numCrossings + 1;
            end
        end
    end
end

function avgVelocity = getAvgVelocity(NextVelocityField, CSpace)
    avgVelocity = sum(NextVelocityField, [1, 2]) / sum(CSpace, [1, 2]);
end

function stagnationPercentage = getStagnation(NextVelocityField, CSpace, RedLightLocation, RoadLen)
    BehindRedLight = [ones(2, RedLightLocation - 1), zeros(2, RoadLen - RedLightLocation + 1)];
    vehicles_behind_redlight = BehindRedLight .* CSpace;
    stagnated_vehicles = (NextVelocityField == 0) .* vehicles_behind_redlight;
    
    stagnationPercentage = 0;
    if (sum(vehicles_behind_redlight) == 0)
        return;
    end
    
    stagnationPercentage = sum(stagnated_vehicles) / sum(vehicles_behind_redlight);
end

function [stf, st_records] = updateStagnationTimes(stagnationTimeField, newVelocityField, carLocations, RedLightLocation)    
    stf = zeros(size(stagnationTimeField));

    st_records = -Inf * ones(1, 2);
    
    for row = 1:2
        for car_pos = carLocations{row}
            new_pos = car_pos + newVelocityField(row, car_pos);
            if (new_pos >= RedLightLocation)
                % Using the fact that: There is only one exit at a time per
                % lane
                if (car_pos < RedLightLocation)
                    st_records(row) = stagnationTimeField(row, car_pos);
                end
                break;
            end
            isStagnated = (car_pos < RedLightLocation) && (new_pos == car_pos);
            stf(row, new_pos) = stagnationTimeField(row, car_pos) + isStagnated;
        end
    end
end

%% ==== Licenses =====

%% [1] Project CA_NaSch
% 
% Copyright (c) 2013 Codinfox(codinfox.me)
% 
% 
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, including without limitation the rights
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
% copies of the Software, and to permit persons to whom the Software is
% furnished to do so, subject to the following conditions:
% 
% The above copyright notice and this permission notice shall be included in
% all copies or substantial portions of the Software.
% 
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
% THE SOFTWARE.
%
