





clear all, close all, clc
format long
v_0 = 0.001;
gamma_0 = 89.8;%89.8 worked fine
R_earth = 6378;
X_0 = 0;
H_0 = 0;
m_structure1 = 22000; %26000;
m_structure2 = 4000;
m_payload = 6000;
m_fuel1 = 369000;%236000;
m_fuel2 = 81100;
m_tot = m_structure1 + m_structure2 + m_fuel2 + m_payload +m_fuel1;

% Initialize global counters
global gamma_values_stage2 t_counter gamma_counter t_values_stage2;
gamma_values_stage2 = [];
t_values_stage2 = [];
t_counter = 0;
gamma_counter = 0;




m_tot_stage2 = m_structure2 + m_payload + m_fuel2;

m_dot = -250*9;
m_dot2 = -267;
mu = 398600;    %(5.97219*10^24)*(6.6743*10^-11);
g_0 = (mu / (R_earth)^2);
I_sp = 282;
I_sp2 = 348;
T = g_0*-m_dot*I_sp;    %In KN
T2 = g_0 * -m_dot2 * I_sp2;


C_d = 0.3;
A = (pi*(3.7^2)/4)*10^-6;

initial_conditions = [v_0, deg2rad(gamma_0), X_0, H_0, m_tot]; %remember mfuel 1

tspan = [0:1:164]; %160


[t, state] = ode45(@(t, y) rocket_dynamics(t, y, mu, m_dot, T, C_d, A), tspan, initial_conditions);
arc_length = state(:, 3);
altitude = state(:, 4); 

% Extract final state of as initial condition for stage 2
final_state_stage1 = state(end, :);



% Set initial conditions and time span for the second stage
initial_conditions_stage2 = [final_state_stage1(1), final_state_stage1(2), final_state_stage1(3), final_state_stage1(4), m_tot_stage2];
tspan_stage2 = [0:1:183];  % Adjust as needed for your simulation duration

% Solve ODE for second stage with the new function
[t_stage2, state_stage2] = ode45(@(t, y) rocket_dynamics_stage2_engine(t, y, mu, m_dot2, T2, C_d, A), tspan_stage2, initial_conditions_stage2);



% Initial conditions for the coasting phase (using the final state of stage 2)
initial_mass_coast = state_stage2(end, 5) % Mass left after the second stage burns
initial_conditions_coast = [state_stage2(end, 1), state_stage2(end, 2), state_stage2(end, 3), state_stage2(end, 4), m_tot_stage2-m_fuel2+initial_mass_coast-10000];

% Time span for the coasting phase
tspan_coast = [0:1:195]; %360 is good % Adjust the time span as needed for the coasting phase

% Solve ODE for coasting phase
[t_coast, state_coast] = ode45(@(t, y) rocket_dynamics_coast(t, y, mu, C_d, A), tspan_coast, initial_conditions_coast);

gamma0_2 = state_coast(end, 2); 
initial_conditions_stage2_ignition2 = [state_coast(end,1), state_coast(end,2), state_coast(end,3), state_coast(end,4), m_tot_stage2-m_fuel2+initial_mass_coast-10000];





% 
% initial_conditions_stage2 = [final_state_stage1(1), final_state_stage1(2), final_state_stage1(3), final_state_stage1(4), m_tot_stage2];
% gamma0_2 = final_state_stage1(2);














% % Define initial conditions for coasting phase
% initial_conditions_coast = [final_state_stage1(1), final_state_stage1(2), final_state_stage1(3), final_state_stage1(4), m_tot_stage2];
% 
% % Time span for coasting phase
% tspan_coast = [0:1:100];  % Adjust time span as needed for coasting
% 
% % Solve ODE for coasting phase
% [t_coast, state_coast] = ode45(@(t, y) rocket_dynamics_coast(t, y, mu, C_d, A), tspan_coast, initial_conditions_coast);








t_burnout_stage2 = (m_fuel2-(-m_dot2*tspan_stage2(end)))/-m_dot2;
m_fuel_guidance = m_fuel2-(m_dot2*tspan_stage2);
tspan2 = [0:1:t_burnout_stage2];


[t2, state2] = ode45(@(t, y) rocket_dynamics_stage2(t, y, mu, m_dot2, T2, C_d, A, m_fuel_guidance, t_burnout_stage2,gamma0_2), tspan2, initial_conditions_stage2_ignition2);

% disp('Gamma values for stage 2:');
% disp(gamma_values_stage2);
% disp('t values for stage 2:');
% disp(t_values_stage2);

% Combine results from all four stages
t_end_stage2 = t_stage2 + t(end);
t_end_coast = t_coast + t_end_stage2(end);
t_combined_all = [t; t_stage2 + t(end); t_coast + t_end_stage2(end); t2 + t_end_coast(end)];  % Combined time for all stages
t_combined_all_gamma = [t; t_stage2 + t(end); t_coast + t_end_stage2(end); t_values_stage2 + t_end_coast(end)];  % Combined time for all stages

arc_length_combined_all = [state(:, 3); state_stage2(:, 3); state_coast(:, 3); state2(:, 3)];  % Combine arc lengths
altitude_combined_all = [state(:, 4); state_stage2(:, 4); state_coast(:, 4); state2(:, 4)];    % Combine altitudes
mass_combined_all = [state(:, 5); state_stage2(:, 5); state_coast(:, 5); state2(:, 5)];        % Combine mass
velocity_combined_all = [state(:, 1); state_stage2(:, 1); state_coast(:, 1); state2(:, 1)];    % Combine velocity

% Create a new gamma_combined using all four stages
gamma_in_degrees_stage1 = rad2deg(state(:, 2));          % Convert radians to degrees for stage 1
gamma_in_degrees_stage2 = rad2deg(state_stage2(:, 2));   % Convert radians to degrees for stage 2
gamma_in_degrees_coast = rad2deg(state_coast(:, 2));     % Convert radians to degrees for coasting phase
gamma_in_degrees_guidance = rad2deg(state2(:, 2));       % Convert radians to degrees for steering law phase

% Combine flight path angles from all phases
gamma_combined_all = [gamma_in_degrees_stage1; gamma_in_degrees_stage2; gamma_in_degrees_coast; gamma_values_stage2];



% Plot Rocket Flight Path (Arc Length vs Altitude) for all four stages
figure;
plot(arc_length_combined_all, altitude_combined_all);
xlabel('Arc Length (km)');
ylabel('Altitude (km)');
title('Rocket Flight Path (All Stages)');
grid on;

% Velocity, Arc Length, Altitude, Mass plots for all four stages
% Define the lengths of each phase to segment the data correctly
len_stage1 = length(t);
len_stage2 = length(t_stage2);
len_coast = length(t_coast);
len_guidance = length(t2);

% Define the colors for each stage
colors = {'r', 'g', 'b', 'm'};  % Red, Green, Blue, Magenta

% Figure for combined plots
figure;

% Velocity plot
subplot(3, 1, 1);
hold on;
plot(t_combined_all(1:len_stage1), velocity_combined_all(1:len_stage1), colors{1}, 'LineWidth', 1.5, 'DisplayName', 'Stage 1');  % Stage 1
plot(t_combined_all(len_stage1+1:len_stage1+len_stage2), velocity_combined_all(len_stage1+1:len_stage1+len_stage2), colors{2}, 'LineWidth', 1.5, 'DisplayName', 'Stage 2');  % Stage 2
plot(t_combined_all(len_stage1+len_stage2+1:len_stage1+len_stage2+len_coast), velocity_combined_all(len_stage1+len_stage2+1:len_stage1+len_stage2+len_coast), colors{3}, 'LineWidth', 1.5, 'DisplayName', 'Coasting');  % Coasting Phase
plot(t_combined_all(len_stage1+len_stage2+len_coast+1:end), velocity_combined_all(len_stage1+len_stage2+len_coast+1:end), colors{4}, 'LineWidth', 1.5, 'DisplayName', 'Steering Law Phase');  % Steering Phase
xlabel('Time (s)');
ylabel('Velocity (km/s)');
title('Velocity (All Stages)');
legend('Location', 'best');
grid on;
hold off;

% Altitude plot
subplot(3, 1, 2);
hold on;
plot(t_combined_all(1:len_stage1), altitude_combined_all(1:len_stage1), colors{1}, 'LineWidth', 1.5, 'DisplayName', 'Stage 1');  % Stage 1
plot(t_combined_all(len_stage1+1:len_stage1+len_stage2), altitude_combined_all(len_stage1+1:len_stage1+len_stage2), colors{2}, 'LineWidth', 1.5, 'DisplayName', 'Stage 2');  % Stage 2
plot(t_combined_all(len_stage1+len_stage2+1:len_stage1+len_stage2+len_coast), altitude_combined_all(len_stage1+len_stage2+1:len_stage1+len_stage2+len_coast), colors{3}, 'LineWidth', 1.5, 'DisplayName', 'Coasting');  % Coasting Phase
plot(t_combined_all(len_stage1+len_stage2+len_coast+1:end), altitude_combined_all(len_stage1+len_stage2+len_coast+1:end), colors{4}, 'LineWidth', 1.5, 'DisplayName', 'Steering Law Phase');  % Steering Phase
xlabel('Time (s)');
ylabel('Altitude (km)');
title('Altitude (All Stages)');
legend('Location', 'best');
grid on;
hold off;

% Mass plot
subplot(3, 1, 3);
hold on;
plot(t_combined_all(1:len_stage1), mass_combined_all(1:len_stage1), colors{1}, 'LineWidth', 1.5, 'DisplayName', 'Stage 1');  % Stage 1
plot(t_combined_all(len_stage1+1:len_stage1+len_stage2), mass_combined_all(len_stage1+1:len_stage1+len_stage2), colors{2}, 'LineWidth', 1.5, 'DisplayName', 'Stage 2');  % Stage 2
plot(t_combined_all(len_stage1+len_stage2+1:len_stage1+len_stage2+len_coast), mass_combined_all(len_stage1+len_stage2+1:len_stage1+len_stage2+len_coast), colors{3}, 'LineWidth', 1.5, 'DisplayName', 'Coasting');  % Coasting Phase
plot(t_combined_all(len_stage1+len_stage2+len_coast+1:end), mass_combined_all(len_stage1+len_stage2+len_coast+1:end), colors{4}, 'LineWidth', 1.5, 'DisplayName', 'Steering Law Phase');  % Steering Phase
xlabel('Time (s)');
ylabel('Mass (kg)');
title('Mass (All Stages)');
legend('Location', 'best');
grid on;
hold off;


figure;
subplot(4, 1, 1);
plot(t, state(:, 1)); % Velocity 
xlabel('Time (s)'); ylabel('Velocity (km/s)'); title('Velocity');


subplot(4, 1, 2);
plot(t, state(:, 3)); % Arc length 
xlabel('Time (s)'); ylabel('Arc Length'); title('Arc Length ');

subplot(4, 1, 3);
plot(t, state(:, 4)); % Altitude 
xlabel('Time (s)'); ylabel('Altitude'); title('Altitude');

subplot(4, 1, 4);
plot(t, state(:, 5)); % Our boy losing weight
xlabel('Time (s)'); ylabel('Mass'); title('Mass');

figure;
gamma_in_degrees = rad2deg(state(:, 2)); % Convert radians to degrees
plot(t, gamma_in_degrees, 'r');
xlabel('Time (s)');
ylabel('Flight Path Angle (degrees)');
title('Flight Path Angle vs Time');
grid on;


% Define the lengths of each phase to segment the data correctly
len_stage1 = length(t);
len_stage2 = length(t_stage2);
len_coast = length(t_coast);
len_guidance = length(t_values_stage2);

% Plot flight path angle change for all four stages with different colors for each phase
figure;
hold on;

line_thickness = 2.5;

% Highlight each phase with different colors and thicker lines
plot(t_combined_all_gamma(1:len_stage1), gamma_combined_all(1:len_stage1), 'r', 'LineWidth', line_thickness, 'DisplayName', 'Stage 1');  % Stage 1
plot(t_combined_all_gamma(len_stage1+1:len_stage1+len_stage2), gamma_combined_all(len_stage1+1:len_stage1+len_stage2), 'g', 'LineWidth', line_thickness, 'DisplayName', 'Stage 2');  % Stage 2
plot(t_combined_all_gamma(len_stage1+len_stage2+1:len_stage1+len_stage2+len_coast), gamma_combined_all(len_stage1+len_stage2+1:len_stage1+len_stage2+len_coast), 'b', 'LineWidth', line_thickness, 'DisplayName', 'Coasting');  % Coasting Phase
plot(t_combined_all_gamma(len_stage1+len_stage2+len_coast+1:end), gamma_combined_all(len_stage1+len_stage2+len_coast+1:end), 'm', 'LineWidth', line_thickness, 'DisplayName', 'Steering Law Phase');  % Steering Phase

% Labels and title
xlabel('Time (s)');
ylabel('Flight Path Angle (degrees)');
title('Flight Path Angle vs Time (All Stages Highlighted)');
legend('Location', 'best');
grid on;
hold off;


% Define the lengths of each phase to segment the data correctly
len_stage1 = length(t);
len_stage2 = length(t_stage2);
len_coast = length(t_coast);
len_guidance = length(t2);

% Plot Rocket Flight Path (Altitude vs Down-Range Distance) for all four stages
figure;
hold on;

% Highlight each phase with different colors and labels
plot(arc_length_combined_all(1:len_stage1), altitude_combined_all(1:len_stage1), 'r', 'LineWidth', 1.5, 'DisplayName', 'Stage 1');  % Stage 1
plot(arc_length_combined_all(len_stage1+1:len_stage1+len_stage2), altitude_combined_all(len_stage1+1:len_stage1+len_stage2), 'g', 'LineWidth', 1.5, 'DisplayName', 'Stage 2');  % Stage 2
plot(arc_length_combined_all(len_stage1+len_stage2+1:len_stage1+len_stage2+len_coast), altitude_combined_all(len_stage1+len_stage2+1:len_stage1+len_stage2+len_coast), 'b', 'LineWidth', 1.5, 'DisplayName', 'Coasting');  % Coasting Phase
plot(arc_length_combined_all(len_stage1+len_stage2+len_coast+1:end), altitude_combined_all(len_stage1+len_stage2+len_coast+1:end), 'm', 'LineWidth', 1.5, 'DisplayName', 'Steering Law Phase');  % Steering Phase

% Labels and title
xlabel('Down-Range Distance (Arc Length) (km)');
ylabel('Altitude (km)');
title('Altitude vs Down-Range Distance (All Stages Highlighted)');
legend('Location', 'best');
grid on;
hold off;



% Calculate and display the absolute final velocity and final altitude at the end of the simulation
final_velocity = velocity_combined_all(end); % Get the final velocity from the combined results
final_altitude = altitude_combined_all(end); % Get the final altitude from the combined results

% Calculate the needed orbital speed at the final altitude
R_earth = 6378; % Earth's radius in km
v_c = sqrt(mu / (R_earth + final_altitude)); % Orbital speed at the final altitude

% Calculate the remaining fuel after the first stage separation
initial_mass_stage1 = m_tot; % Initial total mass of the rocket
final_mass_stage1 = state(end, 5); % Mass at the end of stage 1 (after fuel burn)
fuel_consumed_stage1 = initial_mass_stage1 - final_mass_stage1; % Fuel consumed during stage 1
remaining_fuel_stage1 = m_fuel1 - fuel_consumed_stage1; % Remaining fuel in stage 1 tank

% Calculate the total time elapsed during the simulation
total_time_lapsed = t_combined_all(end); % Total time from the combined time vector

% Display the final results
disp(['Final Velocity: ', num2str(final_velocity), ' km/s']); % Display the final velocity
disp(['Final Altitude: ', num2str(final_altitude), ' km']);   % Display the final altitude
disp(['Needed Speed to Stay in Orbit: ', num2str(v_c), ' km/s']); % Display the needed orbital speed
disp(['Remaining Fuel in Stage 1: ', num2str(remaining_fuel_stage1), ' kg']); % Display the remaining fuel in stage 1
disp(['Total Time Lapsed: ', num2str(total_time_lapsed), ' s']); % Display the total time lapsed in seconds





% Display Rocket Specifications for Each Stage
disp('============================================');
disp('Rocket Specifications:');
disp('============================================');

% Stage 1 Specifications
disp('Stage 1 Specifications:');
disp(['  Initial Total Mass: ', num2str(m_tot), ' kg']);
disp(['  Fuel Mass: ', num2str(m_fuel1), ' kg']);
disp(['  Structural Mass: ', num2str(m_structure1), ' kg']);
disp(['  Payload Mass: ', num2str(m_payload), ' kg']);
disp(['  Specific Impulse (Isp): ', num2str(I_sp), ' s']);
disp(['  Thrust: ', num2str(T*10^3), ' N']);
disp(['  Mass Flow Rate: ', num2str(m_dot), ' kg/s']);
disp(' '); % Empty line for better readability

% Stage 2 Specifications
disp('Stage 2 Specifications:');
disp(['  Initial Stage Mass: ', num2str(m_tot_stage2), ' kg']);
disp(['  Fuel Mass: ', num2str(m_fuel2), ' kg']);
disp(['  Structural Mass: ', num2str(m_structure2), ' kg']);
disp(['  Specific Impulse (Isp): ', num2str(I_sp2), ' s']);
disp(['  Thrust: ', num2str(T2), ' N']);
disp(['  Mass Flow Rate: ', num2str(m_dot2), ' kg/s']);
disp(' '); % Empty line for better readability

% Coasting Phase Specifications (after Stage 2 cutoff)
disp('Coasting Phase Specifications:');
disp(['  Initial Coasting Mass: ', num2str(initial_mass_coast), ' kg']);
disp(['  Final Altitude: ', num2str(final_altitude), ' km']);
disp(['  Final Velocity: ', num2str(final_velocity), ' km/s']);
disp(['  Needed Speed to Stay in Orbit: ', num2str(v_c), ' km/s']);
disp(' '); % Empty line for better readability

% General Rocket Properties
disp('General Rocket Properties:');
disp(['  Radius of Earth (R_earth): ', num2str(R_earth), ' km']);
disp(['  Gravitational Parameter (mu): ', num2str(mu), ' km^3/s^2']);
disp('============================================');






% Plot Rocket Flight Path (Altitude vs Down-Range Distance) for all four stages
figure('Color', 'k');  % Set figure background color to black
hold on;
ax = gca;  % Get the current axes
set(ax, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', 'w', 'GridAlpha', 0.5);  % Set axes properties

% Highlight each phase with different colors and labels
plot(arc_length_combined_all(1:len_stage1), altitude_combined_all(1:len_stage1), 'r', 'LineWidth', 1.5, 'DisplayName', 'Stage 1');  % Stage 1
plot(arc_length_combined_all(len_stage1+1:len_stage1+len_stage2), altitude_combined_all(len_stage1+1:len_stage1+len_stage2), 'g', 'LineWidth', 1.5, 'DisplayName', 'Stage 2');  % Stage 2
plot(arc_length_combined_all(len_stage1+len_stage2+1:len_stage1+len_stage2+len_coast), altitude_combined_all(len_stage1+len_stage2+1:len_stage1+len_stage2+len_coast), 'b', 'LineWidth', 1.5, 'DisplayName', 'Coasting');  % Coasting Phase
plot(arc_length_combined_all(len_stage1+len_stage2+len_coast+1:end), altitude_combined_all(len_stage1+len_stage2+len_coast+1:end), 'm', 'LineWidth', 1.5, 'DisplayName', 'Steering Law Phase');  % Steering Phase

% Labels and title with white color for better contrast
xlabel('Down-Range Distance (Arc Length) (km)', 'Color', 'w');
ylabel('Altitude (km)', 'Color', 'w');
title('Altitude vs Down-Range Distance (All Stages Highlighted)', 'Color', 'w');
legend('Location', 'best', 'TextColor', 'w');  % Set legend text color to white for visibility
grid on;
hold off;





% Plot flight path angle change for all four stages with different colors for each phase
figure('Color', 'k');  % Set figure background color to black
hold on;

% Configure axes properties for the black background
ax_flight_path_angle = gca;  % Get the current axes
set(ax_flight_path_angle, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', 'w', 'GridAlpha', 0.5);  % Set axes properties

line_thickness = 2.5;

% Highlight each phase with different colors and thicker lines
plot(t_combined_all_gamma(1:len_stage1), gamma_combined_all(1:len_stage1), 'r', 'LineWidth', line_thickness, 'DisplayName', 'Stage 1');  % Stage 1
plot(t_combined_all_gamma(len_stage1+1:len_stage1+len_stage2), gamma_combined_all(len_stage1+1:len_stage1+len_stage2), 'g', 'LineWidth', line_thickness, 'DisplayName', 'Stage 2');  % Stage 2
plot(t_combined_all_gamma(len_stage1+len_stage2+1:len_stage1+len_stage2+len_coast), gamma_combined_all(len_stage1+len_stage2+1:len_stage1+len_stage2+len_coast), 'b', 'LineWidth', line_thickness, 'DisplayName', 'Coasting');  % Coasting Phase
plot(t_combined_all_gamma(len_stage1+len_stage2+len_coast+1:end), gamma_combined_all(len_stage1+len_stage2+len_coast+1:end), 'm', 'LineWidth', line_thickness, 'DisplayName', 'Steering Law Phase');  % Steering Phase

% Set labels and title with white color for better contrast
xlabel('Time (s)', 'Color', 'w');
ylabel('Flight Path Angle (degrees)', 'Color', 'w');
title('Flight Path Angle vs Time (All Stages Highlighted)', 'Color', 'w');
legend('Location', 'best', 'TextColor', 'w');  % Set legend text color to white for visibility
grid on;  % Enable grid
hold off;






% Combined Plots for Velocity, Altitude, and Mass with Black Background
figure('Color', 'k');  % Set figure background color to black

% Velocity plot
subplot(3, 1, 1);
hold on;
ax_velocity = gca;  % Get the current axes
set(ax_velocity, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', 'w', 'GridAlpha', 0.5);  % Set axes properties
plot(t_combined_all(1:len_stage1), velocity_combined_all(1:len_stage1), colors{1}, 'LineWidth', 1.5, 'DisplayName', 'Stage 1');
plot(t_combined_all(len_stage1+1:len_stage1+len_stage2), velocity_combined_all(len_stage1+1:len_stage1+len_stage2), colors{2}, 'LineWidth', 1.5, 'DisplayName', 'Stage 2');
plot(t_combined_all(len_stage1+len_stage2+1:len_stage1+len_stage2+len_coast), velocity_combined_all(len_stage1+len_stage2+1:len_stage1+len_stage2+len_coast), colors{3}, 'LineWidth', 1.5, 'DisplayName', 'Coasting');
plot(t_combined_all(len_stage1+len_stage2+len_coast+1:end), velocity_combined_all(len_stage1+len_stage2+len_coast+1:end), colors{4}, 'LineWidth', 1.5, 'DisplayName', 'Steering Law Phase');
xlabel('Time (s)', 'Color', 'w');
ylabel('Velocity (km/s)', 'Color', 'w');
title('Velocity (All Stages)', 'Color', 'w');
legend('Location', 'best', 'TextColor', 'w');  % Set legend text color to white for visibility
grid on;
hold off;

% Altitude plot
subplot(3, 1, 2);
hold on;
ax_altitude = gca;  % Get the current axes
set(ax_altitude, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', 'w', 'GridAlpha', 0.5);  % Set axes properties
plot(t_combined_all(1:len_stage1), altitude_combined_all(1:len_stage1), colors{1}, 'LineWidth', 1.5, 'DisplayName', 'Stage 1');
plot(t_combined_all(len_stage1+1:len_stage1+len_stage2), altitude_combined_all(len_stage1+1:len_stage1+len_stage2), colors{2}, 'LineWidth', 1.5, 'DisplayName', 'Stage 2');
plot(t_combined_all(len_stage1+len_stage2+1:len_stage1+len_stage2+len_coast), altitude_combined_all(len_stage1+len_stage2+1:len_stage1+len_stage2+len_coast), colors{3}, 'LineWidth', 1.5, 'DisplayName', 'Coasting');
plot(t_combined_all(len_stage1+len_stage2+len_coast+1:end), altitude_combined_all(len_stage1+len_stage2+len_coast+1:end), colors{4}, 'LineWidth', 1.5, 'DisplayName', 'Steering Law Phase');
xlabel('Time (s)', 'Color', 'w');
ylabel('Altitude (km)', 'Color', 'w');
title('Altitude (All Stages)', 'Color', 'w');
legend('Location', 'best', 'TextColor', 'w');  % Set legend text color to white for visibility
grid on;
hold off;

% Mass plot
subplot(3, 1, 3);
hold on;
ax_mass = gca;  % Get the current axes
set(ax_mass, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', 'w', 'GridAlpha', 0.5);  % Set axes properties
plot(t_combined_all(1:len_stage1), mass_combined_all(1:len_stage1), colors{1}, 'LineWidth', 1.5, 'DisplayName', 'Stage 1');
plot(t_combined_all(len_stage1+1:len_stage1+len_stage2), mass_combined_all(len_stage1+1:len_stage1+len_stage2), colors{2}, 'LineWidth', 1.5, 'DisplayName', 'Stage 2');
plot(t_combined_all(len_stage1+len_stage2+1:len_stage1+len_stage2+len_coast), mass_combined_all(len_stage1+len_stage2+1:len_stage1+len_stage2+len_coast), colors{3}, 'LineWidth', 1.5, 'DisplayName', 'Coasting');
plot(t_combined_all(len_stage1+len_stage2+len_coast+1:end), mass_combined_all(len_stage1+len_stage2+len_coast+1:end), colors{4}, 'LineWidth', 1.5, 'DisplayName', 'Steering Law Phase');
xlabel('Time (s)', 'Color', 'w');
ylabel('Mass (kg)', 'Color', 'w');
title('Mass (All Stages)', 'Color', 'w');
legend('Location', 'best', 'TextColor', 'w');  % Set legend text color to white for visibility
grid on;
hold off;














% Determine the stage separation points and phase change indices
sep_index_stage1 = length(t);                         % End of Stage 1
sep_index_stage2 = sep_index_stage1 + length(t_stage2); % Stage 2 Engine Cut Off
sep_index_coast = sep_index_stage2 + length(t_coast);   % End of Coasting Phase

% Altitude and Arc Length at each separation
sep_arc_length_stage1 = arc_length_combined_all(sep_index_stage1);
sep_altitude_stage1 = altitude_combined_all(sep_index_stage1);

sep_arc_length_stage2 = arc_length_combined_all(sep_index_stage2);
sep_altitude_stage2 = altitude_combined_all(sep_index_stage2);

sep_arc_length_coast = arc_length_combined_all(sep_index_coast);
sep_altitude_coast = altitude_combined_all(sep_index_coast);

% Create a figure for the animation
figure;
h = plot(arc_length_combined_all(1), altitude_combined_all(1), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8); % Initial rocket position
hold on;
xlabel('Down-Range Distance (Arc Length) (km)');
ylabel('Altitude (km)');
title('Rocket Flight Path with Stage Separation and Phases');
grid on;

% Plot the entire flight path for reference
plot(arc_length_combined_all, altitude_combined_all, 'b--');  % Plot full trajectory

% Set axis limits
xlim([0 max(arc_length_combined_all)]);
ylim([0 max(altitude_combined_all)]);

% Mark the stage separation points (black circles)
sep_marker_stage1 = plot(sep_arc_length_stage1, sep_altitude_stage1, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 10);
sep_marker_stage2 = plot(sep_arc_length_stage2, sep_altitude_stage2, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 10);
sep_marker_coast = plot(sep_arc_length_coast, sep_altitude_coast, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 10);

% Hide the markers until their respective separations
set(sep_marker_stage1, 'Visible', 'off');
set(sep_marker_stage2, 'Visible', 'off');
set(sep_marker_coast, 'Visible', 'off');

% Text annotations for each phase (initially hidden)
text_stage1 = text(sep_arc_length_stage1, sep_altitude_stage1, 'Stage 1 Complete', 'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom', 'FontSize', 12, 'Color', 'r');
text_stage2 = text(sep_arc_length_stage2, sep_altitude_stage2, 'Stage 2 Engine CutOff', 'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom', 'FontSize', 12, 'Color', 'g');
text_coast = text(sep_arc_length_coast, sep_altitude_coast, 'Coasting Complete', 'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom', 'FontSize', 12, 'Color', 'b');
text_reignition = text(sep_arc_length_coast, sep_altitude_coast - 10, 'Stage 2 Re-Ignition', 'HorizontalAlignment', 'left', 'VerticalAlignment', 'top', 'FontSize', 12, 'Color', 'm'); % Added Re-Ignition text

% Hide text annotations until the relevant phase is reached
set(text_stage1, 'Visible', 'off');
set(text_stage2, 'Visible', 'off');
set(text_coast, 'Visible', 'off');
set(text_reignition, 'Visible', 'off');

% Animation loop using pre-calculated data
for i = 1:length(arc_length_combined_all)
    % Update the rocket's position on the path
    set(h, 'XData', arc_length_combined_all(i), 'YData', altitude_combined_all(i)); 

    % Check if we have reached the stage separation points
    if i == sep_index_stage1
        set(sep_marker_stage1, 'Visible', 'on');  % Show the first stage separation marker
        set(text_stage1, 'Visible', 'on');  % Show the text annotation
        pause(0.5);  % Pause to highlight the separation event
    elseif i == sep_index_stage2
        set(sep_marker_stage2, 'Visible', 'on');  % Show the second stage separation marker
        set(text_stage2, 'Visible', 'on');  % Show the text annotation
        pause(0.5);  % Pause to highlight the separation event
    elseif i == sep_index_coast
        set(sep_marker_coast, 'Visible', 'on');  % Show the coasting phase marker
        set(text_coast, 'Visible', 'on');  % Show the text annotation
        set(text_reignition, 'Visible', 'on');  % Show the re-ignition text annotation
        pause(0.5);  % Pause to highlight the separation event
    end

    % Control the animation speed
    pause(0.01);
end

% Finalize the plot after the animation completes
set(h, 'XData', arc_length_combined_all, 'YData', altitude_combined_all); % Ensure the full flight path is displayed












function dydt = rocket_dynamics(t, y, mu, m_dot, T, C_d, A)
    R_earth = 6378;
    v = y(1);
    gamma = y(2);
    X = y(3);
    H = y(4);

    m = y(5);


    g = (mu / (R_earth + H)^2);

     % Atmosphere table for altitudes above 84 km
    altitude_table = [150, 200, 250, 300, 350, 400, 450, 500, 550, 600, 650, 700, 750, 800, 850, 900, 950, 1000];
    density_table = [2.076e-10, 2.541e-10, 6.073e-11, 1.916e-11, 7.014e-12, 2.803e-12, 1.184e-12, 5.215e-13, 2.384e-13, 1.137e-13, 5.712e-14, 3.070e-14, 1.788e-14, 1.136e-14, 7.824e-15, 5.759e-15, 4.435e-15, 3.561e-15];

    if H < 84
        [Temp, a, P, rho] = atmosisa([H*1000], 'extended', true);
        rho = rho * 10^9; % Convert to consistent units
    elseif H > 84
        % Linear interpolation for density based on altitude
         rho = interp1(altitude_table, density_table, H, 'linear', 'extrap') * 10^9;
     end


    % Define the tilt change start and end times
    t_start = 10;
    t_end = 13;
    tolerance = 1e-2;  % Define a tolerance value to handle non-uniform steps

     if t < 10

        dgamma_dt = 0;  
        gamma = deg2rad(89.8);
        dXdt = 0;
    elseif t >= t_start && t <= t_end
        t;
        5;
        % Interpolate gamma smoothly between 89.8 degrees and 88.5 degrees from t=10 to t=11
        gamma_start = deg2rad(89.8);
        gamma_end = deg2rad(88.0);
        gamma = gamma_start + (gamma_end - gamma_start) * (t - t_start) / (t_end - t_start);
        dgamma_dt = 0; % Set gamma as a fixed value during this phase
        dXdt = (R_earth/(R_earth+H))*v*cos(gamma);%v * cos(y(2));
        

    else
        dgamma_dt = -(1/v) * (g - v^2/(R_earth + H)) * cos(gamma);
        dXdt = (R_earth / (R_earth + H)) * v * cos(gamma);
    end


    




    D = 0.5 * rho * v^2 * C_d * A;

    %Five golden equations for the gravity turn
    dVdt = (T/y(5)) - (D/y(5)) - g*sin(y(2));
    %dgamma_dt = -(1/y(1)) * (g - (y(1)^2/(R_earth + H))) * cos(y(2));
    % dXdt = v * cos(y(2))%(R_earth/(R_earth+H))*v*cos(gamma)%
    dHdt = v * sin(y(2));
    dmdt = m_dot; 


    dydt = [dVdt; dgamma_dt; dXdt; dHdt; dmdt];

end


function dydt = rocket_dynamics_stage2_engine(t, y, mu, m_dot2, T2, C_d, A)
    R_earth = 6378;  % Radius of Earth in km
    v = y(1);        % Velocity (km/s)
    gamma = y(2);    % Flight path angle (radians)
    X = y(3);        % Arc length (km)
    H = y(4);        % Altitude (km)
    m = y(5);        % Mass (kg)

    % Gravity at current altitude
    g = (mu / (R_earth + H)^2);

    % Atmospheric density
    altitude_table = [150, 200, 250, 300, 350, 400, 450, 500, 550, 600, 650, 700, 750, 800, 850, 900, 950, 1000];
    density_table = [2.076e-10, 2.541e-10, 6.073e-11, 1.916e-11, 7.014e-12, 2.803e-12, 1.184e-12, 5.215e-13, 2.384e-13, 1.137e-13, 5.712e-14, 3.070e-14, 1.788e-14, 1.136e-14, 7.824e-15, 5.759e-15, 4.435e-15, 3.561e-15];
    rho = interp1(altitude_table, density_table, H, 'linear', 'extrap') * 10^9;

    % Drag force
    D = 0.5 * rho * v^2 * C_d * A;

    % Rocket dynamics equations
    dVdt = (T2/m) - (D/m) - g*sin(gamma);                % Change in velocity
    dgamma_dt = -(1/v) * (g - v^2/(R_earth + H)) * cos(gamma);  % Change in flight path angle
    dXdt = (R_earth/(R_earth + H)) * v * cos(gamma);  % Change in arc length
    dHdt = v * sin(gamma);  % Change in altitude
    dmdt = m_dot2;          % Mass flow rate

    % Output derivatives
    dydt = [dVdt; dgamma_dt; dXdt; dHdt; dmdt];
end






function dydt = rocket_dynamics_coast(t, y, mu, C_d, A)
    % Parameters for coasting phase
    R_earth = 6378;  % Radius of Earth in km
    v = y(1);        % Velocity (km/s)
    gamma = y(2);    % Flight path angle (radians)
    X = y(3);        % Arc length (km)
    H = y(4);        % Altitude (km)

    % Gravity at current altitude
    g = (mu / (R_earth + H)^2);

    % Atmospheric density
    altitude_table = [150, 200, 250, 300, 350, 400, 450, 500, 550, 600, 650, 700, 750, 800, 850, 900, 950, 1000];
    density_table = [2.076e-10, 2.541e-10, 6.073e-11, 1.916e-11, 7.014e-12, 2.803e-12, 1.184e-12, 5.215e-13, 2.384e-13, 1.137e-13, 5.712e-14, 3.070e-14, 1.788e-14, 1.136e-14, 7.824e-15, 5.759e-15, 4.435e-15, 3.561e-15];

    rho = interp1(altitude_table, density_table, H, 'linear', 'extrap') * 10^9;  % Convert to consistent units
    rho_testing = interp1(altitude_table, density_table, H, 'linear', 'extrap');
    % Drag force
    D = 0.5 * rho * v^2 * C_d * A;

    % Coasting dynamics (no thrust, constant mass)
    dVdt = -D - g * sin(gamma);   % Change in velocity
    dgamma_dt = -(1/v) * (g - v^2/(R_earth + H)) * cos(gamma);  % Change in flight path angle
    dXdt = (R_earth/(R_earth + H)) * v * cos(gamma);  % Change in arc length
    dHdt = v * sin(gamma);  % Change in altitude

    % No change in mass during coasting phase
    dmdt = 0;

    % Output derivatives
    dydt = [dVdt; dgamma_dt; dXdt; dHdt; dmdt];
end



function dydt = rocket_dynamics_stage2(t, y, mu, m_dot, T, C_d, A,m_fuel_guidance, t_burnout_stage2, gamma0_2)
    global gamma_values_stage2 t_counter gamma_counter t_values_stage2;
    R_earth = 6378;
    v = y(1);
    gammabla = y(2);
    X = y(3);
    H = y(4);
    m = y(5);
    g = (mu / (R_earth + H)^2);

     % Atmosphere table for altitudes above 84 km
    altitude_table = [150, 200, 250, 300, 350, 400, 450, 500, 550, 600, 650, 700, 750, 800, 850, 900, 950, 1000];
    density_table = [2.076e-10, 2.541e-10, 6.073e-11, 1.916e-11, 7.014e-12, 2.803e-12, 1.184e-12, 5.215e-13, 2.384e-13, 1.137e-13, 5.712e-14, 3.070e-14, 1.788e-14, 1.136e-14, 7.824e-15, 5.759e-15, 4.435e-15, 3.561e-15];


    rho = interp1(altitude_table, density_table, H, 'linear', 'extrap') * 10^9;
    D = 0.5 * rho * v^2 * C_d * A ;
    % Linear tangent steering law for stage 2
    % tan(gamma(t)) = tan(gamma_0) * (1 - t / t_burnout_stage2)
    % Initial gamma passed from stage 1's final angle
   if t>t_burnout_stage2
       1;
   end
    gamma0_2;
    gamma_tangent = tan(gamma0_2) * (1 - t / t_burnout_stage2);
    gamma = atan(gamma_tangent); % Calculate the current gamma based on the steering law

    gamma_values_stage2 = [gamma_values_stage2; rad2deg(gamma)];
   t_values_stage2 = [t_values_stage2; t];
    % Increment counters
    t_counter = t_counter + 1;
    gamma_counter = gamma_counter + 1;

    % Display values
    disp(['t: ', num2str(t), ', Gamma: ', num2str(rad2deg(gamma))]);

    if m<=10000 || v>=7.35
        m
        rho;
        dVdt = 0 - (D/m) - g * sin(gamma);
        % dgamma_dt = -(1/v) * (g - v^2/(R_earth + H)) * cos(gamma);
        dXdt = (R_earth/(R_earth+H)) * v * cos(gamma);
        dHdt = v * sin(gamma);
        dmdt = 0;

    else
        2;
        % dgamma_dt = -(1/v) * (g - v^2/(R_earth + H)) * cos(gamma);

        dXdt = (R_earth/(R_earth+H)) * v * cos(gamma);
        dVdt = (T/m) - (D/m) - g * sin(gamma);
        dHdt = v * sin(gamma);
        dmdt = m_dot;

    end


t;
rad2deg(gamma);




    dydt = [dVdt; gamma; dXdt; dHdt; dmdt];
end
















