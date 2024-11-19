% Define system matrices A, B, and C
A = [0 1; -2 -3];  % Example matrix
B = [0; 1];        % Example input matrix
C = [1 0];         % Example output matrix

% Define the time span and initial condition
t_span = [0 100];   % Time interval for the solution
x0 = [1; 0];       % Initial state

% Define the control input u(t)
u = @(t) sin(t);  % Example input function

% Numerical Solution using ode45
dxdt = @(t, x) A * x + B * u(t);
[t_num, x_num] = ode45(dxdt, t_span, x0);

% Analytical Solution
% Define time points for evaluation
t_analytical = linspace(t_span(1), t_span(2), 1000);

% Compute the matrix exponential e^(At) for each time point
x_analytical = zeros(length(t_analytical), length(x0));
for i = 1:length(t_analytical)
    t_i = t_analytical(i);
    Phi = expm(A * t_i);  % State transition matrix e^(At)
    
    % Integral term for forced response
    integral_term = zeros(size(x0));
    for tau = linspace(0, t_i, 100)  % Use numerical integration
        integral_term = integral_term + expm(A * (t_i - tau)) * B * u(tau) * (t_i / 100);
    end
    
    % State solution
    x_analytical(i, :) = (Phi * x0 + integral_term)';
end

% Compute the output y(t) = Cx(t)
y_num = C * x_num.';  % Numerical output
y_analytical = (C * x_analytical.').';

% Interpolate analytical solution at numerical time points
x_analytical_interp = interp1(t_analytical, x_analytical, t_num, 'linear'); % Interpolation
y_analytical_interp = interp1(t_analytical, y_analytical, t_num, 'linear'); % Interpolation

% Compute Absolute Errors
abs_error_x = abs(x_num - x_analytical_interp); % Absolute error for state variables
abs_error_y = abs(C * x_num.' - C * x_analytical_interp.'); % Absolute error for output

% Compute Mean Squared Errors
mse_x = mean((x_num - x_analytical_interp).^2, 1); % MSE for each state variable
mse_y = mean((C * x_num.' - C * x_analytical_interp.').^2); % MSE for the output

% Display Errors
disp('Mean Squared Error for State Variables (x):');
disp(mse_x);
disp('Mean Squared Error for Output (y):');
disp(mse_y);

% Plot Absolute Errors
figure;
plot(t_num, abs_error_x, 'LineWidth', 1.2);
title('Absolute Error for State Variables');
xlabel('Time (s)');
ylabel('Absolute Error');
legend('x_1 Error', 'x_2 Error');
grid on;

figure;
plot(t_num, abs_error_y, 'LineWidth', 1.2);
title('Absolute Error for Output');
xlabel('Time (s)');
ylabel('Absolute Error');
legend('y Error');
grid on;