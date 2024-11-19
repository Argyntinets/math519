% Define system matrices for the continuous-time system
A = [0 1; -2 -3]; % Example continuous-time system matrix
B = [0; 1];       % Input matrix
C = [1 0];        % Output matrix
L = [1; 1];       % Observer gain
K = [-3 -4];      % Feedback gain

% Define sampling period
h = 0.1; % Sampling interval

% Compute discrete-time matrices
Ad = expm(A * h); % Discrete-time A matrix
Bd = integral(@(tau) expm(A * tau) * B, 0, h, 'ArrayValued', true); % Discrete-time B matrix
Ld = integral(@(tau) expm(A * tau) * L, 0, h, 'ArrayValued', true); % Discrete-time L matrix

% Define simulation time
T = 10; % Total simulation time
N = T / h; % Number of steps
t = 0:h:T; % Time vector

% Initial conditions
chi0 = [1; 0]; % Initial condition for chi
chi_v0 = [1; 0]; % Initial condition for chi_v

% Input function u(t)
u = @(t) sin(t); % Example input

% Quantizer settings for chi_v(t)
Theta = 10; % Quantization gain
Q_Theta = @(x) (1/Theta) * round(Theta * x); % Uniform quantizer

% Preallocate variables
chi = zeros(2, N+1);        % Standard system chi
chi_v = zeros(2, N+1);      % Encrypted system chi_v
chi(:, 1) = chi0;           % Initial condition for chi
chi_v(:, 1) = chi_v0;       % Initial condition for chi_v

% Simulate both systems
for k = 1:N
    % Standard Luenberger observer for chi(t)
    y_k = C * chi(:, k); % Output
    chi_dot = A * chi(:, k) + B * u(t(k)) + L * (y_k - C * chi(:, k));
    chi(:, k+1) = chi(:, k) + h * chi_dot; % Euler integration

    % Encrypted observer dynamics for chi_v(t)
    y_v_k = C * chi_v(:, k); % Output
    chi_v_dot = A * chi_v(:, k) + B * u(t(k)) + L * (y_v_k - C * chi_v(:, k));
    chi_v(:, k+1) = Q_Theta(chi_v(:, k) + h * chi_v_dot); % Euler + Quantization
end

% Compute absolute errors
abs_error_chi = abs(chi - chi_v); % Absolute error between chi(t) and chi_v(t)
mse_error_chi = mean(abs_error_chi.^2, 2); % Mean squared error

% Display results
disp('Mean Squared Error between chi(t) and chi_v(t):');
disp(mse_error_chi);

% Plot results
figure;
plot(t, chi(1, :), 'b-', 'LineWidth', 1.5); hold on;
plot(t, chi_v(1, :), 'r--', 'LineWidth', 1.5);
title('State Variable \chi_{1}(t) Comparison');
xlabel('Time (s)');
ylabel('\chi_{1}(t)');
legend('Standard \chi_{1}(t)', 'Encrypted \chi_{v,1}(t)');
grid on;

figure;
plot(t, chi(2, :), 'b-', 'LineWidth', 1.5); hold on;
plot(t, chi_v(2, :), 'r--', 'LineWidth', 1.5);
title('State Variable \chi_{2}(t) Comparison');
xlabel('Time (s)');
ylabel('\chi_{2}(t)');
legend('Standard \chi_{2}(t)', 'Encrypted \chi_{v,2}(t)');
grid on;

figure;
plot(t, abs_error_chi(1, :), 'k-', 'LineWidth', 1.5);
hold on;
plot(t, abs_error_chi(2, :), 'm--', 'LineWidth', 1.5);
title('Absolute Error between \chi(t) and \chi_v(t)');
xlabel('Time (s)');
ylabel('Absolute Error');
legend('\chi_{1} Error', '\chi_{2} Error');
grid on;
