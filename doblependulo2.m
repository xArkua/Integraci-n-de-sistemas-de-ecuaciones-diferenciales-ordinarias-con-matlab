%Parámetros del sistema
m1 = 0.2;
m2 = 0.1;
l1 = 0.3;
l2 = 0.25;
g = 9.81;

%Condiciones iniciales
theta1_0 = 0;
theta2_0 = deg2rad(35);
omega1_0 = 0;
omega2_0 = 0;

%vector Condiciones iniciales
y0 = [theta1_0; omega1_0; theta2_0; omega2_0];

%Simulación
tspan = [0 10];

%Sistema de ecuaciones diferenciales
[t, y] = ode45(@(t, y) double_pendulum_ode(t, y, m1, m2, l1, l2, g), tspan, y0);

%Función
function dydt = double_pendulum_ode(~, y, m1, m2, l1, l2, g)
    theta1 = y(1);
    omega1 = y(2);
    theta2 = y(3);
    omega2 = y(4);

    delta = theta2 - theta1;

    den1 = (m1 + m2) * l1 - m2 * l1 * cos(delta) * cos(delta);
    den2 = (l2 / l1) * den1;

    dydt = zeros(4, 1);
    dydt(1) = omega1;
    dydt(2) = (m2 * l1 * omega1 * omega1 * sin(delta) * cos(delta) + m2 * g * sin(theta2) * cos(delta) + m2 * l2 * omega2 * omega2 * sin(delta) - (m1 + m2) * g * sin(theta1)) / den1;
    dydt(3) = omega2;
    dydt(4) = (-m2 * l2 * omega2 * omega2 * sin(delta) * cos(delta) + (m1 + m2) * g * sin(theta1) * cos(delta) - (m1 + m2) * l1 * omega1 * omega1 * sin(delta) - (m1 + m2) * g * sin(theta2)) / den2;
end

%Graficar los resultados
figure;
subplot(2, 1, 1);
plot(t, y(:, 1), t, y(:, 3));
legend('\theta_1', '\theta_2');
xlabel('Tiempo (s)');
ylabel('Ángulo (rad)');
title('Posición del Doble Péndulo');

subplot(2, 1, 2);
plot(t, y(:, 2), t, y(:, 4));
legend('\omega_1', '\omega_2');
xlabel('Tiempo (s)');
ylabel('Velocidad Angular (rad/s)');
title('Velocidad del Doble Péndulo');