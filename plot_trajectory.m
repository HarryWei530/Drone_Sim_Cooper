load("output.mat");
state_output = logsout{6}.Values.Data;
time = logsout{6}.Values.Time;
x = state_output(:,1);
y = state_output(:,2);
z = state_output(:,3);

scatter3(x,y,z)
xlabel("x")
ylabel("y")
zlabel("z")