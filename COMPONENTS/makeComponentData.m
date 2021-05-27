load BatteryComponentData;
load MotorComponentData;
load PropellerComponentData;

ComponentDatabase = vertcat(BatteryComponentData, MotorComponentData, PropellerComponentData);

save ComponentDatabase.mat ComponentDatabase