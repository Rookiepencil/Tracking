function [Measurement, FalseAlarm]= generateMeasurements(parameter, target_state)
   Measurement =[];
   FalseAlarm = [];
   SensorXpos = parameter.Xpos;
   SensorYpos = parameter.Ypos;
   Pd = parameter.Pd;
   FalseRate = parameter.FalseDensity;
   RangeNoise = parameter.rangeSigma;
   azimuthNoise = parameter.azimuthSigma;
   Lamda = FalseRate * (parameter.rangeUb  - parameter.rangeLb) * (parameter.AziUb  - parameter.AziLb);

   if rand(1) < Pd
        rangemeasurement = sqrt((target_state(1) - SensorXpos)^2 + (target_state(3) - SensorYpos)^2);
        rangemeasurement = rangemeasurement + randn * RangeNoise; %what is the accept noise range

        azimuthmeasurement = atan2(target_state(3)- SensorYpos , target_state(1)- SensorXpos);
        azimuthmeasurement = azimuthmeasurement + randn * azimuthNoise; %what is the accept noise range
        Measurement = [Measurement;rangemeasurement,azimuthmeasurement];
        disp(['Range: ', num2str(Measurement), ', Azimuth: ', num2str(azimuthmeasurement)]);
   end

   for i = 1:Lamda
       falseRange = (parameter.rangeUb  - parameter.rangeLb) * rand(1);
       falseazimuth = (parameter.AziUb  - parameter.AziLb) * rand(1);
       FalseAlarm = [FalseAlarm; falseRange,falseazimuth];
   end

end