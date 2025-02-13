function Measurement = generateMeasurements(parameter, target_state) %combine measurement and falsealarm together
   Measurement =[];
   SensorXpos = parameter.Xpos;
   SensorYpos = parameter.Ypos;
   Pd = parameter.Pd;
   FalseRate = parameter.FalseDensity;
   RangeNoisestandD = parameter.rangeSigma;
   azimuthNoise = parameter.azimuthSigma;
   Lamda = FalseRate * (parameter.rangeUb  - parameter.rangeLb) * (parameter.AziUb  - parameter.AziLb);

   if rand(1) < Pd
        rangemeasurement = sqrt((target_state(1) - SensorXpos)^2 + (target_state(3) - SensorYpos)^2);
        rangemeasurement = rangemeasurement + randn * RangeNoisestandD * 0.5; %what is the accept noise range
        

        azimuthmeasurement = atan2(target_state(3)- SensorYpos , target_state(1)- SensorXpos);
        azimuthmeasurement = azimuthmeasurement + randn * azimuthNoise * 0.5; %what is the accept noise range
       
        Measurement = [Measurement;rangemeasurement,azimuthmeasurement];
        %disp(['Range: ', num2str(Measurement), ', Azimuth: ', num2str(azimuthmeasurement)]);
   end

   
   numberofFalse = poissrnd(Lamda);
   for i = 1:numberofFalse
       falseRange = (parameter.rangeUb  - parameter.rangeLb) * rand(1) + parameter.rangeLb;
       falseazimuth = (parameter.AziUb  - parameter.AziLb) * rand(1) + parameter.AziLb;
       Measurement = [Measurement; falseRange,falseazimuth];
   end

end