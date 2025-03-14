function [MeasurementT1, MeasurementT2] = generateMeasurements(parameter, target1_state, target2_state) %combine measurement and falsealarm together
  % Initialization and Necessary parameters
   MeasurementT1 = [];
   MeasurementT2 = [];
   SensorXpos = parameter.Xpos;
   SensorYpos = parameter.Ypos;
   Pd = parameter.Pd;
   FalseRate = parameter.FalseDensity;
   RangeNoisestandD = parameter.rangeSigma;
   azimuthNoise = parameter.azimuthSigma;
   Lamda = FalseRate * (parameter.rangeUb  - parameter.rangeLb) * (parameter.AziUb  - parameter.AziLb);


   if rand(1) < Pd %if we fall into Pd range we generate measurement, otherwise no measurement
       if isnan(target1_state)
           MeasurementT1 = [];
       else
        %range measurement
        rangemeasurement = sqrt((target1_state(1) - SensorXpos)^2 + (target1_state(3) - SensorYpos)^2);
        rangemeasurement = rangemeasurement + randn * RangeNoisestandD * 0.5; 
        
        % Azimuth generation 
        azimuthmeasurement = atan2(target1_state(3)- SensorYpos , target1_state(1)- SensorXpos);
        azimuthmeasurement = azimuthmeasurement + randn * azimuthNoise * 0.5;
       
        MeasurementT1 = [MeasurementT1;rangemeasurement,azimuthmeasurement];
       end

        if  isnan(target2_state)
           MeasurementT2 = [];
        else
        %range measurement
        rangemeasurement = sqrt((target2_state(1) - SensorXpos)^2 + (target2_state(3) - SensorYpos)^2);
        rangemeasurement = rangemeasurement + randn * RangeNoisestandD; 
        
        % Azimuth generation 
        azimuthmeasurement = atan2(target2_state(3)- SensorYpos , target2_state(1)- SensorXpos);
        azimuthmeasurement = azimuthmeasurement + randn * azimuthNoise;
       
        MeasurementT2 = [MeasurementT2;rangemeasurement,azimuthmeasurement];
       end


   end

   %False alarm generation
   numberofFalseT1 = poissrnd(Lamda); %calculate corresponding false alarm numbers
   numberofFalseT2 = poissrnd(Lamda);

   for i = 1:numberofFalseT1 % Actual false alarm generation for target 1
       falseRange = (parameter.rangeUb  - parameter.rangeLb) * rand(1) + parameter.rangeLb;
       falseazimuth = (parameter.AziUb  - parameter.AziLb) * rand(1) + parameter.AziLb;
       MeasurementT1 = [MeasurementT1; falseRange,falseazimuth];
   end

   for k = 1:numberofFalseT2 % Actual false alarm generation for target 2
       falseRange = (parameter.rangeUb  - parameter.rangeLb) * rand(1) + parameter.rangeLb;
       falseazimuth = (parameter.AziUb  - parameter.AziLb) * rand(1) + parameter.AziLb;
       MeasurementT2 = [MeasurementT2; falseRange,falseazimuth];
   end

end