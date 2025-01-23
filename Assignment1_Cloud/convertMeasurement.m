function [MeasurementConvert, FalseAlarmConvert,R]= convertMeasurement(Measurement, FalseAlarm, Sensor_Parameter)
  %% Initialization
  
   MeasurementConvert = [];
   FalseAlarmConvert = [];
   R =[];
   lamdatheta1= exp(-(Sensor_Parameter.azimuthSigma^2)/2);
   lamdatheta2= exp(-2*(Sensor_Parameter.azimuthSigma^2));

%% Unbias Conversion
   if ~isempty(Measurement)
       range = Measurement(1);
       Azi = Measurement(2);
       x_unbiased = Sensor_Parameter.Xpos + ((lamdatheta1^-1) * range * cos(Azi));
       y_unbiased = Sensor_Parameter.Ypos + ((lamdatheta1^-1) * range * sin(Azi));

       MeasurementConvert = [MeasurementConvert; x_unbiased, y_unbiased];
       Rp11 = (lamdatheta1^-2 - 2) * range^2 * cos(Azi)^2 + 0.5 * (range^2 + Sensor_Parameter.rangeSigma^2) * (1 + lamdatheta2 * cos(2 * Azi));
       Rp22 = (lamdatheta1^-2 - 2) * range^2 * sin(Azi)^2 + 0.5 * (range^2 + Sensor_Parameter.rangeSigma^2) * (1 - lamdatheta2 * cos(2 * Azi));
       Rp12 = (lamdatheta1^-2 - 2) * range^2 * cos(Azi) * sin(Azi) + (0.5 * (range^2 + Sensor_Parameter.rangeSigma^2)) * lamdatheta2 * sin(2 * Azi);
       R = [Rp11, Rp12; Rp12, Rp22];
   end
   
   for i = 1:size(FalseAlarm, 1)
        EachFalse_range = FalseAlarm(i, 1);
        EachFalse_azimuth = FalseAlarm(i, 2);
        fa_x = Sensor_Parameter.Xpos + (1 / lamdatheta1) * EachFalse_range * cos(EachFalse_azimuth);
        fa_y = Sensor_Parameter.Ypos + (1 / lamdatheta1) * EachFalse_range * sin(EachFalse_azimuth);
        FalseAlarmConvert = [FalseAlarmConvert; fa_x, fa_y];
   end


   

end