function [MeasurementConvert,Rp11,Rp22,Rp12]= convertMeasurement(Measurement, Sensor_Parameter)
  %% Initialization
   MeasurementConvert = [];
   R =[];
   %Lamba 1 & 2 Calculation
   lamdatheta1= exp(-(Sensor_Parameter.azimuthSigma^2)/2); 
   lamdatheta2= exp(-2*(Sensor_Parameter.azimuthSigma^2));
%% Unbias Conversion
   for i = 1:size(Measurement, 1)
       %Extract each range measurement and Azimuth measurement
       range = Measurement(i,1);
       Azi = Measurement(i,2);
       x_unbiased = Sensor_Parameter.Xpos + ((lamdatheta1^-1) * range * cos(Azi));
       y_unbiased = Sensor_Parameter.Ypos + ((lamdatheta1^-1) * range * sin(Azi));
       MeasurementConvert = [MeasurementConvert; x_unbiased, y_unbiased]; %Attach Each new converted measurement into new matrix for further processing
       
       % Covariance Matrix Calculation
       Rp11 = (lamdatheta1^-2 - 2) * range^2 * cos(Azi)^2 + 0.5 * (range^2 + Sensor_Parameter.rangeSigma^2) * (1 + lamdatheta2 * cos(2 * Azi));
       Rp22 = (lamdatheta1^-2 - 2) * range^2 * sin(Azi)^2 + 0.5 * (range^2 + Sensor_Parameter.rangeSigma^2) * (1 - lamdatheta2 * cos(2 * Azi));
       Rp12 = (lamdatheta1^-2 - 2) * range^2 * cos(Azi) * sin(Azi) + (0.5 * (range^2 + Sensor_Parameter.rangeSigma^2)) * lamdatheta2 * sin(2 * Azi);
       %R = [R;[Rp11, Rp12; Rp12, Rp22]]; %Attach Each new covariance matrix into new matrix for further processing
   end
end