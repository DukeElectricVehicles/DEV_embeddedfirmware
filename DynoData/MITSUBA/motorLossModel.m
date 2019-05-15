lossPoly_aeroAndBearing = [-1.06527e-08	-6.50352e-07	-1.02305e-04	1.12781e-03];
lossPoly_eddy = [-1.11897e-08	-5.30369e-06	-6.07395e-03	3.03073e-02] - lossPoly_aeroAndBearing;

RPM = 250;

loss = table();
loss.types = {'aero and bearing';...
              'magnetic'};
loss.power = [polyval(lossPoly_aeroAndBearing, RPM);...
              polyval(lossPoly_eddy, RPM)];
          
loss