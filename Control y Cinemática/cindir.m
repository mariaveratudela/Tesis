function [x1,x2,x3,x4] = cindir(x,y,theta)
    %Variables del robot movil
    %alpha1=56.95143; alpha2=326.9514319; alpha3=146.9514132; alpha4=236.9514319;
    %betha1=33.04820562;betha2=-33.04820562;betha3=-33.04820562;betha4=33.04820562;
    %y1=45;y2=-45;y3=y2;y4=y1;
    alpha1=0.99399; alpha2=5.70637898; alpha3=2.564786; alpha4=4.135582654;
    betha1=0.5768;betha2=-0.5768;betha3=-0.5768;betha4=0.5768;
    y1=pi/4;y2=-pi/4;y3=y2;y4=y1;

    l=0.24755 ; r = 0.05;

    A = [ sin(alpha1+betha1+y1) -cos(alpha1+betha1+y1) -l*cos(betha1+y1)
         -sin(alpha2+betha2+y2) cos(alpha2+betha2+y2) l*cos(betha2+y2)
          sin(alpha3+betha3+y3) -cos(alpha3+betha3+y3) -l*cos(betha3+y3)
         -sin(alpha4+betha4+y4) cos(alpha4+betha4+y4) l*cos(betha4+y4)];


    B = [cos(y1) 0 0 0
         0 cos(y2) 0 0
         0 0 cos(y3) 0
         0 0 0 cos(y4)];

    C = [r*cos(y1) 0 0 0
         0 r*cos(y2) 0 0
         0 0 r*cos(y3) 0
         0 0 0 r*cos(y4)];

    D = [cos(y1)
         cos(y2)
         cos(y3)
         cos(y4)];

    twist = [x;y;theta];
    dphi = pinv(C)*(A)*(twist);
    x1 = dphi(1);
    x2 = dphi(2);
    x3 = dphi(3);
    x4 = dphi(4);
end
