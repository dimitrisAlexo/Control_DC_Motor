function [x1] = Observer_Linear_Feedback(setpos, a)

%delete(instrfind({'Port'},{'COM4'}));

V_7805 = 5.566;
Vref_arduino = 5.05;

%a = arduino('COM4');

analogWrite(a,9,0);   % Σταματάμε τον κινητήρα.
analogWrite(a,6,0);

positionData = [];
obv_positionData = [];
velocityData = [];
obv_velocityData = [];
setposData = [];
eData = [];
timeData = [];

t=0;
w = pi/4;
prevtime = 0;

% CLOSE ALL PREVIOUS FIGURES FROM SCREEN

close all

% WAIT A KEY TO PROCEED
disp('Connect cable from Arduino to Input Power Amplifier and then press enter to start controller');
pause()

position = analogRead(a,5); % Διαβάζουμε τις αρχικές τιμές. Δηλαδή την κατάσταση που βρίσκεται το σύστημα στην εκκίνηση.
velocity = analogRead(a,3);

x1 = 3*Vref_arduino*position/1024;
x2 = 2*(2*velocity*Vref_arduino/1024- V_7805);

k1 = 14;
k2 = 6.32;
kr = k1;

Tm = 0.521;
km = 260.241;
ku = 1/36;
k0 = 0.2383;
kT = 0.003704;
l1 = -10;
l2 = -50;
p1 = -(l1+l2);
p2 = l1*l2;
x_obv = [];
x_obv(:, 1) = [x1 ; 0];
   
A = [0 ku*k0/kT ; 0 -1/Tm];
B = [0 ; km*kT/Tm];
C = [1 0];
L = [(p1-1/Tm) ; (p2*kT/(ku*k0) - kT*(p1-1/Tm)/(ku*k0*Tm))];

u = kr*setpos - k1*x1 - k2*0;

error = abs(u);

%START CLOCK
tic

i = 2;

while(i < 200)
    
    u = round(u / 2 * 255 / Vref_arduino);
	
	if u > 0
		analogWrite(a,6,0);
		
		motor_command = min(u,255);		% Επειδή η εντολή που δέχεται το Arduino δεν μπορεί να είναι μεγαλύτερη του 255
										% αλλά το u μπορεί να προκύψει μεγαλύτερο του 255 , τότε στέλνουμε το 255.

        if(motor_command < 45)			% Σε μικρές τιμές της τάσης δεν γυρίζει ο κινητήρας. Για αυτό επιλέγεται σαν ελάχιστη τιμή η τιμή με την οποία  μπορει να κινηθεί ο κινητήρας.
			
                motor_command = 45;
        end
		disp(motor_command)
		analogWrite(a,9,round(motor_command));

    else
        analogWrite(a,9,0);

		motor_command = min(-u , 255);
		
        if(motor_command < 45)
			
			motor_command = 45;
        end
		
        disp(motor_command)
		analogWrite(a,6,round(motor_command));
   
	end
 
	position = analogRead(a,5);
    velocity = analogRead(a,3);
    
    t=toc;
    dt = toc - prevtime;
    
    x1 = 3*Vref_arduino*position/1024;
    x2 = 2*(2*velocity*Vref_arduino/1024- V_7805);
    
    x_obv1 = x_obv(:, i-1) + (A*x_obv(:, i-1) + B*u +L*(x1 - C*x_obv(:, i-1)))*dt;
    x_obv = [x_obv x_obv1];
    
    prevtime = t;
    
    u = kr*setpos - k1*x_obv(1, i) - k2*x_obv(2, i);

	error = abs(u);
    
    t=toc;

    timeData = [timeData t];
    positionData = [positionData x1];
    obv_positionData = [obv_positionData x_obv(1, i)];
    velocityData = [velocityData x2];
    obv_velocityData = [obv_velocityData x_obv(2, i)];
    setposData = [setposData setpos];
    eData = [eData u];
    
    i = i+1;
 
end

analogWrite(a,9,0);		% Σταματάμε τον κινητήρα.
analogWrite(a,6,0);

disp('End of control Loop. Press enter to see diagramms');
pause();

figure
plot(timeData,positionData);
hold on;
plot(timeData, setposData);
title('position')

figure
plot(timeData,velocityData);
hold on;
title('velocity')

figure
plot(timeData,eData);
title('control input')

disp('Disonnect cable from Arduino to Input Power Amplifier and then press enter to stop controller');

end
