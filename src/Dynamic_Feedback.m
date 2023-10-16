function [x1] = Dynamic_Feedback(setpos, a)

%delete(instrfind({'Port'},{'COM4'}));

V_7805 = 5.566;
Vref_arduino = 5.05;

%a = arduino('COM4');

analogWrite(a,9,0);   % Σταματάμε τον κινητήρα.
analogWrite(a,6,0);

positionData = [];
velocityData = [];
setposData = [];
eData = [];
timeData = [];

t=0;
prevtime = 0;

% CLOSE ALL PREVIOUS FIGURES FROM SCREEN

close all

% WAIT A KEY TO PROCEED
disp('Connect cable from Arduino to Input Power Amplifier and then press enter to start controller');
pause()

position = analogRead(a,5); % Διαβάζουμε τις αρχικές τιμές. Δηλαδή την κατάσταση που βρίσκεται το σύστημα στην εκκίνηση.
velocity = analogRead(a,3);

k1 = 12;
k2 = 5.772;
ki = 9.5;
z = [];
z(1) = 0;

x1 = 3*Vref_arduino*position/1024;
x2 = 2*(2*velocity*Vref_arduino/1024- V_7805);
u = - k1*x1 - k2*x2 - ki*z(1);

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
 
	position=analogRead(a,5);
    velocity = analogRead(a,3);
    
    t=toc;
    dt = toc - prevtime;
    
    x1 = 3*Vref_arduino*position/1024;
    x2 = 2*(2*velocity*Vref_arduino/1024- V_7805);
    z(i) = z(i-1) + (x1 - setpos)*dt;
    
    prevtime = t;
    
    u = - k1*x1 - k2*x2 - ki*z(i);

	error = abs(u);

    timeData = [timeData t];
    positionData = [positionData x1];
    velocityData = [velocityData x2];
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
title('velocity')

figure
plot(timeData,eData);
title('control input')

disp('Disonnect cable from Arduino to Input Power Amplifier and then press enter to stop controller');
pause();

end
