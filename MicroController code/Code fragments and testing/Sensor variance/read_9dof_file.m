function [data] = read_9dof_file(fileNo, origin)
%reads a sensor output file and processes it into useful data
%   fileNo is an integer corresponding to file number, used to recontruct
%   file name
%   origin is either '9dof' or 'disc' (i.e analog discovery)
%   
%   if origin is '9dof', data is a 2 by 1 cell aray, which contains and x
%   by 3 array of accelerometer values, and an x by 3 array of gyroscope
%   data
%   if origin is 'disc' data is a 3 by 1 cell cintaining a x length vector of
%   axis1 data, x length vector of axis2 data, and x length vector of time
%   data, of which the MIDDLE value is 0


if strcmp(origin, '9dof')
    path = [pwd, '\Dof_readings\FILE_', num2str(fileNo), '.TXT'];
    
    fileID = fopen(path, 'r');
    
    i = 1;
    line = fgets(fileID);
    while ischar(line)
        i = i+1;
        line = fgets(fileID);
    end
    fclose(fileID);
    
    Accel = zeros(i, 3);
    Gyro = zeros(i, 3);
    
    
    fileID = fopen(path, 'r');
    i = 1;
    
    line = fgets(fileID);
    
    while ischar(line)
        C = strsplit(line, {',', '\r', '\n'});
        
        Accel(i, 1:3) = str2double(C(2:4));
        Gyro(i, 1:3)  = str2double(C(6:8));
        
        line = fgets(fileID);
        i = i+1;
    end
    fclose(fileID);
    data = {Accel, Gyro};
    %disp('Kev doubted us. Langer. ')
    
    
    
    
    
elseif strcmp(origin , 'disc')
    
    path = [pwd, '\discovery_readings\test', num2str(fileNo), '.csv'];
    fileID = fopen(path, 'r');
    
    for i = 1:6
        line = fgets(fileID);
    end
    i = 0;
    while ischar(line)
        i = i+1;
        line = fgets(fileID);
    end
    fclose(fileID);
    
    voltage = zeros(i, 1);
    time = zeros(i, 1);
    %the discovert readings - data in volts
    
    fileID = fopen(path, 'r');
    
    for i = 1:6
        line = fgets(fileID);
    end
    i = 1;
    while ischar(line)
        
        
        C = strsplit(line, {',', '\r', '\n'});
        
        time(i)  = str2double(C(1));
        voltage(i) = str2double(C(2));
        
        i = i+1;
        line = fgets(fileID);
    end
    fclose(fileID);
    
    data = {time, voltage};
    
    
end





