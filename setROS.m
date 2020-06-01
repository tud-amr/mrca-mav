% ROS setup for MATLAB

%% For a sinlge local computer
ROS_MASTER_IP = 'localhost';
ROS_IP = 'localhost';
setenv('ROS_MASTER_URI', ['http://',ROS_MASTER_IP,':11311']);
setenv('ROS_IP', ROS_IP);
try
    fprintf('[%s] ROS: Attempting ROS init... \n',datestr(now,'HH:MM:SS'));
    rosinit;
catch
    fprintf('[%s] ROS: ROS will be reinitialised \n',datestr(now,'HH:MM:SS'));
    rosshutdown
    rosinit;
end
