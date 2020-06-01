% save logged data

% prompt user for file name prefix
prefix = input('(Optional) Enter a prefix for MAT file (output: <prefix>_log_DATE.mat: ','s');

% save dir
date_now = datestr(now, 'yyyymmdd_HHMMSS');
current_dir = pwd;
save_dir = [current_dir, '/logs/'];

% save data
if ~isempty(prefix)
    file_name = strcat(save_dir, prefix,'_log_', date_now,'.mat');
else
    file_name = strcat(save_dir,'log_', date_now,'.mat');
end
save(file_name, 'pr', 'model', 'index', '-regexp', '^log');
fprintf('[%s] Saved log data to %s \n', datestr(now, 'HH:MM:SS'), file_name);


clear current_dir save_dir prefix file_name
