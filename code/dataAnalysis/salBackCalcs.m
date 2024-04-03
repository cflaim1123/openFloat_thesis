% Specify the path to your directory
colvos1 = "/Users/calebflaim/Documents/thesis/openFloat_thesis/data/intermediateData/buoyancyDataCsv/colvos1/gliderNeutralBuoyancy.csv";
colvos2 = "/Users/calebflaim/Documents/thesis/openFloat_thesis/data/intermediateData/buoyancyDataCsv/colvos2/gliderNeutralBuoyancy.csv";
shilshole1 = "/Users/calebflaim/Documents/thesis/openFloat_thesis/data/intermediateData/buoyancyDataCsv/shilshole1/gliderNeutralBuoyancy.csv";
shilshole2 = "/Users/calebflaim/Documents/thesis/openFloat_thesis/data/intermediateData/buoyancyDataCsv/shilshole2/gliderNeutralBuoyancy.csv";
shilshole3 = "/Users/calebflaim/Documents/thesis/openFloat_thesis/data/intermediateData/buoyancyDataCsv/shilshole3/gliderNeutralBuoyancy.csv";
data_path = '/Users/calebflaim/Documents/thesis/openFloat_thesis/data/tankData3.csv'

% press = [1046.38,1046.52,1046.85,1046.95,1047.02,1047.06,1047.4,1047.19,1047.26,1047.36,1047.39,1047.46,1047.8,1048.14,1048.17, 1047.93];
% press = press/1000;
% temp = [21.2672,21.3372,21.3722,21.4247,21.4596,21.4771,21.5121,21.5471,21.5821,21.6346,21.6521,21.6871,21.722,21.757,21.7745,21.792];
% rho = [1023.91,1023.91,1023.91,1023.91,1023.91,1023.91,1023.91,1023.91,1023.91,1023.91,1023.91,1023.91,1023.91,1023.91,1023.91,1023.91];
% data = readtable(colvos2);
data = readtable(data_path);
% disp(data)
% rho = data.density*1000;
press = (data.pressure-1014.5)/100;%right pressure
temp = data.temperature-0.16281972144169288; %uncorrected temp
rho = zeros(length(temp),1)+1023.5493224359292;%rho calculated with corected sal
SA = gsw_SA_from_rho_t_exact(rho, temp, press)

% Get a list of files in the directory
% files = dir(fullfile(directoryPath, '*.csv'));

% Display the names of the files
% disp('List of CSV files in the directory:');
% for i = 1:length(files)
%     disp(files(i).name);
% end
% 
% % Read CSV files into tables
% dataTables = cell(1, length(files));

% for i = 1:length(files)
%     filePath = fullfile(directoryPath, files(i).name);
%     dataTables{i} = readtable(filePath);
%     table = dataTables{i};
%     % Display the table for each CSV file
%     disp(['Table for ' files(i).name ':']);
%     disp(dataTables{i});
% 
%     temp = table.temperature;
%     dens = table.density;
%     press = table.pressure;
% 
%     SA = gsw_SA_from_rho_t_exact(dens, temp, press);
%     % disp(SA)
% 
%     write_cols = ["time", "pressure", "absolute_salinity"];
%     write_table = table(:, write_cols);
%     write_table.("calculated_sal") = SA;
% 
%     disp(write_table)
% 
%     write_path = "~/Documents/toAddToThesis/salinityEstimationVals/SG175_dive_"+string(i)+"_sal_estimation.csv";
%     writetable(write_table, write_path);
%     
% end