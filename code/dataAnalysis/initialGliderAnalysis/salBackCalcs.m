% Specify the path to your directory
directoryPath = "/Users/calebflaim/Documents/thesis/openFloat/code/dataAnalysisScripts/initialGliderAnalysis/gliderNeutralBuoyancy.csv";

data = readtable(directoryPath);
disp(data)
temp = data.temperature;
rho = data.density*1000;
press = data.pressure;
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