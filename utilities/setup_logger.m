function exp_logger = setup_logger(exp_type, exp_date, cbf_specs)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

    %% Setup Logger
    exp_logger              = logger; 
    exp_logger.logMesg(sprintf(['Logger for experiment type ', num2str(exp_type) ,' on ', datestr(exp_date, 'dd-mmm-yyyy HH:MM:SS'), '\n\n']))
    exp_logger.logMesg('#### CBF SPECIFICATIONS ####')
    field_names = fieldnames(cbf_specs);
    for i=1:length(field_names)
        value = cbf_specs.(field_names{i});
        if isfloat(value)
            value = num2str(value);
        end
        exp_logger.logMesg(sprintf(['\t', field_names{i}, '= ', value]))
    end
    exp_logger.logMesg(sprintf('##########################\n'))
end

