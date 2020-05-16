function [] = generateCsvFilesFromBin(binFileName)

assert(strcmpi(binFileName(end-3:end), '.bin'), sprintf('''%s'' is not a bin file', binFileName));
assert(exist(binFileName, 'file') > 0, sprintf('file ''%s'' doesn''t exist', binFileName));


for logName = {'IMU', 'NKF1', 'NKF2', 'AOA', 'ARSP', 'BAT', 'RCOU'}
    log = logName{1};
    fprintf(sprintf('%s...\n', log));
    system(sprintf('mavlogdump.py --types=%s --format=csv %s > %s_data_%s.csv', ...
        log, binFileName, lower(log), binFileName(1:end-4)));
end


end