function data = getLogData(logName)

assert(strcmpi(logName(end-3:end), '.csv'), sprintf('''%s'' is not a csv', logName));
assert(exist(logName, 'file') > 0, sprintf('file ''%s'' doesn''t exist', logName));

fprintf(sprintf('Reading ''%s''...\n', logName));

d = importdata(logName, ',', 1);
for i=1:numel(d.colheaders)
    data.(d.colheaders{i}) = d.data(:,i);
end

end