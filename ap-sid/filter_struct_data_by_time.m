function s = filter_struct_data_by_time(s, t0 ,t1)

idx = s.time >= t0 & s.time <= t1;

fnames = fieldnames(s);
for i=1:length(fnames)
    s.(fnames{i}) = s.(fnames{i})(idx);
end

end