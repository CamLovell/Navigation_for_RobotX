function map = generateMapNE(map)

%Extract non-zero map cells
eastpts = map.east(:).';
eastpts = eastpts(ones(length(map.north),1),:);
northpts = map.north(:);
northpts = northpts(:,ones(1,length(map.east)));

idx = find(map.Z);
map.eastpts = eastpts(idx);
map.northpts = northpts(idx);
