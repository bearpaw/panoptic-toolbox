function draw_pc(pc_path)
    pc = pcread(pc_path);
    [n, d] = size(pc.Location);
    fprintf('num of points %d\n', n);
    pcshow(pc);
    hold on;
    R = [1     0     0;
         0     1     0;
         0     0     1];
    cam = plotCamera('Location',[0 0 0],'Orientation',R,'Opacity', 0,  ...
        'Color', [0 1 1], 'Size', 0.3);
end
