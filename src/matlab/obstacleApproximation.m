function patchSet = obstacleApproximation(obsinfo, approx_num, inflate_alpha)
    if size(obsinfo,2) ~= 3
        error('Dimension of the obstacle information should be nx3')
    end
    
    patchSet = {};
    
    obs_x = obsinfo(:,1);
    obs_y = obsinfo(:,2);
    if size(inflate_alpha) == size(obsinfo(:,3))
        obs_r = obsinfo(:,3).*inflate_alpha;        
    else
        error(['The size of the inflation vector should be ' num2str(size(obsinfo(:,3)))])
    end
    
    approx_sine = zeros(approx_num, size(obsinfo,1));
    approx_cosine = zeros(approx_num, size(obsinfo,1));
    k = zeros(approx_num, size(obsinfo,1));
    b = zeros(approx_num, size(obsinfo,1));
    
    for i = 1:size(obsinfo,1)
        approx_sine(:,i) = sin((2*pi*[1:approx_num])./approx_num)';
        approx_cosine(:,i) = cos((2*pi*[1:approx_num])./approx_num)';
        k(:,i) = -approx_cosine(:,i)./approx_sine(:,i);
        b(:,i) = obs_r(i)./approx_sine(:,i);
        
        intersect = zeros(2, approx_num);
        for j = 1:approx_num
             if abs(k(j,i)) > 1e5
                [px, py] = lineIntersection(k(rem(j, approx_num)+1, i), b(rem(j, approx_num)+1, i), k(j, i), b(j, i));
             else
                [px, py] = lineIntersection(k(j, i), b(j, i), k(rem(j, approx_num)+1, i), b(rem(j, approx_num)+1, i));
             end
             intersect(:,j) = [px + obs_x(i); py + obs_y(i)];
        end
        patchSet(end+1) = {intersect};
    end
end