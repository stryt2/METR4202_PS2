function [result, it, res] = track_template(im, im_template, init, ... 
    max_steps, epsilon)

if ~exist('max_steps', 'var')
    max_steps = 50;
end

if ~exist('epsilon', 'var')
    epsilon = 1e-1;
end

[gx, gy] = imgradientxy(im, 'central');

t_kp = init;
t_k = zeros(size(init));

template_size = size(im_template);
[u, v] = meshgrid(0:template_size(2) - 1, 0:template_size(1)-1);
u = u(:);
v = v(:);

I_t = im_template(:);

tplt_vec_size = size(I_t);
ones_array = ones(tplt_vec_size);
zeros_array = zeros(tplt_vec_size);

A_x = vertcat(ones_array, zeros_array);
A_y = vertcat(zeros_array, ones_array);

A = horzcat(A_x, A_y);

for it=  1:max_steps
    
    ud = u + t_kp(1);
    vd = v+ t_kp(2);
    
    r = interp2(im, ud, vd) - I_t;
    r = vertcat(r, r);
    
    dr_x = interp2(gx, ud, vd);
    dr_y = interp2(gy, ud, vd);
    
    dr = vertcat(dr_x, dr_y);
    
    w = true(size(dr));
    w(abs(dr) < 0.01) = false;
    
    t_x = t_kp(1) * ones_array;
    t_y = t_kp(2) * ones_array;
    
    t = vertcat(t_x, t_y);
    
    q = t - r ./ dr;
    
    Aw = bsxfun(@times, A, w);
    rw = q .* w;
    
    t_k = Aw\rw;
    
    t_diff = t_k - t_kp;
    
    if norm(t_diff) < epsilon
        break
    end
    
    t_kp = t_k;
end

result = t_k;
res = norm(t_diff);

end
