function img_noShadow = shadowRemoval(img_color, theta)
    J = im2double(img_color);
    R = J(:,:,1);
    G = J(:,:,2);
    B = J(:,:,3);

    [len,wid] = size(R);
    % Generation of 2-D Log Chromaticity Image.
    for i = 1:len
        for j = 1:wid
            if ((R(i,j)*G(i,j)*B(i,j))~= 0)
                c1(i,j) = R(i,j)/((R(i,j)*G(i,j)*B(i,j))^(1/3));
                c2(i,j) = G(i,j)/((R(i,j)*G(i,j)*B(i,j))^(1/3));
                c3(i,j) = B(i,j)/((R(i,j)*G(i,j)*B(i,j))^(1/3));
            else
                c1(i,j) = 1;
                c2(i,j) = 1;
                c3(i,j) = 1;
            end
        end
    end

    rho1 = mat2gray(log(c1));
    rho2 = mat2gray(log(c2));
    rho3 = mat2gray(log(c3));

    X1 = mat2gray(rho1*1/(sqrt(2)) - rho2*1/(sqrt(2))); %(1/sqrt(2); -1/sqrt(2); 0)
    X2 = mat2gray(rho1*1/(sqrt(2)) + rho2*1/(sqrt(2)) - rho3*2/(sqrt(6))); %(1/sqrt(6); 1/sqrt(6); -2/sqrt(6))

    % theta = 15;

    img_noShadow = cos(theta*pi/180)*X1 + sin(theta*pi/180)*X2;

end