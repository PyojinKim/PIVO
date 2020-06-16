if (toVisualize)
    %% prerequisite to visualize
    
    p_gc_PIVO = zeros(3, imgIdx);
    for k = 1:imgIdx
        p_gc_PIVO(:,k) = T_gc_PIVO{k}(1:3,4);
    end
    
    %% update current image part
    
    axes(ha1); cla;
    imshow(imageCur, []);
    title('current image');
    
    %% update warped image part
    
    axes(ha2); cla;
    plot_warped_image(imageLast, depthLast, imageCur, T_21_final, cam.K);
    title('warped image');
    
    %% update 3D trajectory part
    
    axes(ha3); cla;
    % draw moving trajectory
    plot3( p_gc_PIVO(1,1:imgIdx), p_gc_PIVO(2,1:imgIdx), p_gc_PIVO(3,1:imgIdx), 'm', 'LineWidth', 2 ); hold on; grid on; axis equal;
    
    % draw camera body and frame
    plot_inertial_frame(0.5);
    RgcPIVO_current = T_gc_PIVO{imgIdx}(1:3,1:3);
    pgcPIVO_current = T_gc_PIVO{imgIdx}(1:3,4);
    plot_camera_frame(RgcPIVO_current, pgcPIVO_current, imageCur, 1.3, 'm'); hold off;
    refresh; pause(0.01);
    
    %% save current figure
    
    if (toSave)
        % save directory for MAT data
        SaveDir = [datasetPath '/AURO2019'];
        if (~exist( SaveDir, 'dir' ))
            mkdir(SaveDir);
        end
        
        % save directory for images
        SaveImDir = [SaveDir '/PIVO'];
        if (~exist( SaveImDir, 'dir' ))
            mkdir(SaveImDir);
        end
        
        pause(0.1); refresh;
        saveImg = getframe(h);
        imwrite(saveImg.cdata , [SaveImDir sprintf('/%06d.png',imgIdx)]);
    end
end

