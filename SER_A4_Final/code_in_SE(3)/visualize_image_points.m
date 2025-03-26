function visualize_image_points(y_k_j, y, k1, k2)
    interval = k2 - k1 + 1;
    for k = 1:1:interval
        plot(squeeze(y_k_j(1,k,:)), squeeze(y_k_j(2,k,:)), 'ro');
        axis([0, 640, 0, 480]);
        grid on;
        hold on;
        plot(squeeze(y_k_j(3,k,:)), squeeze(y_k_j(4,k,:)), 'bo');
    
        % plot a line to link the left and right image features.
        line([squeeze(y_k_j(1,k,:)), squeeze(y_k_j(3,k,:))]', ...
            [squeeze(y_k_j(2,k,:)), squeeze(y_k_j(4,k,:))]', 'Color', 'g');

        plot(squeeze(y(1, k, :)), squeeze(y(2, k, :)), 'r*');
        axis([0, 640, 0, 480]);
        grid on;
        hold on;
        plot(squeeze(y(3, k, :)), squeeze(y(4, k, :)), 'b*');
    
        % plot a line to link the left and right image features.
        line([squeeze(y(1, k, :)), squeeze(y(3, k, :))]', ...
            [squeeze(y(2, k, :)), squeeze(y(4, k, :))]', 'Color', 'g');
        hold off;
        title(['k =' num2str(k+k1-1)]);
        drawnow;
    end
    xlim([-200, 1000]);
    ylim([-200, 700]);

    % draw a rectangle to represent the size of the image plane.
    x_ld = 0; % left-down coordinates
    y_ld = 0;% left-down coordinates
    width = 640;
    height = 480;
    rectangle('Position', [x_ld, y_ld, width, height], 'EdgeColor', 'r', 'LineWidth', 1, 'LineStyle','--');
end