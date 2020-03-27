folders = {'data_robust','data_non_robust'};
plot_titles = {'Robust CBF Experiment','Regular CBF Experiment'};

for i_fold = 1:numel(folders)
    folder_name = folders{i_fold};
    myFiles = dir(strcat(folder_name,'/*.mat'));
    h = figure(i_fold); % set(h,'Visible', 'off');
    %   title(plot_titles{i_fold});
    hold on
    shift = 0;
    labels = {};
    time_violated = 0;
    for i = 1:numel(myFiles)
        data = load(strcat(folder_name,'/',myFiles(i).name));
        N = data.N;
        h_min = data.min_hs_h;
        comp_time = data.comp_h;
        iters = data.iters;
        if iters > numel(h_min)
            iters = numel(h_min);
        end
        comp_time_stats = [mean(comp_time),var(comp_time)];
        plot(((1:iters)+shift-1)*.033,h_min(1:iters),'black','linewidth',2)
        shift = shift + iters;
        labels{i} = strcat('Trial-',num2str(i));
        time_violated = time_violated + sum(h_min<0)*0.033;
    end
    plot((1:shift)*.033,zeros(1,shift),'r--','linewidth',2)
    %legend(labels)
    set(gca,'fontsize',50,'linewidth',2);
    axis tight;
    ylim([-0.01,0.1]);
    xlabel('Time (s)');
    ylabel('min(h_i_j(t))');
    disp(strcat('Average Qp sol time:', num2str(comp_time_stats(1))))
    disp(strcat('Variance Qp sol time:', num2str(comp_time_stats(2))))
    savefig(folder_name);
    hold off
    disp(time_violated);
end
