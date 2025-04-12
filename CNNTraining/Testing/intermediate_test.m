for i = 1:300

    d = extractdata(dlFeatures(:,:,i));
    if max(d, [], "all")>0.1
    
    imagesc(d); colorbar
    title(num2str(i))
    %set(gca,'ColorScale','log')

        pause(0.2)
    end

end


%%
for i = 1:262

    d = extractdata(ans(:,:,i));
    
    imagesc(d); colorbar
    title(num2str(i))
        %caxis([0 1])

        pause(0.3)
end

%%
for i = 1:13

    d = (ans(:,:,i));
    
    imagesc(d); colorbar
    title(num2str(i))
            caxis([0 1])

        pause(0.5)

end


%%
for i = 1:262

    d = mean(extractdata(ans(:,:,:,i)), 3);
    
    imagesc(d); colorbar
    title(num2str(i))
        %caxis([0 1])

        pause(0.2)
end