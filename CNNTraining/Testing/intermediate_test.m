for i = 1:1024

    d = extractdata(dlFeatures(:,:,i));
    if max(d, [], "all")>0.1
    
    imagesc(d); colorbar
    title(num2str(i))
    %set(gca,'ColorScale','log')

        pause(0.2)
    end

end


%%
ans=predict(obj.FeatureExtractionNet, dlX, 'Outputs',   'gn4b7_branch2c');


for i = 1:256

    d = extractdata(ans(:,:,i));
    
    imagesc(d); colorbar
    title(num2str(i))
        %caxis([0 1])

        pause(0.1)
end

%%
for i = 1:13

    d = (ans(:,:,i));
    
    imagesc(d); colorbar
    title(num2str(i))
            %caxis([0 1])

        pause(0.5)

end


%%
for i = 1:262

    d = ((ans(:,:,1,i)));
    
    imagesc(d); colorbar
    title(num2str(i))
        %caxis([0 1])

        pause(0.4)
end