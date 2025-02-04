% The class defines the Board thumbnails for Stereo Camera Calibrator App.

% Copyright 2020 The MathWorks, Inc.
classdef  StereoBoardThumbnails < vision.internal.calibration.webTool.BoardThumbnails
    
    methods
        function this = StereoBoardThumbnails(hParent)
            this@vision.internal.calibration.webTool.BoardThumbnails(hParent);
        end
    end
    
    methods (Access = protected)
        %------------------------------------------------------------------
        % Positioned thumbnail and it's label
        function repositionElements(this, imageNum, topLeftyx)
            
            hDataInd = this.ImageNumToDataInd(imageNum);
            hImage   = this.hImageData(hDataInd).hImage;
            
            topMargin = (this.BlockSize(1)-20 - this.ThumbnailSize(1))/2;
            topLeftyx(1) = topLeftyx(1) + topMargin;
            
            textOffset = this.SelectionPatchInset ...
                +this.ThumbnailSize(1)...
                +topLeftyx(1);
            
            % Align thumbnail (account for text space)
            hImage.YData = topLeftyx(1)...
                +this.ThumbnailSize(1) -size(hImage.CData,1);
            
            leftMargin = (this.BlockSize(1)-20 - this.ThumbnailSize(1))/2;
            hImage.XData = leftMargin;
            hImage.Visible = 'on';
            
            % Align Stereo board thumbnail labels
            textLength = this.hImageData(this.ImageNumToDataInd(imageNum)).hNameText.Extent(3);
            if textLength < this.ThumbnailSize(2)
                posx = this.SelectionPatchInset + (this.ThumbnailSize(2)-textLength)/2;
            else
                posx = topLeftyx(1);
            end
            
            posy = textOffset;
            this.hImageData(hDataInd).hNameText.Position = [posx posy];
            this.hImageData(hDataInd).hNameText.Visible = 'on';
        end
        
        %------------------------------------------------------------------
        function [thumbnail, userdata] = createThumbnail(this, imageNum)
            try
                fullImage1 = imread(this.ImageFilename{1, imageNum});
                fullImage2 = imread(this.ImageFilename{2, imageNum});
                
                % Correct thumbnail image content
                fullImage1 = this.correctDimensionOfImage(fullImage1);
                fullImage2 = this.correctDimensionOfImage(fullImage2);
                
                this.ThumbnailImageInfo(imageNum) = 1;
            catch ALL %#ok<NASGU>
                fullImage1 = this.CorruptedImagePlaceHolder;
                fullImage2 = this.CorruptedImagePlaceHolder;
            end
            
            % Resize thumbnail.
            % The Stereo Camera Calibrator App thumbnail has fixed height.
            % The width varies according to image width-height ratio.
            thumbnail = this.resizeToThumbnailStereo(fullImage1, fullImage2);
            this.ThumbnailImages{imageNum} = thumbnail;
            
            % Meta data of the full image
            userdata.class = class(fullImage1);
            userdata.size = size(fullImage1);
            userdata.isStack = false;
        end
        
        %------------------------------------------------------------------
        function thumbnailLabel = createThumbnailLabel(this, imageNum)
            % Remove filename extension if any
            % Stereo Board thumbnail displays two board name in a single string
            [~, label1, ext1] = fileparts(this.ImageFilename{1, imageNum});
            [~, label2, ext2] = fileparts(this.ImageFilename{2, imageNum});
            
            fileName1 = strcat(label1, ext1);
            fileName2 = strcat(label2, ext2);
            
            % Append 2 labels with '&' separated.
            fileName = [num2str(imageNum), ': ', fileName1, ' & ', fileName2];
            
            thumbnailLabel = fileName;
            
            updateMaxTextCharacter(this, fileName);
        end
        
        %------------------------------------------------------------------
        function widthInPixel = getThumbnailPanelwidth(this, charLength, ~)
            leftMargin = (this.BlockSize(1) - 20 - this.ThumbnailSize(1))/2;
            
            if this.hImageData(1).hNameText.Extent(3) < this.ThumbnailSize(2)
                widthInPixel = leftMargin + this.ThumbnailSize(2);
            else
                widthInPixel = leftMargin + this.MaxTextChars * charLength;
            end
        end
        
        %------------------------------------------------------------------
        function updateBoardThumbnailSize(this, imgSize)
            this.ThumbnailSize(2) = round(this.ThumbnailSize(1)*imgSize(2)/imgSize(1))*2 + 2;
        end
        
        %------------------------------------------------------------------
        function setThumbnailCount(this)
            % Left and Right camera images shown in single thumbnail.
            this.NumberOfThumbnails = numel(this.ImageFilename)/2;
        end
        
        %------------------------------------------------------------------
        function setBlockSize(this)
            % The block size should be greater than thumbnail size so
            % that some space will be visible across all the direction
            % of thumbnail.
            this.BlockSize = [96 this.hAxes.Position(3)];
        end
    end
    
    methods (Access = private)
        %------------------------------------------------------------------
        % Resize thumbnail
        function thumbnail = resizeToThumbnailStereo(thumbs, fullImage1, fullImage2)
            imageSize = thumbs.ThumbnailSize;
            imageSize(2) = imageSize(2)/2 - 1;
            image1 = im2uint8(imresize(fullImage1, imageSize));
            image2 = im2uint8(imresize(fullImage2, imageSize));
            
            thumbnail = vision.internal.calibration.tool.fuseWithSeparator(image1, image2);
        end
    end
end
