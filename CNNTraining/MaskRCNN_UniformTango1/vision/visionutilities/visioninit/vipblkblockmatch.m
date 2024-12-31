function varargout = vipblkblockmatch(action, which2img)
    % VIPBLKOPTICALFLOW Video Processing Blockset Block Matching mask helper function.

    % Copyright 1995-2019 The MathWorks, Inc.
    %  
    %if nargin==0, action = 'dynamic'; end

    blkh = gcbh;
    blk = gcb;
    disp_str='';
    si=[];
    so=[];
    dtInfo = [];
    switch action
    case 'icon'
      isBlkInLibrary = strcmp(get_param(bdroot(blkh),'BlockDiagramType'),'library');

      disp_str = sprintf('Block\nMatching');

      puOUTVELFORM      = get_param(blkh,'outVelForm');
      if strncmp(puOUTVELFORM,'Hori ..',2)
          so(1).port = 1;
          so(1).txt = 'V';
      else
          so(1).port = 1;
          so(1).txt = '|V|^2';
      end
      useDelayBlk = (which2img==2);
      if useDelayBlk % only one input port
         si(1).port = 1;
         si(1).txt = 'I'; 
         si(2).port = 1;
         si(2).txt = 'I';
      else % two input port
         si(1).port = 1;
         si(1).txt = 'I1'; 
         si(2).port = 2;
         si(2).txt = 'I2';
      end

      varargout(1) = {disp_str};  
      varargout(2) = {si};  
      varargout(3) = {so};  

      %% change the subsystem if necessary
      OFE_blk = [blk,'/block match'];
      if useDelayBlk 
         if exist_block(blk, 'In2')%% second inpt port blk exists
             w = warning;
             warning('off');

             delete_line_frmOFE_2ndinport_to2ndInPort(blk);
             delete_second_inport(blk);
             add_delay_block(blk);
             set_param([blk,'/Delay'], 'DelayLength', 'N');
             add_line_frm_1stInport_to_dlyInport(blk);
             add_line_frm_dlyOutport_to_OFE_2ndInport(blk); 
             warning(w);
         end
      else
         if exist_block(blk, 'Delay')% delay block exists
             w = warning;
             warning('off');

             delete_line_frm_1stInport_to_DlyInPort(blk);
             delete_line_frm_dlyOutport_to_OFE_2ndInport(blk);
             delete_delay_block(blk);
             add_second_inport(blk);
             add_line_frmOFE_2ndinport_to2ndInPort(blk);

             warning(w);
         end
      end
    case 'init'
        % output = varargout = {dtInfo}
        % num = misc(1)
            % output H (2)
            % accum(4)
            % prodOutput(8)
        lower_blk = [blk,'/block match'];
        puORcbParams = {'searchMethod','matchCriteria','outVelForm','outputMode','accumMode','prodOutputMode','roundingMode','overflowMode'};
        for i=1:length(puORcbParams)
            BLKpuORcb = get_param(blk, puORcbParams{i});
            LBLKpuORcb = get_param(lower_blk, puORcbParams{i});
            if ~strcmp(BLKpuORcb,LBLKpuORcb)
                set_param(lower_blk, puORcbParams{i}, BLKpuORcb);
            end
        end 
        dynamicBlockUpdate(blk);
        
        case 'dynamic'
            dynamicBlockUpdate(blk);

    end % end of switch statement
end

function delete_line_frmOFE_2ndinport_to2ndInPort(blk)
  delete_line(blk,'In2/1','block match/2');
end
 
function delete_second_inport(blk)  
  SecondPort_blk = [blk,'/In2'];
  delete_block(SecondPort_blk);
end
    
function add_delay_block(blk)
  load_system('simulink');
  add_block('simulink/Discrete/Delay',[blk,'/Delay'],'position',[125    72   165    98]);
end

function add_line_frm_1stInport_to_dlyInport(blk)
  add_line(blk,'In1/1','Delay/1');
end

function add_line_frm_dlyOutport_to_OFE_2ndInport(blk)
  add_line(blk,'Delay/1','block match/2');
end

function delete_line_frm_1stInport_to_DlyInPort(blk) 
  delete_line(blk,'In1/1','Delay/1');
end

function delete_line_frm_dlyOutport_to_OFE_2ndInport(blk) 
  delete_line(blk,'Delay/1','block match/2');
end

function delete_delay_block(blk) 
  delete_block([blk,'/Delay']);
end
  
function add_second_inport(blk) 
  add_block('built-in/Inport',[blk,'/In2'],'position',[25    78    55    92]);
end
  
function add_line_frmOFE_2ndinport_to2ndInPort(blk) 
  add_line(blk,'In2/1','block match/2');
end
         
         
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function present = exist_block(sys, name)
    present = ~isempty(find_system(sys,'searchdepth',1,...
        'followlinks','on','lookundermasks','on','name',name));
end

function dynamicBlockUpdate(blk)
    oldVisibilities = get_param(blk, 'MaskVisibilities');
    newVisibilities = oldVisibilities;
    maskNames    = get_param(blk,'MaskNames');
    blkMask = Simulink.Mask.get(gcb);
    
    %which2img
    which2imgIdx = find(strcmp(maskNames(:),'which2img'));
    newVisibilities{which2imgIdx} = 'on';
    %N
    which2imgValue = get_param(blk, 'which2img');
    NIdx = find(strcmp(maskNames(:),'N'));
    if(strcmp(which2imgValue, 'Current frame and N-th frame back'))
        newVisibilities{NIdx} = 'on';
    else
        newVisibilities{NIdx} = 'off';
    end
    %Search method
    searchMethodIdx = find(strcmp(maskNames(:),'searchMethod'));
    newVisibilities{searchMethodIdx} = 'on';
    %Block matching criteria
    matchCriteriaIdx = find(strcmp(maskNames(:),'matchCriteria'));
    newVisibilities{matchCriteriaIdx} = 'on';
    %blockSize
    blockSizeIdx = find(strcmp(maskNames(:),'blockSize'));
    newVisibilities{blockSizeIdx} = 'on';
    %overlapping
    overlappingIdx = find(strcmp(maskNames(:),'overlapping'));
    newVisibilities{overlappingIdx} = 'on';
    %Maximum displacement (search region)
    maxDisplacementIdx = find(strcmp(maskNames(:),'maxDisplacement'));
    newVisibilities{maxDisplacementIdx} = 'on';

    outVelFormValue = get_param(blk, 'outVelForm');
    isOutVelMagsq = strncmp(outVelFormValue,'Magnitude-squared',3);
    matchCriteriaValue = get_param(blk, 'matchCriteria');
    isMatchCriteriaMAD = strncmp(matchCriteriaValue,'Mean absolute difference', 7);
    
    
    accumMode = get_param(blk, 'accumMode');
    outputMode = get_param(blk, 'outputMode');
    prodOutputMode = get_param(blk, 'prodOutputMode');

    accumWLengthIdx = find(strcmp(maskNames(:),'accumWordLength'));
    accumFLengthIdx = find(strcmp(maskNames(:),'accumFracLength'));

    outputWLengthIdx = find(strcmp(maskNames(:),'outputWordLength'));
    outputFLengthIdx = find(strcmp(maskNames(:),'outputFracLength'));

    prodOutputModeIdx = find(strcmp(maskNames(:),'prodOutputMode'));
    prodOutputWLengthIdx = find(strcmp(maskNames(:),'prodOutputWordLength'));
    prodOutputFLengthIdx = find(strcmp(maskNames(:),'prodOutputFracLength'));


    accumSignedText = blkMask.getDialogControl('accumSignedText');
    outputSignedText = blkMask.getDialogControl('outputSignedText');
    prodOutputLabel = blkMask.getDialogControl('prodOutputLabel');
    prodOutputSignedText = blkMask.getDialogControl('prodOutputSignedText');
    SignedColumnLabel = blkMask.getDialogControl('SignedColumnLabel');
    WordLengthColumnLabel =  blkMask.getDialogControl('WordLengthColumnLabel');
    FracLengthColumnLabel =  blkMask.getDialogControl('FractionLengthColumnLabel');

    

    signedColumnLabelVisibility = 'off';
    wordLengthColumnLabelVisibility = 'off';
    fracLengthColumnLabelVisibility = 'off';
    
    if isMatchCriteriaMAD
        newVisibilities{prodOutputModeIdx} = 'off';
        prodOutputLabel.Visible = 'off';
        prodOutputSignedText.Visible = 'off';
        newVisibilities{prodOutputWLengthIdx} = 'off';
        newVisibilities{prodOutputFLengthIdx} = 'off';
    else
        if strcmp(prodOutputMode, 'Binary point scaling')
            prodOutputSignedText.Visible = 'on';
            newVisibilities{prodOutputWLengthIdx} = 'on';
            newVisibilities{prodOutputFLengthIdx} = 'on';
            signedColumnLabelVisibility = 'on';
            wordLengthColumnLabelVisibility = 'on';
            fracLengthColumnLabelVisibility = 'on';   
        else
            %prodOutputSignedText.Visible = 'off';
            newVisibilities{prodOutputWLengthIdx} = 'off';
            newVisibilities{prodOutputFLengthIdx} = 'off';
        end
    end
    
    
    if strcmp(accumMode, 'Binary point scaling')
        accumSignedText.Visible = 'on';
        newVisibilities{accumWLengthIdx} = 'on';
        newVisibilities{accumFLengthIdx} = 'on';
        signedColumnLabelVisibility = 'on';
        wordLengthColumnLabelVisibility = 'on';
        fracLengthColumnLabelVisibility = 'on';   
    else
        accumSignedText.Visible = 'off';
        newVisibilities{accumWLengthIdx} = 'off';
        newVisibilities{accumFLengthIdx} = 'off';
    end
    

    if strcmp(outputMode, 'Binary point scaling')
        outputSignedText.Visible = 'on';
        newVisibilities{outputWLengthIdx} = 'on';
        signedColumnLabelVisibility = 'on';
        wordLengthColumnLabelVisibility = 'on';
        fracLengthColumnLabelVisibility = 'on';
    else
        outputSignedText.Visible = 'off';
        newVisibilities{outputWLengthIdx} = 'off';
        newVisibilities{outputFLengthIdx} = 'off';
    end
    
    if(isOutVelMagsq)
        outputSignedText.Prompt = 'Simulink:dialog:no_CB';
    else
        outputSignedText.Prompt = 'Simulink:dialog:yes_CB';
    end
        
        
    SignedColumnLabel.Visible = signedColumnLabelVisibility;
    WordLengthColumnLabel.Visible =  wordLengthColumnLabelVisibility;
    FracLengthColumnLabel.Visible =  fracLengthColumnLabelVisibility;
    
    if ~isequal(newVisibilities,oldVisibilities)
        set_param(blk, 'MaskVisibilities', newVisibilities);
    end

end

% [EOF]



